#pragma once

#include <Eigen/Core>
#include <vector>
#include <memory>
#include "types.hpp"
#include "utils.cpp"
#include <labrob_qpsolvers/qpsolvers.hpp>

namespace hrc {

  enum class solve_type {Pinv, QP};

  template <int _n_vars>
  struct PriorityGroup {
    static constexpr int n_vars = _n_vars;

    PriorityGroup(): n_eqs(0), n_ineqs(0), s(solve_type::QP){}

    template <int n_constraints>
    void add_eq_constr(const Eigen::Matrix<double, n_constraints, n_vars>& A,
                       const Eigen::Matrix<double, n_constraints, 1>& b) {
      B_vec.push_back(std::make_shared<Eigen::MatrixXd>(A));
      b_vec.push_back(std::make_shared<Eigen::MatrixXd>(b));
      n_eqs += n_constraints;
    }

    void solve(const Eigen::MatrixXd& solution,
               const Eigen::MatrixXd& projector,
               Eigen::MatrixXd& new_solution,
               Eigen::MatrixXd& new_projector,
               Eigen::MatrixXd& q_ref,
               Eigen::MatrixXd& q_meas,
               Eigen::MatrixXd& q_dot){ // TODO remove these if no longer necessary
      if (n_eqs == 0) {new_solution=solution;new_projector=projector;return;}

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_eqs, projector.cols());
        Eigen::VectorXd b = Eigen::VectorXd::Zero(n_eqs, 1);

        auto it_B = B_vec.begin();
        auto it_b = b_vec.begin();
        int n = 0;
        while (it_B != B_vec.end()) {
          Eigen::MatrixXd B_ = *(*it_B).get();
          Eigen::VectorXd b_ = *(*it_b).get();
          
          //print_shape("B_",B_);
          //print_shape("b_",b_);
          // multiply matrices for projector

          B.block(n,0,B_.rows(), projector.cols()) = B_*projector;
          b.block(n,0,b_.rows(), b_.cols()) = b_+ B_*solution;
          ++it_B; ++it_b; n+= B_.rows();
        }
        // std::cout << "b " << b.transpose() << std::endl;
        // std::cout << "solving"<< std::endl;
        Eigen::VectorXd u;
        if (s == solve_type::Pinv) { 
          double damping = 1e-18;
          u = (- B.transpose() * (B * B.transpose() + damping*Eigen::MatrixXd::Identity(B.rows(),B.rows())).inverse() * b);
        } else {

        // create cost function
        print_shape("proj",projector);
        print_shape("B", B);
        print_shape("b", b);
        print_shape("sol", solution);
        //Eigen::MatrixXd H = 1e-6 * Eigen::MatrixXd::Identity(projector.cols(), projector.cols());;
        double reg = 1e-6;
        Eigen::MatrixXd H = B.transpose()*B + reg*projector.transpose()*projector;
        Eigen::VectorXd F = B.transpose()*b;

        //H+= B.transpose()*B;
        //F+= B.transpose()*b;
        //H = H.setZero();
        //F = F.setZero();

        // input constraints
        Eigen::MatrixXd jointLimConstrMatrix = 0*Eigen::MatrixXd::Identity(projector.cols(), projector.cols());
        Eigen::VectorXd jointLimUpperBound = 0*10 * Eigen::VectorXd::Ones(projector.cols());
        Eigen::VectorXd jointLimLowerBound = -0*10 * Eigen::VectorXd::Ones(projector.cols());
        
        //Eigen::MatrixXd A_dummy = B;
        //Eigen::VectorXd b_dummy = -b;
        Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Zero(1,n_vars);
        Eigen::VectorXd b_dummy = Eigen::VectorXd::Zero(1);

        std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> IK_qp_solver_ptr_ = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
            std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(projector.cols(), A_dummy.rows(), projector.cols()));

        IK_qp_solver_ptr_->solve(
            H,
            F,
            A_dummy,
            b_dummy,
            jointLimConstrMatrix, // C
            jointLimLowerBound,   // l
            jointLimUpperBound    // u
            // C >= l
            // C <= u
        );
        u = (IK_qp_solver_ptr_->get_solution());
        std::cout << u.transpose() << "\n";
      }
      new_solution = solution + projector*u; //
      // find null space
      Eigen::BDCSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullV);
      new_projector = projector * svd.matrixV().rightCols(B.cols() - svd.rank());
      std::cout << "next priority space: " << new_projector.cols()<<"\n";
    }

    void set_solve_type(solve_type st){
      s = st;
    }

    // equalities
    std::vector<std::shared_ptr<Eigen::MatrixXd>> B_vec;
    std::vector<std::shared_ptr<Eigen::MatrixXd>> b_vec;
    int n_eqs;
    //

    // inequalities


    int n_ineqs;
    //

    solve_type s;
  };


  template <int _n_vars>
  struct HierarchicalSolver {
    static constexpr int max_priority_levels = 5;
    static constexpr int n_vars = _n_vars;

    // TODO remove these if no longer necessary
    HierarchicalSolver(Eigen::MatrixXd q_ref_, Eigen::MatrixXd q_meas_, Eigen::MatrixXd q_dot_): q_ref(q_ref_), q_meas(q_meas_), q_dot(q_dot_){}

    template <int n_constraints>
    void add_eq_constr(const Eigen::Matrix<double, n_constraints, n_vars>& A,
                       const Eigen::Matrix<double, n_constraints, 1>& b,
                       unsigned int priority) {
      groups.at(priority).add_eq_constr(A, b);
    }

    Matrixd<n_vars,1> solve() {
      Eigen::MatrixXd projector = Eigen::MatrixXd::Identity(n_vars,n_vars);
      Eigen::MatrixXd solution  = Eigen::MatrixXd::Zero(n_vars, 1);

      return _solve(projector, solution, 0);
    }

    Eigen::MatrixXd _solve(const Eigen::MatrixXd& prev_projector,
                           const Eigen::MatrixXd& prev_solution,
                           int current_priority) {
      std::cout << "priority: "<< current_priority << "\n";
      PriorityGroup<n_vars> &group = groups.at(current_priority);

      Eigen::MatrixXd solution;
      Eigen::MatrixXd projector;

      group.solve(prev_solution, prev_projector, solution, projector, q_ref, q_meas, q_dot);
      // std::cout << solution.transpose() << "\n";
      // last priority: no tasks further down
      if (current_priority == max_priority_levels) return solution;
      
      else {
        return _solve(projector, solution, current_priority+1 );
      }
    }

    void set_solve_type(int priority, solve_type s) {
      PriorityGroup<n_vars> &group = groups.at(priority);
      group.set_solve_type(s);

    }

    std::array<PriorityGroup<_n_vars>,max_priority_levels+1> groups;
    Eigen::MatrixXd q_ref, q_meas, q_dot;
  };

} // namespace hrc