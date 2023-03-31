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

    PriorityGroup(): n_eqs(0), n_ineqs(0), s(solve_type::Pinv){}

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
               Eigen::MatrixXd& new_projector){
      if (n_eqs == 0) {new_solution=solution;new_projector=projector;return;}

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_eqs, projector.cols());
        Eigen::MatrixXd b = Eigen::MatrixXd::Zero(n_eqs, 1);

        auto it_B = B_vec.begin();
        auto it_b = b_vec.begin();
        int n = 0;
        while (it_B != B_vec.end()) {
          Eigen::MatrixXd B_ = *(*it_B).get();
          Eigen::MatrixXd b_ = *(*it_b).get();
          
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
          double damping = 0;
          u = (- B.transpose() * (B * B.transpose() + damping*Eigen::MatrixXd::Identity(B.rows(),B.rows())).inverse() * b);
        } else {
        // create a QP solver pointer
        //std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> IK_qp_solver_ptr_ = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
        //std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(n_vars, 1, 1));

        // create cost function
        Eigen::MatrixXd H = projector.transpose()*B.transpose()*B*projector;
        Eigen::VectorXd F = 2 * projector.transpose()*B.transpose()*(b+B*solution);

        // input constraints
        Eigen::MatrixXd jointLimConstrMatrix = 0*Eigen::MatrixXd::Identity(n_vars, n_vars);
        Eigen::VectorXd jointLimUpperBound = 0*10 * Eigen::VectorXd::Ones(n_vars);
        Eigen::VectorXd jointLimLowerBound = -0*10 * Eigen::VectorXd::Ones(n_vars);
        
        // dummy equality constraint
        Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Zero(1,n_vars);
        Eigen::VectorXd b_dummy = Eigen::VectorXd::Zero(1);
      

        std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> IK_qp_solver_ptr_ = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
            std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(n_vars, 1, n_vars));

        IK_qp_solver_ptr_->solve(
            H,
            F,
            A_dummy,
            b_dummy,
            jointLimConstrMatrix,
            jointLimLowerBound,
            jointLimUpperBound
        );
        u = (IK_qp_solver_ptr_->get_solution());

      }
      new_solution = solution + projector*u; //
      // find null space
      Eigen::BDCSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullV);
      new_projector = projector * svd.matrixV().rightCols(B.cols() - svd.rank());
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
    static constexpr int max_priority_levels = 4;
    static constexpr int n_vars = _n_vars;

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
      std::cout << "priority: "<< current_priority << " ";
      PriorityGroup<n_vars> &group = groups.at(current_priority);

      if (group.s == solve_type::Pinv) 
        std::cout << "PINV\n";
      else
        std::cout << "QP\n";

      Eigen::MatrixXd solution;
      Eigen::MatrixXd projector;

      group.solve(prev_solution, prev_projector, solution, projector);
      // std::cout << solution.transpose() << "\n";
      // last priority: no tasks further down
      if (current_priority == max_priority_levels) return solution;
      
      else {
        return _solve(projector, solution, current_priority+1 );
      }
    }

    void set_solve_type(int priority, solve_type s) {
      PriorityGroup<n_vars> &group = groups.at(priority);
      std::cout << "before...";
      if (group.s == solve_type::Pinv) 
        std::cout << "PINV\n";
      else
        std::cout << "QP\n";
      group.set_solve_type(s);
      std::cout << "after...";
      if (group.s == solve_type::Pinv) 
        std::cout << "PINV\n";
      else
        std::cout << "QP\n";
    }

    std::array<PriorityGroup<_n_vars>,max_priority_levels+1> groups;

  };

} // namespace hrc