#pragma once

#include <Eigen/Core>
#include <vector>
#include <memory>
#include "types.hpp"
#include "utils.cpp"

namespace hrc {

  template <int _n_vars>
  struct PriorityGroup {
    static constexpr int n_vars = _n_vars;

    PriorityGroup(): n_eqs{0}{}

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
      if (1==1) { // pinv
        //print_shape("previous level projector", projector);
        //print_shape("previous solution", solution);
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
        new_solution = solution + projector*(- B.transpose() * (B * B.transpose()).inverse() * b); //

        //print_shape("u", (- B.transpose() * (B * B.transpose()).inverse() * b)); 
        // find null space
        Eigen::BDCSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullV);
        new_projector = projector * svd.matrixV().rightCols(B.cols() - svd.rank());

      } else {
        // QP
      }
    }

    std::vector<std::shared_ptr<Eigen::MatrixXd>> B_vec;
    std::vector<std::shared_ptr<Eigen::MatrixXd>> b_vec;
    int n_eqs;
  };


  template <int _n_vars>
  struct HierarchicalSolver {
    static constexpr int max_priority_levels = 3;
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
      // std::cout << current_priority << "\n";
      PriorityGroup<n_vars> group = groups.at(current_priority);

      Eigen::MatrixXd solution;
      Eigen::MatrixXd projector;

      group.solve(prev_solution, prev_projector, solution, projector);

      // last priority: no tasks further down
      if (current_priority+1 == max_priority_levels) return solution;
      
      else {
      auto x = _solve(projector, solution, current_priority+1 );
      return x;
      }
    }

    std::array<PriorityGroup<_n_vars>,max_priority_levels> groups;

  };

} // end namespace hrc