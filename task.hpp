#pragma once

#include <Eigen/Core>
#include <vector>
#include <memory>
#include "types.hpp"

namespace hrc {

  struct EQConstr {
    EQConstr(const Eigen::MatrixXd& A, 
             const Eigen::MatrixXd &b): 
             n_vars(A.cols()), n_constraints{A.rows()}, A(A), b(b) {
             }

    const int n_vars;
    const int n_constraints; 
    const Eigen::MatrixXd& A;
    const Eigen::MatrixXd& b;
  };

  // group many Constrs with the same priority
  template <int _n_vars>
  struct PriorityLevel {
    static constexpr int n_vars = _n_vars;

    PriorityLevel(int priority): n_eq{0}, priority{priority} {}

    void add_eq(const Eigen::MatrixXd&A, const Eigen::MatrixXd&b) {
      EQConstr t = EQConstr(A,b);
      n_eq += t.n_constraints;
      std::cout << "A at add \n" << t.A << std::endl;
      eq_list.push_back(t);
    }

    

    void solve(Matrixd<n_vars,1>& solution, 
               Eigen::Matrix<double, n_vars, Eigen::Dynamic>& Z){
      if (1) {
        // pinv
        // see if we can bound the dimensions 
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_eq, n_vars);
        Eigen::MatrixXd b = Eigen::MatrixXd::Zero(n_eq, 1);
        
        int n = 0;
        auto it = eq_list.begin();
        while (it != eq_list.end()) {
          EQConstr t = *it;
          // put the constraint in the matrix
          std::cout << "\n\n\nA\n" << t.A <<"\n";
          B.block(n,0,t.A.rows(),t.A.cols()) = t.A;
          b.block(n,0,t.b.rows(),t.b.cols())  = t.b;
          n += t.n_constraints;
          ++it;
        }
        std::cout << "B\n" << B << "\nb\n" << b << "\n";
        solution = - B.transpose() * (B * B.transpose()).inverse() * b;
        auto svd = Eigen::BDCSVD<Eigen::MatrixXd>(B, Eigen::ComputeFullV) ;
        Z =  svd.matrixV().rightCols(B.cols() - svd.rank());

      } else {
        // QP TODO
      }
      
    }

    void solve(const Eigen::Matrix<double, n_vars,Eigen::Dynamic,0,n_vars,n_vars>& Z) { 
      // Z: null space projector for tasks with lower priority
      
    }


    // number of equality constraints
    int n_eq; 


    int priority;

    // store the list of constraints
    std::vector<EQConstr> eq_list;
  };




} // end namespace hrc