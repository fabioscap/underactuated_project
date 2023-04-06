#pragma once

#include <Eigen/Core>
#include <vector>
#include <memory>
#include "types.hpp"
#include "utils.cpp"
#include <labrob_qpsolvers/qpsolvers.hpp>

namespace hrc {

  enum class solve_type {Pinv, QP};

  struct PriorityGroup {

    PriorityGroup(): n_eqs(0), n_ineqs(0), s(solve_type::QP) {}

    // Bx = b
    void add_eq_constr(const Eigen::MatrixXd& A,
                       const Eigen::MatrixXd& b);

    void add_ineq_constr(const Eigen::MatrixXd& C,
                         const Eigen::VectorXd& lower,
                         const Eigen::VectorXd& upper);

    void solve(const Eigen::MatrixXd& solution,
               const Eigen::MatrixXd& projector,
                     Eigen::MatrixXd& new_solution,
                     Eigen::MatrixXd& new_projector);

    inline void set_solve_type(solve_type st){ s = st;}

    // equalities
    std::vector<std::shared_ptr<Eigen::MatrixXd>> B_vec;
    std::vector<std::shared_ptr<Eigen::MatrixXd>> b_vec;
    int n_eqs;
    //

    // inequalities
    std::vector<std::shared_ptr<Eigen::MatrixXd>> C_vec;
    std::vector<std::shared_ptr<Eigen::MatrixXd>> lower_vec;
    std::vector<std::shared_ptr<Eigen::MatrixXd>> upper_vec;
    int n_ineqs;
    //

    solve_type s;
  };

  struct HierarchicalSolver {
    static constexpr int max_priority_level = 5;

    HierarchicalSolver(int _n_vars): n_vars(_n_vars){}

    inline void add_eq_constr(const Eigen::MatrixXd& B,
                              const Eigen::MatrixXd& b,
                              unsigned int priority) {groups.at(priority).add_eq_constr(B, b);}

    void add_ineq_contstr(const Eigen::MatrixXd& C,
                          const Eigen::MatrixXd& lower,
                          const Eigen::MatrixXd& upper,
                          unsigned int priority);

    Eigen::MatrixXd solve();

    Eigen::MatrixXd _solve(const Eigen::MatrixXd& prev_projector,
                           const Eigen::MatrixXd& prev_solution,
                           int current_priority);

    inline void set_solve_type(int priority, solve_type s) {groups.at(priority).set_solve_type(s);};

    const int n_vars;
    std::array<PriorityGroup,max_priority_level+1> groups;
  };

} // namespace hrc