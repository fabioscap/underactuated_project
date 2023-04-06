#include "task.hpp"

void hrc::PriorityGroup::solve(const Eigen::MatrixXd& solution,
                               const Eigen::MatrixXd& projector,
                               Eigen::MatrixXd& new_solution,
                               Eigen::MatrixXd& new_projector) {
  if (n_eqs == 0) {new_solution=solution;new_projector=projector;return;}
  if (n_ineqs > 0 && s == hrc::solve_type::Pinv) {
    std::cout << "Solving with PINV but there are inequalities... \n";
  }

  // EQUALITY MATRICES
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_eqs, projector.cols());
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n_eqs, 1);

  auto it_B = B_vec.begin();
  auto it_b = b_vec.begin();
  int n = 0;
  while (it_B != B_vec.end()) {
    Eigen::MatrixXd B_ = *(*it_B).get();
    Eigen::VectorXd b_ = *(*it_b).get();
    
    // project constraints in the null space of the previous priorities
    B.block(n,0,B_.rows(), projector.cols()) = B_*projector;
    b.block(n,0,b_.rows(), b_.cols()) = b_ - B_*solution;

    ++it_B; ++it_b; n+= B_.rows();
  }

  // INEQUALITY MATRICES
  Eigen::MatrixXd C;
  Eigen::VectorXd upper;
  Eigen::VectorXd lower;

  if (n_ineqs > 0) {

    C = Eigen::MatrixXd::Zero(n_ineqs, projector.cols());
    upper = Eigen::VectorXd::Zero(n_ineqs, 1);
    lower = Eigen::VectorXd::Zero(n_ineqs, 1);

    auto it_C = C_vec.begin();
    auto it_upper = upper_vec.begin();
    auto it_lower = lower_vec.begin();
    n = 0;
    while (it_C != C_vec.end()) {
      Eigen::MatrixXd C_ = *(*it_C).get();
      Eigen::VectorXd upper_ = *(*it_upper).get();
      Eigen::VectorXd lower_ = *(*it_lower).get();
      
      // project constraints in the null space of the previous priorities
      C.block(n,0,C_.rows(), projector.cols()) = C_*projector;
      upper.block(n,0,upper_.rows(), upper_.cols()) = upper_- C_*solution;
      lower.block(n,0,lower_.rows(), lower_.cols()) = lower_- C_*solution;

      ++it_C; ++it_lower; ++it_upper; n+= C_.rows();
    }
  } else {
    // dummy matrices to put in the solver
    C = Eigen::MatrixXd::Zero(1, projector.cols());
    upper = Eigen::VectorXd::Zero(1);
    lower = Eigen::VectorXd::Zero(1);
  }

  Eigen::VectorXd u;
  if (s == solve_type::Pinv) { 
    double damping = 1e-18;
    u = (B.transpose() * (B * B.transpose() + damping*Eigen::MatrixXd::Identity(B.rows(),B.rows())).inverse() * b);
  } else {

    // create cost function
    double reg = 1e-6;
    Eigen::MatrixXd H = B.transpose()*B + reg*projector.transpose()*projector;
    Eigen::VectorXd F = -B.transpose()*b;
    
    Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Zero(1,projector.cols());
    Eigen::VectorXd b_dummy = Eigen::VectorXd::Zero(1);

    std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> IK_qp_solver_ptr_ = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
        std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(projector.cols(), A_dummy.rows(), C.rows()));

    IK_qp_solver_ptr_->solve(
        H,
        F,
        A_dummy,
        b_dummy,
        C, // C
        lower,   // l
        upper    // u
        // C >= l
        // C <= u
    );
    u = (IK_qp_solver_ptr_->get_solution());
  }
  new_solution = solution + projector*u; //
  // find null space
  Eigen::BDCSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullV);
  new_projector = projector * svd.matrixV().rightCols(B.cols() - svd.rank());
  if (new_projector.isZero(0)) {
    std::cout << "hai finito i dof\n";
    
  } else {
    std::cout << "next priority space: " << new_projector.cols()<<"\n";
  }
} 


void hrc::PriorityGroup::add_eq_constr(const Eigen::MatrixXd& B,
                                       const Eigen::MatrixXd& b) {
  assert(B.rows() == b.rows());
  B_vec.push_back(std::make_shared<Eigen::MatrixXd>(B));
  b_vec.push_back(std::make_shared<Eigen::MatrixXd>(b));
  n_eqs += B.rows();

}

void hrc::PriorityGroup::add_ineq_constr(const Eigen::MatrixXd &C, const Eigen::VectorXd &l, const Eigen::VectorXd &u){
  assert(C.rows() == l.rows() && C.rows() == u.rows());
  C_vec.push_back(std::make_shared<Eigen::MatrixXd>(C));
  lower_vec.push_back(std::make_shared<Eigen::MatrixXd>(l));
  upper_vec.push_back(std::make_shared<Eigen::MatrixXd>(u));
  n_ineqs += C.rows();
}

void hrc::HierarchicalSolver::add_ineq_contstr(const Eigen::MatrixXd &C, const Eigen::MatrixXd &lower, const Eigen::MatrixXd &upper, unsigned int priority) {
  // the inequality at priority r is also an equality for priority > r
  unsigned int r = priority;

  while (r <= max_priority_level) {
    groups.at(r).add_ineq_constr(C,lower,upper);
    ++r;
  }
}

Eigen::MatrixXd hrc::HierarchicalSolver::solve() {
  Eigen::MatrixXd projector = Eigen::MatrixXd::Identity(n_vars,n_vars);
  Eigen::MatrixXd solution  = Eigen::MatrixXd::Zero(n_vars, 1);

  return _solve(projector, solution, 0);
}

Eigen::MatrixXd hrc::HierarchicalSolver::_solve(const Eigen::MatrixXd& prev_projector,
                        const Eigen::MatrixXd& prev_solution,
                        int current_priority) {
  std::cout << "priority: "<< current_priority << "\n";
  hrc::PriorityGroup &group = groups.at(current_priority);

  Eigen::MatrixXd solution;
  Eigen::MatrixXd projector;

  group.solve(prev_solution, prev_projector, solution, projector);
  // std::cout << solution.transpose() << "\n";
  // last priority: no tasks further down
  if (current_priority == max_priority_level) return solution;
  else return _solve(projector, solution, current_priority+1 );
  
}