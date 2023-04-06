#include "task.hpp"

void hrc::PriorityGroup::solve(const Eigen::MatrixXd& solution,
                               const Eigen::MatrixXd& projector,
                               Eigen::MatrixXd& new_solution,
                               Eigen::MatrixXd& new_projector) {
  if (n_eqs == 0) {new_solution=solution;new_projector=projector;return;}

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
    double reg = 1e-6;
    Eigen::MatrixXd H = B.transpose()*B + reg*projector.transpose()*projector;
    Eigen::VectorXd F = B.transpose()*b;

    // input constraints
    Eigen::MatrixXd jointLimConstrMatrix = 0*Eigen::MatrixXd::Identity(projector.cols(), projector.cols());
    Eigen::VectorXd jointLimUpperBound = 0*10 * Eigen::VectorXd::Ones(projector.cols());
    Eigen::VectorXd jointLimLowerBound = -0*10 * Eigen::VectorXd::Ones(projector.cols());
    
    //Eigen::MatrixXd A_dummy = B;
    //Eigen::VectorXd b_dummy = -b;
    Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Zero(1,projector.cols());
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
  }
  new_solution = solution + projector*u; //
  // find null space
  Eigen::BDCSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullV);
  new_projector = projector * svd.matrixV().rightCols(B.cols() - svd.rank());
  std::cout << "next priority space: " << new_projector.cols()<<"\n";
} 


void hrc::PriorityGroup::add_eq_constr(const Eigen::MatrixXd& A,
                                       const Eigen::MatrixXd& b) {
  assert(A.rows() == b.rows());
  B_vec.push_back(std::make_shared<Eigen::MatrixXd>(A));
  b_vec.push_back(std::make_shared<Eigen::MatrixXd>(b));
  n_eqs += A.rows();

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
  if (current_priority == max_priority_levels) return solution;
  
  else {
    return _solve(projector, solution, current_priority+1 );
  }
}

void hrc::HierarchicalSolver::set_solve_type(int priority, solve_type s) {
  hrc::PriorityGroup &group = groups.at(priority);
  group.set_solve_type(s);
}