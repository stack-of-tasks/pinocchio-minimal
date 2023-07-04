#include <iostream>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/sample-models.hpp>

int main(int argc, char *argv[]) {
  using namespace pinocchio;
  Model model;
  buildModels::humanoid(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(+1.);

  Data data(model);

  // Run CRBA from random configuration
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::MatrixXd M = crba(model, data, q);

  // Symmetrize M
  M.triangularView<Eigen::StrictlyLower>() =
      M.transpose().triangularView<Eigen::StrictlyLower>();

  std::cout << "Joint space inertia matrix:\n" << M << std::endl;

  return 0;
}
