//
// Copyright (c) 2018 CNRS
//
// This file is part of pinocchio-minimal
// pinocchio-minimal is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include <iostream>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/sample-models.hpp>

int main(int argc, char *argv[]) {
  using namespace pinocchio;
  Model model;
  buildModels::humanoid(model);

  Data data(model);

  // Run CRBA from random configuration
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::MatrixXd M = crba(model, data, q);

  // Symmetrize M
  M.triangularView<Eigen::StrictlyLower>() =
      M.transpose().triangularView<Eigen::StrictlyLower>();

  std::cout << "Joint space inertia matrix:\n" << M << std::endl;

  return -1;
}
