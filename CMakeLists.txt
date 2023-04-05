#
# Copyright (c) 2018 CNRS
#
# This file is part of pinocchio-minimal pinocchio is free software: you can
# redistribute it and/or modify it under the terms of the GNU Lesser General
# Public License as published by the Free Software Foundation, either version 3
# of the License, or (at your option) any later version. pinocchio is
# distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE. See the GNU General Lesser Public License for more
# details. You should have received a copy of the GNU Lesser General Public
# License along with pinocchio If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.6)

set(PROJECT_NAME pinocchio-minimal)
set(PROJECT_DESCRIPTION
    "Minimal example of a simple application depending on Pinocchio")
set(PROJECT_URL "http://github.com/stack-of-tasks/pinocchio-minimal")

# pinocchio dependency
find_package(pinocchio 2.3.0 REQUIRED)

# Create the main executable
add_executable(main main.cpp)

# Link main to its dependencies
target_link_libraries(main PUBLIC pinocchio::pinocchio)
