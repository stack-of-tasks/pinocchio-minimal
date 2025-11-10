# pinocchio-minimal

[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/stack-of-tasks/pinocchio-minimal/master.svg)](https://results.pre-commit.ci/latest/github/stack-of-tasks/pinocchio-minimal/master)

Minimal project using Pinocchio as Rigid Body Dynamics library.
This simple example shows how to link Pinocchio to a dedicated application.

## Build instructions:
```sh
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/main # to test
```
