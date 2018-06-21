# Trajectory Optimization Library

## Setup

1) Create a new directory outside of this repo where you would like your dependencies installed, copy setup.sh to it, give it execution permissions, and run it in the empty directory.

2) Copy [this file](cmake/LocalProperties.cmake.sample) to cmake/LocalProperties.cmake and paste the output "Range include directory" path in place of the FIXME.

3) Make a directory called `build/` inside `src/`, and run `cmake ..` and then `make`. `ctest` is to run tests and `./trajectoryOptimization` to run the main routine. `make` is to recompile at any time.