# Kalman Filter

## Introduction
This is a basic implementation of Kalman Filter in C++. The implemented filter has been tested on a linear time invariant

system. The filter is composed of two prediction and correction steps. In the prediction step, The system model is used to

estimate the current state from the previous state. The correction step uses the measurement to reduce the estimation uncertainty.

- An introduction to Kalman Filter [by TonyLacey](http://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf)
- An introduction to system discretization [wiki page](https://en.wikipedia.org/wiki/Discretization)

## Configuring CodeCoverage
Inside the UnitTest build directory

* cmake -DCMAKE_BUILD_TYPE=Debug ..
* make
* make TestKalmanFilter_coverage

* By making these files the corresponding code coverage folder will be created. 

* Open the report with index.html file in the generated code coverage folders in the build folder.

## How to Use
* mkdir build
* cd build
* cmake ..
* make
* ./TestKalmanFilter

You need to use DataHandling.py to generate dataset and visualize the results.
