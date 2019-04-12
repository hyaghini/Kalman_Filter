# Kalman Filter

## Introduction
This is a basic implementation of Kalman Filter in C++. The implemented filter has been tested on a linear time invariant

system. The filter is composed of two prediction and correction steps. In the prediction step, The system model is used to

estimate the current state from the previous state. The correction step uses the measurement to reduce the estimation uncertainty.

- An introduction to Kalman Filter [by Tony Lacey](http://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf)
- An introduction to system discretization [Wiki Page](https://en.wikipedia.org/wiki/Discretization)

========================================

## Configuring CodeCoverage
Inside the UnitTest build directory

* cmake -DCMAKE_BUILD_TYPE=Debug ..
* make
* make TestKalmanFilter_coverage

* By making these files the corresponding code coverage folder will be created. 

* Open the report with index.html file in the generated code coverage folders in the build folder.

========================================

## How to Use
* go to the project directory
* mkdir build
* cd build
* cmake ..
* make
* ./TestKalmanFilter

========================================

## Using KalmanFilter Class
#### Define the filter with state space matrices:
*  estimation::KalmanFilter kf(dt,A, C, Q, R, P);

where

* Eigen::MatrixXd A(n, n); is System dynamics matrix
* Eigen::MatrixXd C(m, n); is Output matrix
* Eigen::MatrixXd Q(n, n); is Process noise covariance
* Eigen::MatrixXd R(m, m); is Measurement noise covariance
* Eigen::MatrixXd P(n, n); is the initial Estimate error covariance

#### Initialize the filter with the initial state:
* kf.init(x0);

where

* Eigen::VectorXd x0(n) is the guessed initial state

#### Predict and Correct:
At each sampling time, there are two estimation steps:
* kf.predict(); prediction step that estimates the state vector using the previous state and state transition matrix.
* kf.correct(y); Correction step which corrects the prediction step using the measurements.

For an example of using KalmanFilter class take a look at the TestKalmanFilter.cpp

The system in the example is the discretized version of a linear oscillator.

###UnitTest:
A Googletest was designed to test the class members of the Kalman filter. There are several options to test the private members of the class.

A brief discussion about these methods can be found [here](PrivateMemberTest.md).
