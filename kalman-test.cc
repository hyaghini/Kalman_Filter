/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <math.h>

#include "kalman.h"

int main(int argc, char* argv[]) {
    std::vector<double> observation_vec;
    std::string line;
    std::ifstream myfile ("../observation.txt");
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            observation_vec.push_back(stod(line));
//            std::cout << line << std::endl;
        }
        myfile.close();
    }


  int n = 2; // Number of states
  int m = 1; // Number of measurements

  double dt = 1.0/200.; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
//  A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
//  C << 1, 0, 0;
  A << 1., dt, -dt*31.4, 1.;
  C << 1, 0;

  // Reasonable covariance matrices
//  Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
//  R << 5;
//  P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
//  Q << pow(dt, 4)/4., pow(dt,3)/2., pow(dt,3)/2., pow(dt,2);
  Q << 0.0001 , 0.001, 0.001, 0.0001;
  R << 0.01;
  P << 0., 0., 0., 0.;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt,A, C, Q, R, P);


  // Best guess of initial states
  Eigen::VectorXd x0(n);
  x0 << observation_vec[0], 0;
  kf.init(dt,x0);

  // Feed measurements into filter, output estimated states
  double t = 0;
  Eigen::VectorXd y(m);
  std::vector<double> kalman_vec;
//  std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
  for(int i = 0; i < observation_vec.size(); i++) {
      if ((400 < i)  && (i < 700))
      {
          R << 1000.;
          kf.setR(R);
          y << 0.;
      }
      else
      {
          R << 0.01;
          kf.setR(R);
          y << observation_vec[i];
      }
    t += dt;
    kf.predict();
    kf.correct(y);
    kalman_vec.push_back(kf.state()[0]);
//    std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
//        << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
  }

  std::ofstream writefile("../FilterOutput.txt");
  if (writefile.is_open())
  {
      for(const auto& x : kalman_vec)
      {
          writefile << x << "\n";
      }
      writefile.close();
  }


  return 0;
}
