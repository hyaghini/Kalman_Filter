/*!
 * **************************************************
 * @file KalmanFilter.h
 * @brief This class implements Kalman Filter
 *
 * KalmanFilter class uses the system model (defined by the
 * state space matrices) to predict the current state of the
 * system and uses measurements to correct the predicted states.
 *
 * @section  LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * https://www.gnu.org/copyleft/gpl.html
 *
 * @author Hamed Yaghini (hamed84.yaghini@gmail.com)
 * @date 04/05/2019
 *
 * ****************************************************
 */

#include <Eigen/Dense>

#pragma once

class KalmanFilter {

public:
    /*!
     * @brief Specialized constructor for Kalman Filter
     *
     * @param A : State transition matrix
     * @param C : System output matrix
     * @param Q : Process noise covariance
     * @param R : Measurement noise covariance
     * @param R : Estimation error covariance
     *
     * @return An instance of KalmanFilter
     */
    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P
    );

    /*!
     * @brief Default constructor for KalmanFilter
     *
     * @return An instance of KalmanFilter
     */
    KalmanFilter();

    /*!
     * @brief Initializes the filter by the state vector with
     * zero values
     *
     */
    void init();

    /*!
     * @brief Initializes the filter with a guess for
     * state vector
     *
     * @param t0 : Initial time
     * @param x0 : Initial state
     */
    void init(double t0, const Eigen::VectorXd& x0);

    /*!
     * @brief Using the system model and state vector
     *  from the previous step, predicts the current
     *  state vector.
     */
    void predict();

    /*!
     * @brief Using the system output measurements, corrects
     * the predicted state vector
     */
    void correct(const Eigen::VectorXd& y);

    /*!
     * @brief Assigns a new value for the measurement covariance
     *
     * @param R : New measurement covariance
     */
    void setR(const Eigen::MatrixXd& R);

    /*!
     * @brief Returns the current state vector
     *
     * @return Eigen::VectorXd : State vector
     */
    Eigen::VectorXd state() { return x_hat; };

    /*!
     * @brief Returns system time
     *
     * @return double : Current system time
     */
    double time() { return t; };

private:
    Eigen::MatrixXd A; ///< State transition matrix
    Eigen::MatrixXd C; ///< Observation matrix
    Eigen::MatrixXd Q; ///< Process noise covariance
    Eigen::MatrixXd R; ///< Measurement noise covariance
    Eigen::MatrixXd P; ///< Estimation error covariance
    Eigen::MatrixXd K; ///< Kalman gain
    Eigen::MatrixXd P0; ///< Initial estimation error covariance

    int m; ///< Number of measurements
    int n; ///< Number of states

    double t0; ///< Initial time
    double t; ///< Current time

    double dt; ///< Sampling time

    bool initialized; ///< Indicates if the filter is initialized

    Eigen::MatrixXd I; ///< n-size identity

    Eigen::VectorXd x_hat_new; ///< Predicted states
    Eigen::VectorXd x_hat; ///< Corrected states
};
