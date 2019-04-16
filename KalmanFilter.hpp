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
#ifndef __ESTIMATION_KALMANFILTER_H_
#define __ESTIMATION_KALMANFILTER_H_

#include <Eigen/Dense>
#include <gtest/gtest_prod.h>

/*!
 * @namespace estimation Kalman filter implementation for state estimation
 */
namespace estimation {

class KalmanFilter {
    friend class KalmanFilterTest;

public:
    /*!
     * @brief Specialized constructor for Kalman Filter
     *
     * @param stateTransitionMatrix : State transition matrix
     * @param observationMatrix : System output matrix
     * @param processNoiseCovariance : Process noise covariance
     * @param measurementNoiseCovariance : Measurement noise covariance
     * @param estimationErrorCovariance : Estimation error covariance
     *
     * @return An instance of KalmanFilter
     */
    KalmanFilter(
        const Eigen::MatrixXd& stateTransitionMatrix,
        const Eigen::MatrixXd& observationMatrix,
        const Eigen::MatrixXd& processNoiseCovariance,
        const Eigen::MatrixXd& measurementNoiseCovariance,
        const Eigen::MatrixXd& estimationErrorCovariance
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
     * @param initState : Initial state
     */
    void init(const Eigen::VectorXd& initState);

    /*!
     * @brief Using the system model and state vector
     *  from the previous step, predicts the current
     *  state vector.
     */
    void predict();

    /*!
     * @brief Using the system output measurements, corrects
     * the predicted state vector
     *
     * @param measurement : Measurement vector
     */
    void correct(const Eigen::VectorXd& measurement);

    /*!
     * @brief Assigns a new value for the measurement covariance
     *
     * @param measurementNoiseCovariance : New measurement covariance
     */
    void setMeasurementNoiseCovariance(const Eigen::MatrixXd& measurementNoiseCovariance);

    /*!
     * @brief Returns the current state vector
     *
     * @return Eigen::VectorXd : State vector
     */
    Eigen::VectorXd getState();

private:
    Eigen::MatrixXd m_stateTransitionMatrix; ///< State transition matrix
    Eigen::MatrixXd m_observationMatrix; ///< Observation matrix
    Eigen::MatrixXd m_processNoiseCovariance; ///< Process noise covariance
    Eigen::MatrixXd m_measurementNoiseCovariance; ///< Measurement noise covariance
    Eigen::MatrixXd m_estimationErrorCovariance; ///< Estimation error covariance

    int m_numMeasurements; ///< Number of measurements
    int m_numStates; ///< Number of states

    Eigen::MatrixXd m_identityMatrix; ///< n-size identity

    Eigen::VectorXd m_predictedState; ///< Predicted states
    Eigen::VectorXd m_estimatedState; ///< Corrected states

    bool isInitialized; ///< Indicates if the filter is initialized

    FRIEND_TEST(KalmanFilterTest, CheckPrediction);
    FRIEND_TEST(KalmanFilterTest, CheckInitWithoutInputParams);
    FRIEND_TEST(KalmanFilterTest, CheckPredictWithWrongMatrixDimension);
    FRIEND_TEST(KalmanFilterTest, CheckGettingState);
    FRIEND_TEST(KalmanFilterTest, CheckSettingMeasurementNoiseCovariance);
    FRIEND_TEST(KalmanFilterTest, CheckCorrect);
};

} // namespace estimation

#endif // __ESTIMATION_KALMANFILTER_H_
