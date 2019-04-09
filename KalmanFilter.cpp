#include <iostream>
#include <stdexcept>

#include "KalmanFilter.hpp"

namespace estimation {

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& stateTransitionMatrix,
    const Eigen::MatrixXd& observationMatrix,
    const Eigen::MatrixXd& processNoiseCovariance,
    const Eigen::MatrixXd& measurementNoiseCovariance,
    const Eigen::MatrixXd& initialEstimationErrorCovariance)
  : m_stateTransitionMatrix(stateTransitionMatrix),
    m_observationMatrix(observationMatrix),
    m_processNoiseCovariance(processNoiseCovariance),
    m_measurementNoiseCovariance(measurementNoiseCovariance),
    m_estimationErrorCovariance(initialEstimationErrorCovariance),
    m_numMeasurements(observationMatrix.rows()),
    m_numStates(stateTransitionMatrix.rows()), m_samplingTime(dt), isInitialized(false),
    m_identityMatrix(m_numStates, m_numStates), m_estimatedState(m_numStates),
    m_predictedState(m_numStates)
{
    m_identityMatrix.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(const Eigen::VectorXd& initState) {
    m_estimatedState = initState;
    isInitialized = true;
}

void KalmanFilter::init() {
    m_estimatedState.setZero();
    isInitialized = true;
}

void KalmanFilter::predict(void) {
    if(!isInitialized)
        throw std::runtime_error("Filter is not initialized!");
    m_predictedState = m_stateTransitionMatrix * m_estimatedState;
}

void KalmanFilter::correct(const Eigen::VectorXd& measurement) {
    if(!isInitialized)
        throw std::runtime_error("Filter is not initialized!");
    m_estimationErrorCovariance = m_stateTransitionMatrix * m_estimationErrorCovariance *
                                  m_stateTransitionMatrix.transpose() + m_processNoiseCovariance;
    Eigen::MatrixXd kalmanGain = m_estimationErrorCovariance * m_observationMatrix.transpose() *
                 (m_observationMatrix * m_estimationErrorCovariance * m_observationMatrix.transpose() +
                     m_measurementNoiseCovariance).inverse();
    m_predictedState += kalmanGain * (measurement - m_observationMatrix*m_predictedState);
    m_estimationErrorCovariance = (m_identityMatrix - kalmanGain*m_observationMatrix)*m_estimationErrorCovariance;
    m_estimatedState = m_predictedState;
}

void KalmanFilter::setMeasurementNoiseCovariance(const Eigen::MatrixXd& measurementNoiseCovariance)
{
    m_measurementNoiseCovariance = measurementNoiseCovariance;
}

Eigen::VectorXd KalmanFilter::getState(){
    return m_estimatedState;
}

}
