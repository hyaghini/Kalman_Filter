#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <string>
#include <fstream>
#include "../KalmanFilter.hpp"

namespace estimation
{

class KalmanFilterTest : public ::testing::Test
{
protected:
    double m_samplingTime = 1. / 1000;

    Eigen::MatrixXd m_stateTransitionMatrix; // System dynamics matrix
    Eigen::MatrixXd m_observationMatrix; // Output matrix
    Eigen::MatrixXd m_processNoiseCovariance; // Process noise covariance
    Eigen::MatrixXd m_measurementNoiseCovariance; // Measurement noise covariance
    Eigen::MatrixXd m_estimationErrorCovariance; // Estimate error covariance
    Eigen::VectorXd m_initialState;

    void SetUp() override {

        Eigen::MatrixXd stateTransitionMatrix(2,2); // System dynamics matrix
        Eigen::MatrixXd observationMatrix(1,2); // Output matrix
        Eigen::MatrixXd processNoiseCovariance(2,2); // Process noise covariance
        Eigen::MatrixXd measurementNoiseCovariance(1,1); // Measurement noise covariance
        Eigen::MatrixXd estimationErrorCovariance(2,2); // Estimate error covariance
        Eigen::VectorXd initialState(2,1);

        stateTransitionMatrix << 1., m_samplingTime,
                                   -m_samplingTime*31.4*31.4, 1.;
        observationMatrix << 1, 0;

        processNoiseCovariance << 0.0001 , 0.001, 0.001, 0.0001;
        measurementNoiseCovariance << 0.01;
        estimationErrorCovariance << 0., 0., 0., 0.;

        initialState << 0., 1.;

        m_stateTransitionMatrix = stateTransitionMatrix;
        m_observationMatrix = observationMatrix;
        m_processNoiseCovariance = processNoiseCovariance;
        m_measurementNoiseCovariance = measurementNoiseCovariance;
        m_estimationErrorCovariance = estimationErrorCovariance;
        m_initialState = initialState;

    }

};

TEST_F(KalmanFilterTest, CheckPrediction)
{
    KalmanFilter KF(m_stateTransitionMatrix,
                    m_observationMatrix,
                    m_processNoiseCovariance,
                    m_measurementNoiseCovariance,
                    m_estimationErrorCovariance);
    KF.init(m_initialState);
    KF.predict();
    ASSERT_EQ(KF.m_predictedState, m_stateTransitionMatrix * m_initialState);



}

TEST_F(KalmanFilterTest, CheckEmptyInit)
{
    Eigen::VectorXd zeroVector(2);
    zeroVector.setZero();
    KalmanFilter KF(m_stateTransitionMatrix,
                    m_observationMatrix,
                    m_processNoiseCovariance,
                    m_measurementNoiseCovariance,
                    m_estimationErrorCovariance);
    KF.init();
    KF.predict();

    ASSERT_EQ(KF.m_predictedState, zeroVector);
}

TEST_F(KalmanFilterTest, CheckMatrixDimension)
{
    Eigen::MatrixXd A(3,3);
    A << 1., 0., 0.,
         0., 1., 0.,
         0., 0., 1.;
    m_stateTransitionMatrix = A;

    KalmanFilter KF(m_stateTransitionMatrix,
                    m_observationMatrix,
                    m_processNoiseCovariance,
                    m_measurementNoiseCovariance,
                    m_estimationErrorCovariance);
    KF.init(m_initialState);
    KF.predict();
}

TEST_F(KalmanFilterTest, Check_getState)
{
    Eigen::VectorXd measurement(1);
    measurement << 1.;

    KalmanFilter KF(m_stateTransitionMatrix,
                    m_observationMatrix,
                    m_processNoiseCovariance,
                    m_measurementNoiseCovariance,
                    m_estimationErrorCovariance);
    KF.init(m_initialState);
    KF.predict();
    KF.correct(measurement);

    ASSERT_EQ(KF.getState(), KF.m_estimatedState);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

}
