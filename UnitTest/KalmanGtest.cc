#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <string>
#include <fstream>
#include "../kalman.h"

struct Statistics{
    double mean;
    double variance;
};

class KalmanFilterTest : public ::testing::Test{
protected:

    int n = 2; // Number of states
    int m = 1; // Number of measurements
    double dt = 1.0/1000.; // Time step
    KalmanFilter* KF;

    std::vector<double> observation_vec, originalData_vec;
    std::string observ_address = "../../observation.txt";
    std::string origData_address = "../../original_data.txt";

    void ReadData(const std::string& address, std::vector<double>& data_vec){
        std::string line;
        std::ifstream myfile(address);
        if(myfile.is_open())
        {
            while(getline(myfile, line))
            {
                data_vec.push_back(stod(line));
            }
            myfile.close();
        }
    }

    void SetUp() override {
        Eigen::MatrixXd A(n, n); // System dynamics matrix
        Eigen::MatrixXd C(m, n); // Output matrix
        Eigen::MatrixXd Q(n, n); // Process noise covariance
        Eigen::MatrixXd R(m, m); // Measurement noise covariance
        Eigen::MatrixXd P(n, n); // Estimate error covariance

        A << 1., dt, -dt*31.4*31.4, 1.;
        C << 1, 0;

        Q << 0.0001 , 0.001, 0.001, 0.0001;
        R << 0.01;
        P << 0., 0., 0., 0.;

        KF = new KalmanFilter(dt,A, C, Q, R, P);

        ReadData(observ_address, observation_vec);
        ReadData(origData_address, originalData_vec);
    }

    Statistics CalculateStatistics(const std::vector<double>& vec1,
                                   const std::vector<double>& vec2)
    {
        std::vector<double> diff_vec;
        for(size_t i=0; i<vec1.size(); ++i)
        {
            diff_vec.push_back(vec1[i] - vec2[i]);
        }

        Statistics stats;
        stats.mean = std::accumulate(diff_vec.begin(), diff_vec.end(), 0.0) / diff_vec.size();

        double var = 0;
        for (size_t i=0; i<diff_vec.size(); ++i)
        {
            var += (diff_vec[i] - stats.mean) * (diff_vec[i] - stats.mean);
        }
        var /= diff_vec.size();
        stats.variance = var;

        return stats;
    }

    void TearDown() override {
        delete KF;
    }

};


TEST_F(KalmanFilterTest, CheckEstimationError)
{
    Eigen::VectorXd x_init(n);
    x_init << observation_vec[0], 0.;
    KF->init(dt, x_init);

    std::vector<double> filterOutput;
    Eigen::VectorXd y(m);
    for(const auto& obs : observation_vec)
    {
        y << obs;
        KF->predict();
        KF->correct(y);
        filterOutput.push_back(KF->state()[0]);
    }

    Statistics observ_stats = CalculateStatistics(observation_vec, originalData_vec);
    Statistics kalman_stats = CalculateStatistics(filterOutput, originalData_vec);

    ASSERT_TRUE(kalman_stats.variance < observ_stats.variance);
}




int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
