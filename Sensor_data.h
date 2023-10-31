#pragma once

#include <vector>
#include "Eigen/Dense"
#include "kod/rplidar_sdk-master/sdk/sdk/include/rplidar.h"
#include "Landmark.h"

///////////////////////////////////////////////////////////////////////
#include <SFML/Graphics.hpp>
//////////////////////////////////////////////////////////////////////

using Eigen::VectorXf;
using Eigen::RowVectorXf;

class Sensor_data
{
    public:
    Sensor_data();

    bool read_data(rp::standalone::rplidar::RPlidarDriver* lidar);

    void convert_data(float time_per_measure, float time_since_SLAM,
        float velocity, float steer_angle, float lr, float lf, 
        sf::RenderWindow & SLAM_window);

    bool circle_regression(std::vector<float> xdata, std::vector<float> ydata,
        Landmark guess, Landmark & result);

    float root_mean_square_error(std::vector<float> xdata, std::vector<float> ydata,
        Landmark circle);
        
    int data_assocation(float lx_pos, float ly_pos, float cx_pos, float cy_pos);

    std::vector<int> get_landmark_info();
    
    std::vector<float> rdata;
    VectorXf obs_mu, mu_info, mu;
    std::vector<float> thetadata;
    
    int obs_landm{};


};
