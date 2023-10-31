#pragma once
#include <cmath>
#include "Eigen/Dense"
#include <stdio.h>
#include <vector>
#include <string>
#include <math.h>
#include "Sensor_data.h"
#include <iostream>
using Eigen::VectorXf;

using Eigen::MatrixXd;

using Eigen::MatrixXf;
using Eigen::RowVectorXf;
class Pose
{
    public:
    Pose(float _x_pos, float _y_pos, float _angle);

    void set_position(float new_x_pos, float new_y_pos, float new_angle);

    VectorXf estimate_new_pose(float vel, float steer_angle, float dt, float lr,
        float lf);

    Eigen::VectorXf F(VectorXf &, VectorXf &, float vel, float dt, float steer_angle);
    MatrixXf G(float vel, float dt, float steer_angle, float lr, float lf);
    MatrixXf h(float x_pos, float y_pos);
    MatrixXf H(float dx, float dy, float q);
    MatrixXf Q_k();

    void SLAM(float vel, float steer_angle, float dt,float lr,float lf);


    //int data_assocation(float x_pos, float y_pos, float radius, VectorXf pred_mu);

    const VectorXf& get_mu();

    void calculate_mean(float sum_x, float sum_y);
    void localization(float vel, float steer_angle, float dt,float lr,float lf);



    int get_number_of_landmarks();

    std::vector<int> get_landmarks();

    int get_x();
    int get_y();
    int get_ang();

    Sensor_data sens_data;
private:
    int amount_landm{0};
    VectorXf obs_mu;
    MatrixXf cov, R;

};
