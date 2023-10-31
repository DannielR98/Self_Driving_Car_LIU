#pragma once

#include <pthread.h>
#include <mutex>
#include <vector>
#include "path_point.h"

extern float glob_velocity;
extern float glob_steer_angle;
extern float glob_reference_speed;
extern float glob_lf;
extern float glob_lr;
extern std::vector<int> glob_landmarks;
extern std::vector<int> glob_landmark_info;
extern int glob_x;
extern int glob_y;
extern int glob_ang;
extern bool glob_exit;
extern std::vector<path_point> glob_path_points;
extern std::vector<point> glob_port_points;
extern bool glob_manual_mode;
extern float glob_e_fa;
extern bool glob_recalculate_path;
extern float glob_stanley_k;
extern uint8_t glob_next_port_index;
extern std::mutex mtx;

