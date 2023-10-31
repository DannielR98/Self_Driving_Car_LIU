#include <math.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include "stanley.h"
#include "globala_variabler.h"
#include "gottochblandat.h"
#include "path_point.h"
#include "Pose.h"

// ATT GÖRA: SE TILL ATT SÄTTA glob_next_port_index

//int theta{0}; //Bilens vinkel (Hämtas från bilens position)
 //Banans vinkel
//int theta_e{glob_ang - theta_p}; //Vinkelfelet
int last_point{};
float last_point_distance; //Ortogonalt fel (Hämtas från punkt och bilens postion eller färdig från main för vägplanering)
int lr{140};
int lf{140};
float k{0};
float e_fa{};

int delta(int theta_e, float velocitet);
path_point closest_point_to_car(std::vector<path_point> points);
std::vector<path_point> points{};
VectorXf estimate_new_pose(float vel, float steer_angle, float dt, float lr, float lf);
float sinc_ish(float th, float w, float dt);
float cosc_ish(float th, float w, float dt);
float normalize(float ang);
int steer_angle_to_servo_angle(int steer_angle);
float calculate_e_fa();


VectorXf mu{3};

void * stanley(void * threadid)
{
    mtx.lock();
    // NOTERA att dessa behöver laddas in från vägplaneraren (i alla fall om de har ändrats)
    points = std::vector<path_point>{glob_path_points};
    mtx.unlock();
    //std::reverse(points.begin(), points.end()); // Testa basbanan baklänges. Oops fungerar inte. VInklarna ändras ju inte!
    while (1)
    {
     //Mutex?!?!??!?!?!!?
        mtx.lock();
        bool manual_mode{glob_manual_mode};
        mtx.unlock();
        struct timespec start_time, stop_time;
        clock_gettime(CLOCK_REALTIME, &start_time);
        if (!manual_mode)
        {
            int theta_p{closest_point_to_car(points).angle};
            mtx.lock();
            float velocity{glob_velocity};
            float k = glob_stanley_k;
            int theta{glob_ang};
            mtx.unlock();
            int theta_e{};
            if (theta > 90 * 60 && theta_p < -90 * 60)
            {
                theta_e = theta - theta_p - 360 * 60;
            }
            else if (theta < -90 * 60 && theta_p > 90 * 60)
            {
                theta_e = theta + 360 * 60 - theta_p;
            }
            else
                theta_e = theta - theta_p;
            e_fa = calculate_e_fa();
            int steering_angle{delta(theta_e, velocity)};
            mu = estimate_new_pose(velocity, steering_angle, 10, lr, lf); // 10 ms, kanske ska ändras
            mtx.lock();
            if (!glob_manual_mode)
            {
                glob_steer_angle = steer_angle_to_servo_angle(steering_angle); 
                glob_reference_speed = 500;     // Hårdkodat
                glob_x = mu(0);
                glob_y = mu(1);
                glob_ang = mu(2);
                glob_e_fa = e_fa;
            }
            mtx.unlock();
        }
        mtx.lock();
        bool exit{glob_exit};
        mtx.unlock();
        if (exit)
            break;
        clock_gettime(CLOCK_REALTIME, &stop_time);
        int dt{static_cast<int>((stop_time.tv_sec - start_time.tv_sec) * 1000 + (stop_time.tv_nsec - start_time.tv_nsec) / 1000000)};
        std::this_thread::sleep_for(std::chrono::milliseconds{10 - dt});
    }
}

int steer_angle_to_servo_angle(int steer_angle)
{
    return (steer_angle * 30) / 18;
}

int delta(int theta_e, float velocity)
{
    int delta{static_cast<int>(-theta_e + 180 * 60 / 3.141592 * atan(k * e_fa / velocity))}; //Styrvinkel (lär behöva justeras med diverse termer)
    if (delta > 18 * 60)
        return 18 * 60;
    else if (delta < -18 * 60)
        return -18 * 60;
    else
    return delta;
}

//Kopierad från Pose.cc
VectorXf estimate_new_pose(float vel, float steer_angle, float dt, float lr, float lf)
{
    float w{vel / lr * static_cast<float>
        (sin(atan(tan(steer_angle * 3.1415 / 180.0 / 60.0)* lr / (lf + lr))))};
    VectorXf new_pose{3};
    new_pose(0) = mu(0) + vel * sinc_ish(mu(2) * 3.1415 / 180 / 60, w, dt / 1000.0);
    new_pose(1) = mu(1) + vel * cosc_ish(mu(2) * 3.1415 / 180 / 60, w, dt / 1000.0);
    new_pose(2) = mu(2) + w * 60 *180 / 3.14159265359 * dt / 1000.0;
    new_pose(2) = normalize(new_pose(2));

    return new_pose;
}

path_point closest_point_to_car(std::vector<path_point> points)
{
    last_point_distance = hypot(points.at(last_point).x_pos - mu(0), points.at(last_point).y_pos - mu(1));
    int number_of_points{points.size()};
    int look_ahead{50};
    path_point closest_point{points.at(last_point)};
    int temp_last_point{last_point};
    if (number_of_points > temp_last_point + look_ahead && temp_last_point > look_ahead)
    {
        for (int i{temp_last_point - look_ahead} ; i < temp_last_point + look_ahead ; i++)
        {
            float current_point_distance = hypot(points.at(i).x_pos - mu(0), points.at(i).y_pos - mu(1));
            if (current_point_distance < last_point_distance)
            {
                last_point = i;
                last_point_distance = current_point_distance;
            }
        }
    }
    else if (temp_last_point < look_ahead)
    {
        for (int i{number_of_points - look_ahead + temp_last_point} ; i < number_of_points ; i++)
        {
            float current_point_distance = hypot(points.at(i).x_pos - mu(0), points.at(i).y_pos - mu(1));
            if (current_point_distance < last_point_distance)
            {
                last_point = i;
                last_point_distance = current_point_distance;
            }
        }
        for (int i{0} ; i < temp_last_point + look_ahead ; i++)
        {
            float current_point_distance = hypot(points.at(i).x_pos - mu(0), points.at(i).y_pos - mu(1));
            if (current_point_distance < last_point_distance)
            {
                last_point = i;
                last_point_distance = current_point_distance;
            }
        }
    }
    else
    {
        int first{temp_last_point - look_ahead};
        if (first < 0)
            first += number_of_points;
        for (int i{temp_last_point - look_ahead} ; i < number_of_points ; i++)
        {
            float current_point_distance = hypot(points.at(i).x_pos - mu(0), points.at(i).y_pos - mu(1));
            if (current_point_distance < last_point_distance)
            {
                last_point = i;
                last_point_distance = current_point_distance;
            }
        }
        for (int i{0} ; (i < look_ahead - (number_of_points - temp_last_point))
            && (i < number_of_points - (number_of_points - temp_last_point)) ; i++)
        {
            float current_point_distance = hypot(points.at(i).x_pos - mu(0), points.at(i).y_pos - mu(1));
            if (current_point_distance < last_point_distance)
            {
                last_point = i;
                last_point_distance = current_point_distance;
            }
        }
    }
    return points.at(last_point);
}


float calculate_e_fa()
{
    path_point closest_point{points.at(last_point)};
    float x{mu(0)};
    float y{mu(1)};
    float x_1{closest_point.x_pos + 10 * cos(closest_point.angle * 3.141592 / 180.0 / 60.0)};
    float x_2{closest_point.x_pos};
    float y_1{closest_point.y_pos + 10 * sin(closest_point.angle * 3.141592 / 180.0 / 60.0)};
    float y_2{closest_point.y_pos};
    float d{(x - x_1) * (y_2 - y_1) - (y - y_1) * (x_2 - x_1)};
    if(d < 0)
        return last_point_distance;
    else
        return -last_point_distance;
}
