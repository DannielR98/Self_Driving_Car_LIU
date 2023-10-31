
#pragma once

struct point
{
    int x_pos;
    int y_pos;
    int original_angle;  // Vinkel som punkten ska passeras med i bågminuter (grader / 60)
                        // Vinkeln 0 är rakt åt höger längs x-Axeln och ökar moturs
    int new_angle;
    int k;      // k
};

struct path_point
{
    int x_pos;
    int y_pos;
    int angle; //Denna la Axel till
    float dx_dt;
    float d2x_dt2;
    float dy_dt;
    float d2y_dt2;
    float curvature;
};
