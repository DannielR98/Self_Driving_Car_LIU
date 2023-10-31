#include "Landmark.h"

using namespace std;

Landmark::Landmark(float _x_pos, float _y_pos, float _radius)
    :xpos{_x_pos}, ypos{_y_pos}, radius{_radius}{}

void Landmark::set_position(float new_x_pos, float new_y_pos)
{
    xpos = new_x_pos;
    ypos = new_y_pos;
}
