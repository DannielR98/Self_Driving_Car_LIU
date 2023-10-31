#pragma once

class Landmark
{
    public:
    Landmark(float _x_pos, float _y_pos, float _radius);

    void set_position(float new_x_pos, float new_y_pos);

    //private:
        float xpos{};
        float ypos{};
        float radius{};
};
