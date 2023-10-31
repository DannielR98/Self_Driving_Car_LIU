#include "gottochblandat.h"
#include <cmath>

using namespace::std;

// maclaurinutveckling av (sin(th + w * dt) - sin(th)) / w
float sinc_ish(float th, float w, float dt)
{
    return static_cast<float>(cos(th) * dt - sin(th) * w * dt * dt / 2.0
        - cos(th) * w * w * dt * dt * dt / 6.0);

}

// maclaurinutveckling av (cos(th) - cos(th + w * dt)) / w
float cosc_ish(float th, float w, float dt)
{

    return static_cast<float>(sin(th) * dt + cos(th) * w * dt * dt / 2.0
        - sin(th) * w * w * dt * dt * dt / 6.0);
}

// Normerar vinklar i bågminuter mellan -180*60 och 180*60
float normalize(float ang)
{
    while(ang < -180 * 60)
        ang += 360 *60;
    while(ang > 180 * 60)
        ang -= 360 *60;
    return ang;
}

// Normerar vinklar i radianer mellan -pi och pi
float normalize_r(float ang)
{
    while(ang < -3.14159265359)
        ang += 2 * 3.14159265359;
    while(ang > 3.14159265359)
        ang -= 2 * 3.14159265359;
    return ang;
}
