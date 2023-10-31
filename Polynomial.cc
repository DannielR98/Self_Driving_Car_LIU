
#include "Polynomial.h"

Polynomial::Polynomial(int _a3, int _a2, int _a1, int _a0)
: a3{_a3}, a2{_a2}, a1{_a1}, a0{_a0}
{

}

int Polynomial::evaluate(float t_value)
{
    return t_value * t_value * t_value * a3 + t_value * t_value * a2 + t_value * a1 + a0;
}

int Polynomial::evaluate_derivative(float t_value)
{
    return 3 * t_value * t_value * a3 + 2 * t_value * a2 + a1;
}

int Polynomial::evaluate_second_derivative(float t_value)
{
    return 6 * t_value * a3 + 2 * a2;
}
