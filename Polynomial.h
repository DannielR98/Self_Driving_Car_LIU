
#pragma once

class Polynomial
{
public:
    Polynomial(int _a3, int _a2, int _a1, int _a0);
    int evaluate(float t_value);
    int evaluate_derivative(float t_value);
    int evaluate_second_derivative(float t_value);
private:
    int a3;
    int a2;
    int a1;
    int a0;
};
