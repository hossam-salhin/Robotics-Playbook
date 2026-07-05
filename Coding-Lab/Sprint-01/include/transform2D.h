#pragma once 
#include <array>

class Transform2D
{
    public:
    Transform2D();
    Transform2D(double x, double y, double theta);

    std::array<double, 2> apply(double x, double y)const;

    Transform2D compose(const Transform2D& other)const;

    Transform2D inverse() const;

    void display() const;

    bool isIdentity(double epsilon = 1e-9)const;

    private:
    double mat[3][3];

};