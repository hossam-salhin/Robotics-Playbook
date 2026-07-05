#pragma once 
#include <array>
#include <iostream>

class Transform3D
{
    public:
    Transform3D();
    Transform3D(double x, double y, double z,
                    double roll, double pitch, double yaw);

    std::array<double, 3> apply(double x, double y, double z)const;

    Transform3D compose(const Transform3D& other)const;

    Transform3D inverse() const;

    void display() const;

    bool isIdentity(double epsilon = 1e-4)const;


    private:
    double mat[4][4];

    void multiplyMat(double a[3][3], double b[3][3], double result[3][3])
    {
        double temp[3][3] = {0};
        for(size_t i = 0; i < 3; i++)
        {
            for(size_t j = 0; j < 3; j++)
            {
                for (size_t k = 0; k < 3; k++)
                {
                    temp[i][j] += a[i][k] * b[k][j]; 
                }
            }
        }
        for(size_t i = 0; i < 3; i++)
        {
            for(size_t j = 0; j < 3; j++)
            {
                result[i][j] = temp[i][j];  
            }
        }
    }

};