#include "../include/transform2D.h"
#include <cmath>
#include <iostream>


Transform2D::Transform2D()
{
  for(size_t i = 0; i < 3; i++)
    {
        for(size_t j = 0; j < 3; j++)
        {
            mat[i][j] = 0.0;
        }
    }
}

Transform2D::Transform2D(double x, double y, double theta)
{
   mat[0][0] = std::cos(theta*(M_PI/180));
   mat[0][1] = -(std::sin(theta*(M_PI/180)));
   mat[0][2] = x;
   mat[1][0] = -(mat[0][1]);
   mat[1][1] = mat[0][0];
   mat[1][2] = y;
   mat[2][0] = 0;
   mat[2][1] = 0;
   mat[2][2] = 1;
}

std::array<double, 2> Transform2D::apply(double x, double y)const
{   
    //point[0] = mat[0][0] * req_point[0] + mat[0][1] * req_point[1] + mat[0][2] * req_point[2];
    //point[1] = mat[1][0] * req_point[0] + mat[1][1] * req_point[1] + mat[1][2] * req_point[2];

    std::array<double, 2> point{0.0};
    double req_point[3] = {x, y, 1};

    for(size_t i = 0; i < 2; i++)
    {
        for(size_t j = 0; j < 3; j++)
        {
            point[i] += mat[i][j] * req_point[j];
        }
    }
    return point;

}

Transform2D Transform2D::compose(const Transform2D& other)const
{
    Transform2D transformed;
    for(size_t i = 0; i < 3; ++i)
    {
        for(size_t j = 0; j < 3; ++j)
        {
            for(size_t k = 0; k < 3; ++k)
            {
            transformed.mat[i][j] += this->mat[i][k] * other.mat[k][j];
            }
        }
    }
    return transformed;
}

Transform2D Transform2D::inverse() const
{
    Transform2D inverse_TF;
    double R_Transpose[2][2];
    for(size_t i = 0; i < 2; i++)
    {
        for(size_t j = 0; j < 2; j++)
        {
            R_Transpose[j][i] = mat[i][j];
            inverse_TF.mat[j][i] = R_Transpose[j][i];
        }
    }

    //inverse_TF.mat[0][2] = -( R_Transpose[0][0] * mat[0][2] + R_Transpose[0][1] * mat[1][2])
    //inverse_TF.mat[1][2] = -( R_Transpose[1][0] * mat[0][2] + R_Transpose[1][1] * mat[1][2])
    for(size_t i = 0; i < 2; i++)
    {
        size_t k = 2;
        for(size_t j = 0; j < 2; j++)
        {
            inverse_TF.mat[i][k] -= (R_Transpose[i][j] * mat[j][k]);
        }
    }

    
    for(size_t i = 2; i < 3; i++)
    {
        for(size_t j = 0; j < 3; j++)
        {
            if(i == j) inverse_TF.mat[i][j] = 1;
            else inverse_TF.mat[i][j] = 0;
        }
    }
    
    return inverse_TF;
}

void Transform2D::display() const
{
    for(size_t i = 0; i < 3; i++)
    {
        for(size_t j = 0; j < 3; j++)
        {
            std::cout << mat[i][j] << " " ;
        }
        std::cout << std::endl;
    }
}

bool Transform2D::isIdentity(double epsilon) const
{
    for(size_t i = 0; i < 3; i++)
    {
        for(size_t j = 0; j < 3; j++)
        {
            double expected = (i == j) ? 1.0 : 0.0;
            if(std::abs(mat[i][j] - expected) > epsilon)
                return false;
        }
    }
    return true;
}