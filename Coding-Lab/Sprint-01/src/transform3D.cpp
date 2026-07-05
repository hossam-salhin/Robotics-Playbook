#include "../include/transform3D.h"
#include <cmath>



Transform3D::Transform3D()
{
    for(size_t i = 0; i < 4; i++)
    {
        for(size_t j = 0; j < 4; j++)
        {
            mat[i][j] = 0.0;
        }
    }
}
Transform3D::Transform3D(double x, double y, double z,
                    double roll, double pitch, double yaw)
{
    double Rx[3][3];
    Rx[0][0] = 1;
    Rx[0][1] = 0;
    Rx[0][2] = 0;
    Rx[1][0] = 0;
    Rx[1][1] = std::cos(roll*(M_PI/180));
    Rx[1][2] = -(std::sin(roll*(M_PI/180)));
    Rx[2][0] = 0;
    Rx[2][1] = -(Rx[1][2]);
    Rx[2][2] = Rx[1][1];

    double Ry[3][3];
    Ry[0][0] = std::cos(pitch*(M_PI/180));
    Ry[0][1] = 0;
    Ry[0][2] = std::sin(pitch*(M_PI/180));
    Ry[1][0] = 0;
    Ry[1][1] = 1;
    Ry[1][2] = 0;
    Ry[2][0] = -(Ry[0][2]);
    Ry[2][1] = 0;
    Ry[2][2] = Ry[0][0];

    double Rz[3][3];
    Rz[0][0] = std::cos(yaw*(M_PI/180));
    Rz[0][1] = -(std::sin(yaw*(M_PI/180)));
    Rz[0][2] = 0;
    Rz[1][0] = -(Rz[0][1]);
    Rz[1][1] = Rz[0][0];
    Rz[1][2] = 0;
    Rz[2][0] = 0;
    Rz[2][1] = 0;
    Rz[2][2] = 1;

    double R[3][3] = {0};
    multiplyMat(Rz, Ry, R);
    multiplyMat(R, Rx, R);

    for(size_t i = 0; i < 3; i++)
    {
        for(size_t j = 0; j < 3; j++)
        {
            mat[i][j] = R[i][j];
        }
    }
    
    mat[0][3] = x;
    mat[1][3] = y;
    mat[2][3] = z;
    mat[3][0] = 0;
    mat[3][1] = 0;
    mat[3][2] = 0;
    mat[3][3] = 1; 
}

std::array<double, 3> Transform3D::apply(double x, double y, double z)const
{
    std::array<double, 3> point{0.0};
    double req_point[4] = {x, y, z, 1};

    for(size_t i = 0; i < 3; i++)
    {
        for(size_t j = 0; j < 4; j++)
        {
            point[i] += mat[i][j] * req_point[j];
        }
    }
    return point;
}

Transform3D Transform3D::compose(const Transform3D& other)const
{
    Transform3D transformed;
    for(size_t i = 0; i < 4; ++i)
    {
        for(size_t j = 0; j < 4; ++j)
        {
            for(size_t k = 0; k < 4; ++k)
            {
            transformed.mat[i][j] += this->mat[i][k] * other.mat[k][j];
            }
        }
    }
    return transformed;
}

Transform3D Transform3D::inverse() const
{
    Transform3D inverse_TF;
    double R_Transpose[3][3];
    for(size_t i = 0; i < 3; i++)
    {
        for(size_t j = 0; j < 3; j++)
        {
            R_Transpose[j][i] = mat[i][j];
            inverse_TF.mat[j][i] = R_Transpose[j][i];
        }
    }

    for(size_t i = 0; i < 3; i++)
    {
        size_t k = 3;
        for(size_t j = 0; j < 3; j++)
        {
            inverse_TF.mat[i][k] -= (R_Transpose[i][j] * mat[j][k]); 
        }
    }

    inverse_TF.mat[3][3] = 1;

    return inverse_TF;
}

void Transform3D::display() const
{
    for(size_t i = 0; i < 4; i++)
    {
        for(size_t j = 0; j < 4; j++)
        {
            std::cout << mat[i][j] << " " ;
        }
        std::cout << std::endl;
    }
}

bool Transform3D::isIdentity(double epsilon)const
{
     for(size_t i = 0; i < 4; i++)
    {
        for(size_t j = 0; j < 4; j++)
        {
            double expected = (i == j) ? 1.0 : 0.0;
            if(std::abs(mat[i][j] - expected) > epsilon)
                return false;
        }
    }
    return true;
}