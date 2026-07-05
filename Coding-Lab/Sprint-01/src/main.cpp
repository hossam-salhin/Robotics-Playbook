#include "../include/transform2D.h" 
#include "../include/transform3D.h" 
#include <iostream>


int main()
{
    Transform2D T(5.0, 2.0, 90.0);
    Transform2D T_inv = T.inverse();
    Transform2D identity = T.compose(T_inv);
    
    std::cout << "T:" << std::endl;
    T.display();
    
    std::cout << "\nT_inverse:" << std::endl;
    T_inv.display();
    
    std::cout << "\nT * T_inverse (should be identity):" << std::endl;
    identity.display();

    if(identity.isIdentity())
    {
        std::cout << "\nT * T_inverse is valid identity" << std::endl;
    }
    else{std::cout << "\nT * T_inverse is not identity" << std::endl;}

    if(T.isIdentity())
    {
        std::cout << "\nT is valid identity" << std::endl;
    }
    else{std::cout << "\nT is not identity" << std::endl;}
    
    // Session 1 exercise verification
    // Robot at (5,2) facing 90 degrees
    // LiDAR offset: 0.3m forward, no rotation
    // Obstacle at (3.2, 1.7) in LiDAR frame
    Transform2D T_lidar_to_base(0.3, 0.0, 0.0);
    Transform2D T_base_to_world(5.0, 2.0, 90.0);
    Transform2D T_lidar_to_world = T_base_to_world.compose(T_lidar_to_base);
    
    auto obstacle_world = T_lidar_to_world.apply(3.2, 1.7);
    std::cout << "\nObstacle in world frame: ("
              << obstacle_world[0] << ", "
              << obstacle_world[1] << ")" << std::endl;


    std::cout << "****************************************\n";
    std::cout << "              Transfrom3D\n";
    
    Transform3D T3(5,2,0,5,3,60);
    Transform3D T3_inverse = T3.inverse();
    Transform3D identity_3d = T3.compose(T3_inverse);

    std::cout << "T3:" << std::endl;
    T3.display();

    std::cout << "\nT3_inverse:" << std::endl;
    T3_inverse.display();

    std::cout << "\nT3 * T3_inverse (should be identity):" << std::endl;
    identity_3d.display();

    if(identity_3d.isIdentity())
    {
        std::cout << "\nT3 * T3_inverse is valid identity" << std::endl;
    }
    else{std::cout << "\nT3 * T3_inverse is not identity" << std::endl;}


    Transform3D T3_base_to_world(5,2,0,5,3,60);
    Transform3D T3_camera_to_base(0.3,0,0,0,0,0);
    Transform3D T3_camera_to_world = T3_base_to_world.compose(T3_camera_to_base);

    auto landmark_world = T3_camera_to_world.apply(2.0,0.5,0.3);
    std::cout << "\nLandmark in world frame: ("
                << landmark_world[0] << ", "
                << landmark_world[1] << ","
                << landmark_world[2] << ")" << std::endl;
    return 0;
}
