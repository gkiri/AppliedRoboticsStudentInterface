#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <vector>

#include "mission_apis.hpp"

void get_gate_pose(const Polygon& gate, double map_h, double map_w, double robot_length, double* gate_pose){    
    double RAD2DEG = 180.0/M_PI;
    double x_sum = 0;
    double y_sum = 0;
    int n_vertex = 0;
    double delta = 0.025;
    std::vector<double> dist_to_wall; //[left,right,up,down]    
    double lowest_dist = 100;
    int closest_wall;

    // Center point of rectangle
    for(Point vertex : gate){
        std::cout << vertex.x <<","<<vertex.y<<std::endl;
        x_sum += vertex.x;
        y_sum += vertex.y;
        n_vertex++;
    }
    gate_pose[0] = x_sum/n_vertex; //xc
    gate_pose[1] = y_sum/n_vertex; //yc

    //gate_pose
    dist_to_wall.push_back(gate_pose[0]); //left
    dist_to_wall.push_back(map_w - gate_pose[0]); //right
    dist_to_wall.push_back(map_h - gate_pose[1]); //up    
    dist_to_wall.push_back(gate_pose[1]); //down
    
    //Get closest wall
    for(int i=0;i<dist_to_wall.size();i++){
        if(dist_to_wall[i] < lowest_dist){
            lowest_dist = dist_to_wall[i];
            closest_wall = i;
        }
    }
    //std::cout << "lowest dist:" << lowest_dist << std::endl;

    //Set pose and heading angle depending on closest wall
    switch (closest_wall){
    case 0: //left
        gate_pose[0] = robot_length/2 + delta;
        gate_pose[2] = M_PI;
        //std::cout << "closes wall: Left wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    case 1: //right
        gate_pose[0] = map_w - (robot_length/2 + delta);
        gate_pose[2] = 0;
        //std::cout << "closes wall: Right wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    case 2: //up
        gate_pose[1] = map_h - (robot_length/2 + delta);
        gate_pose[2] = M_PI/2;
        //std::cout << "closes wall: Up wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    case 3: //down
        gate_pose[1] = robot_length/2 + delta;
        gate_pose[2] = -M_PI/2;
        //std::cout << "closes wall: Down wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    
    default:
        printf("No closest wall found\n");
        break;
    } 
}

Path mission_0(double start_pose[3], double gate_pose[3]){

}


Path mission_1(double start_pose[3], double gate_pose[3], 
    std::vector<Polygon> inflated_obstacle_list){

}


Path mission_1_5(double start_pose[3], double gate_pose[3], std::vector<Point> bias_points, 
    std::vector<Polygon> inflated_obstacle_list){

}


Path mission_2(double start_pose[3], double gate_pose[3], 
    std::vector<std::pair<int,Polygon>> victim_list, std::vector<Polygon> inflated_obstacle_list){

}
