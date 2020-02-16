#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <vector>
#include "clipper.hpp"

#define OFFSETTING 0    //(0)Deactivation - (1)Activation of printouts.


std::vector<Polygon> inflate_polygons(const std::vector<Polygon>& obstacle_list, double OFFSET){
    //Constant variables
    const double INT_ROUND = 1000.;    

    //Clipper variables
    ClipperLib::Path srcPoly;
    ClipperLib::Paths newPoly;

    //Other variables
    Polygon obstacle;  
    std::vector<Polygon> inflated_obstacle_list;  

    //Initialize variables
    int obl_size = obstacle_list.size();   
    inflated_obstacle_list.resize(obl_size);
    int scaled_OFFSET = ceil(OFFSET*INT_ROUND);
    #if OFFSETTING
    std::cout << "Obstacle_list_size:  " << obl_size << std::endl;
    std::cout << "inflated_obstacle_list_size:  " << inflated_obstacle_list.size() << std::endl;
    std::cout << "OFFSET:" << scaled_OFFSET << std::endl;
    #endif   

    //Loop over each polygon
    for (size_t i = 0; i<obl_size; i++){    
        obstacle = obstacle_list[i];
        #if OFFSETTING
        std::cout << "Polygon:" << i << std::endl;
        #endif
        srcPoly.clear(); //clear for each new polygon
        //Loop over each vertex
        for (size_t j = 0; j<obstacle.size(); j++){        
            int x = obstacle[j].x*INT_ROUND;
            int y = obstacle[j].y*INT_ROUND;
            #if OFFSETTING
            std::cout << "x:" << x << "  y:" << y << std::endl;
            #endif
            srcPoly << ClipperLib::IntPoint(x, y);
        }
        //Polygon offsetting
        ClipperLib::ClipperOffset co;    
        co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(newPoly, scaled_OFFSET); 

        //Save inflated polygon
        for(const ClipperLib::Path &path: newPoly){                
            for (const ClipperLib::IntPoint &pt: path){
                float x = pt.X / INT_ROUND;
                float y = pt.Y / INT_ROUND;
                #if OFFSETTING
                std::cout << "new_x:" << x << "  new_y:" << y << std::endl;
                #endif
                //add vertex (x,y) to current obstacle               
                inflated_obstacle_list[i].emplace_back(x, y);
            }
            #if OFFSETTING
            //Print inflated polygon
            std::cout << "Inflated_Polygon:" << i << std::endl;    
            for (size_t j = 0; j<inflated_obstacle_list[i].size(); j++){        
                float x = inflated_obstacle_list[i][j].x;
                float y = inflated_obstacle_list[i][j].y;
                std::cout << "inflated_x:" << x << "  inflated_y:" << y << std::endl;        
            }
            #endif
        }          
    }

    return inflated_obstacle_list;
}


std::vector<Polygon> inflate_walls(double map_w, double map_h, double OFFSET, 
        Polygon gate, Polygon square_gate){
    
    Polygon wall;
    std::vector<Polygon> inflated_walls, final_walls;
    double diff_x, diff_y, max_diff = 0;
   

    //left wall
    wall.emplace_back(0,0);
    wall.emplace_back(OFFSET,0);
    wall.emplace_back(OFFSET,map_h);
    wall.emplace_back(0,map_h);
    inflated_walls.push_back(wall);
    wall.clear();

    //Upper wall
    wall.emplace_back(0,map_h - OFFSET);
    wall.emplace_back(map_w,map_h - OFFSET);
    wall.emplace_back(map_w,map_h);
    wall.emplace_back(0,map_h);
    inflated_walls.push_back(wall);
    wall.clear();

    //bottom wall
    wall.emplace_back(0,0);
    wall.emplace_back(map_w,0);
    wall.emplace_back(map_w,OFFSET);
    wall.emplace_back(0,OFFSET);
    inflated_walls.push_back(wall);
    wall.clear();

    //right wall
    wall.emplace_back(map_w - OFFSET,0);
    wall.emplace_back(map_w,0);
    wall.emplace_back(map_w,map_h);
    wall.emplace_back(map_w - OFFSET,map_h);
    inflated_walls.push_back(wall);
    wall.clear();    

    //Constant variables
    const double INT_ROUND = 1000.;  

    //Clipper variables
    ClipperLib::Path gatePoly;
    ClipperLib::Path wallPoly;
    ClipperLib::Paths newPoly;
    
    int wall_size = inflated_walls.size();
    final_walls.resize(wall_size);    

    //save square gate    
    for (size_t j = 0; j<square_gate.size(); j++){        
        int x = square_gate[j].x*INT_ROUND;
        int y = square_gate[j].y*INT_ROUND;       
        //std::cout << "gate: x:" << x << "  y:" << y << std::endl;     
        gatePoly << ClipperLib::IntPoint(x, y);
    }  

    for(int i=0; i<inflated_walls.size();i++){
        wallPoly.clear();
        wall = inflated_walls[i];
        for (size_t j = 0; j<wall.size(); j++){        
            int x = wall[j].x*INT_ROUND;
            int y = wall[j].y*INT_ROUND;       
            //std::cout << "wall: x:" << x << "  y:" << y << std::endl;      
            wallPoly << ClipperLib::IntPoint(x, y);
        }
        
        ClipperLib::Clipper clpr;   
        clpr.AddPath(wallPoly, ClipperLib::ptSubject, true);
        clpr.AddPath(gatePoly, ClipperLib::ptClip, true);
        //ClipperLib::Paths solution;
        clpr.Execute(ClipperLib::ctDifference, newPoly, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

        //Save inflated polygon
        for(const ClipperLib::Path &path: newPoly){                
            for (const ClipperLib::IntPoint &pt: path){
                float x = pt.X / INT_ROUND;
                float y = pt.Y / INT_ROUND;
                //std::cout << "new_x:" << x << "  new_y:" << y << std::endl;
                //add vertex (x,y) to current obstacle                       
                final_walls[i].emplace_back(x, y);            
            }
        }        
    }

    return final_walls;
}



Polygon get_gate_pose(const Polygon& gate, double map_h, double map_w, double robot_length, 
    double* gate_pose, double goal_delta){  

    double RAD2DEG = 180.0/M_PI;
    double x_sum = 0;
    double y_sum = 0;
    int n_vertex = 0;
    double gate_w, diff_x, diff_y, w_d2, w_d4, w_3d4;
    //double delta = 0.025;
    std::vector<double> dist_to_wall; //[left,right,up,down]    
    double lowest_dist = 100;
    int closest_wall;
    Polygon square_gate;

    // Center point of rectangle
    for(Point vertex : gate){
        //std::cout << vertex.x <<","<<vertex.y<<std::endl;
        x_sum += vertex.x;
        y_sum += vertex.y;
        n_vertex++;
    }
    gate_pose[0] = x_sum/n_vertex; //xc
    gate_pose[1] = y_sum/n_vertex; //yc

    //gate center point
    Point gate_center = Point(gate_pose[0], gate_pose[1]);

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

    //gate width (biggest side)
    for(int i=0;i<gate.size()-1;i++){
        diff_x = fabs(gate[i].x - gate[i+1].x);
        diff_y = fabs(gate[i].y - gate[i+1].y);
        if(diff_x > diff_y){
            if(diff_x > gate_w){
                gate_w = diff_x;
            }
        }
        else if(diff_y > gate_w){
            gate_w = diff_y;
        }           
    }
    //std::cout << "GATE WIDTH: " << gate_w << std::endl;

    //square gate constants
    Point gate_tl_corner;
    w_d2 = gate_w/2;
    w_d4 = gate_w/4;
    w_3d4 = 3*gate_w/4;
   

    //Set pose and heading angle depending on closest wall
    switch (closest_wall){
    case 0: //left
        gate_pose[0] = robot_length/2 + goal_delta;
        gate_pose[2] = M_PI;
        //std::cout << "closes wall: Left wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;

        //get top left corner of square gate
        gate_tl_corner = Point(gate_center.x-w_d4, gate_center.y+w_d2);

        break;
    case 1: //right
        gate_pose[0] = map_w - (robot_length/2 + goal_delta);
        gate_pose[2] = 0;
        //std::cout << "closes wall: Right wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;

        //get top left corner of square gate
        gate_tl_corner = Point(gate_center.x-w_3d4, gate_center.y+w_d4);

        break;
    case 2: //up
        gate_pose[1] = map_h - (robot_length/2 + goal_delta);
        gate_pose[2] = M_PI/2;
        //std::cout << "closes wall: Up wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
       
        //get top left corner of square gate
        gate_tl_corner = Point(gate_center.x-w_d2, gate_center.y+w_d4);

        break;
    case 3: //down
        gate_pose[1] = robot_length/2 + goal_delta;
        gate_pose[2] = -M_PI/2;
        //std::cout << "closes wall: Down wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        
        //get top left corner of square gate
        gate_tl_corner = Point(gate_center.x-w_d2, gate_center.y+w_3d4);

        break;
    
    default:
        printf("No closest wall found\n");
        break;
    } 

    //Construct square gate    
    square_gate.emplace_back(gate_tl_corner);
    square_gate.emplace_back(gate_tl_corner.x + gate_w, gate_tl_corner.y);
    square_gate.emplace_back(gate_tl_corner.x + gate_w, gate_tl_corner.y - gate_w);
    square_gate.emplace_back(gate_tl_corner.x, gate_tl_corner.y - gate_w);
   

    return square_gate;
}

