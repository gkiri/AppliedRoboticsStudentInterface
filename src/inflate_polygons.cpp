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

std::vector<Polygon> inflate_walls(double map_w, double map_h, double OFFSET){
    //variables    
    Polygon left_wall, right_wall, upper_wall, bottom_wall;
    std::vector<Polygon> inflated_walls;

    //left wall
    left_wall.emplace_back(0,0);
    left_wall.emplace_back(OFFSET,0);
    left_wall.emplace_back(OFFSET,map_h);
    left_wall.emplace_back(0,map_h);

    //bottom wall
    bottom_wall.emplace_back(0,0);
    bottom_wall.emplace_back(map_w,0);
    bottom_wall.emplace_back(map_w,OFFSET);
    bottom_wall.emplace_back(0,OFFSET);

    //right wall
    right_wall.emplace_back(map_w - OFFSET,0);
    right_wall.emplace_back(map_w,0);
    right_wall.emplace_back(map_w,map_h);
    right_wall.emplace_back(map_w - OFFSET,map_h);

    //Upper wall
    upper_wall.emplace_back(0,map_h - OFFSET);
    upper_wall.emplace_back(map_w,map_h - OFFSET);
    upper_wall.emplace_back(map_w,map_h);
    upper_wall.emplace_back(0,map_h);

    //Push all into vector
    inflated_walls.push_back(left_wall);
    inflated_walls.push_back(right_wall);
    inflated_walls.push_back(upper_wall);
    inflated_walls.push_back(bottom_wall);

    return inflated_walls;
}

