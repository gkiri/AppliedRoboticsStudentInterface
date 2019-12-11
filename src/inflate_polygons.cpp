#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <vector>

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

// //////PRINTING OUT POLYGONS///////////
// void print_polygons_out(std::vector<Polygon> obstacle_list, 
//                         std::vector<Polygon> inflated_obstacle_list){
//     //variables
//     Polygon ob, inflated_ob; 
//     int ob_size, inflated_ob_size;
//     int obl_size = obstacle_list.size(); //inflated obstacle size is the same    

//     const double TO_CM = 100;
//     int map_w = 150; //map width in cms
//     int map_h = 100; //map height in cms

//     using namespace cv;
//     std::vector<std::vector<cv::Point>> polys, inf_polys;
//     std::vector<cv::Point> v_img_ob, v_img_inflated_ob;
//     int img_map_w = 720; //map width in pixels
//     int img_map_h = 576; //map height in pixels
//     Mat image = Mat::zeros(img_map_h, img_map_w, CV_8UC3);

//     //Loop over each polygon
//     for (size_t i = 0; i<obl_size; i++){
//         #if PRINTOUT_ALL
//         #else
//         //Reset variables
//         image.setTo(Scalar::all(0));
//         polys.clear();
//         inf_polys.clear();
//         #endif 
//         v_img_ob.clear();
//         v_img_inflated_ob.clear();      
//         //Retrieve both polygons and the number of vertexes
//         ob = obstacle_list[i];
//         inflated_ob = inflated_obstacle_list[i];
//         ob_size = ob.size();
//         inflated_ob_size = inflated_ob.size();     
//         std::cout << "Polygon:" << i << "  size obstacle:" << ob_size << 
//                             "  size inflated obstacle:" << inflated_ob_size << std::endl;    
//         //Loop over each obstacle vertex
//         for (size_t j = 0; j<ob_size; j++){        
//             int x_ob = ob[j].x*TO_CM;
//             int y_ob = ob[j].y*TO_CM;                            
//             v_img_ob.emplace_back(img_map_w*x_ob/map_w, img_map_h*y_ob/map_h);                   
//         }
//         //Loop over each inflated obstacle vertex
//         for (size_t j = 0; j<inflated_ob_size; j++){         
//             int x_inf_ob = inflated_ob[j].x*TO_CM;
//             int y_inf_ob = inflated_ob[j].y*TO_CM;                  
//             v_img_inflated_ob.emplace_back(img_map_w*x_inf_ob/map_w, img_map_h*y_inf_ob/map_h);                
//         }       
        
//         #if PRINTOUT_ALL
//         //Save all polygon 
//         inf_polys.push_back(v_img_inflated_ob);
//         polys.push_back(v_img_ob);

//         #else
//         ////Draw polygons one by one
//         inf_polys = {v_img_inflated_ob};      
//         fillPoly(image, inf_polys, Scalar(255,0,0));
//         polys = {v_img_ob}; 
//         fillPoly(image, polys, Scalar(255,255,255));
//         imshow("Image",image);
//         waitKey( 0 );
//         #endif      
//     }
//     #if PRINTOUT_ALL
//     //Draw all polygons
//     fillPoly(image, inf_polys, Scalar(255,0,0));
//     fillPoly(image, polys, Scalar(255,255,255));
//     imshow("Image",image);
//     waitKey( 0 );
//     #endif
// }


