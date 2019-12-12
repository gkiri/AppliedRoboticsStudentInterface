#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <vector>

//Struct for map visualizer initialization
struct img_map_def{
    cv::Mat img_map;
    float scale;
};
int TO_CM = 100;    //Transform from m to cms

img_map_def initialize_img_map(double map_w, double map_h, double img_map_w){
    // Initialize the map parameters for the visual representation of points, dubins curves
    // and polygons 
    
    float scale = img_map_w/(map_w*TO_CM); //Calculate the scale (relation btw pixels and cms)
    cv::Mat img_map = cv::Mat::zeros(scale*map_h*TO_CM, img_map_w, CV_8UC3); //create empty map

    img_map_def result = {img_map, scale};
    return result;
}

void draw_point(Point point, img_map_def img_map_def){
    //Draw a point given the visual parameter of a map and a point

    int radius = 3; //size of point

    cv::Mat img_map = img_map_def.img_map;  //NOTE  
    cv::Point point_scaled;
    point_scaled.x = img_map_def.scale*point.x;
    point_scaled.y = img_map_def.scale*point.y;    
    
    circle(img_map, point_scaled, radius, cv::Scalar(0, 0, 255), 
            -1, 8, 0);
}

void draw_polygon(Polygon poly, img_map_def img_map_def){
    cv::Mat img_map = img_map_def.img_map;
    std::vector<std::vector<cv::Point>> v_poly_scaled; 
    std::vector<cv::Point> poly_scaled;   

    for (size_t i = 0; i<poly.size(); i++){        
        float poly_scaled_x = poly[i].x*img_map_def.scale*TO_CM;
        float poly_scaled_y = poly[i].y*img_map_def.scale*TO_CM;                         
        poly_scaled.emplace_back(poly_scaled_x, poly_scaled_y);                   
    }
    v_poly_scaled = {poly_scaled};  //fillpoly works with vectors of polygons
    fillPoly(img_map, v_poly_scaled, cv::Scalar(255,0,0));
}

