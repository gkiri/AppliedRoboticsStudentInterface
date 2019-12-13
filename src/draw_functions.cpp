#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <vector>
#include <math.h>

//Struct for map visualizer initialization
struct img_map_def{
    cv::Mat img_map;
    float scale;
};
//Struct for ellipse calculations
struct arc_param{
    double rotation_angle;
    double angle_cs_ce;
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
     
    cv::Point point_scaled;
    // IMPORTANT!!! Draw point is the only drawing function who get values in CMs
    point_scaled.x = img_map_def.scale*point.x;
    point_scaled.y = img_map_def.scale*point.y;
    // point_scaled.x = img_map_def.scale*point.x*TO_CM;
    // point_scaled.y = img_map_def.scale*point.y*TO_CM;         
    
    circle(img_map_def.img_map, point_scaled, radius, cv::Scalar(0, 0, 255), 
            -1, 8, 0);
}

void draw_polygon(Polygon poly, img_map_def img_map_def){    
    std::vector<std::vector<cv::Point>> v_poly_scaled; 
    std::vector<cv::Point> poly_scaled;   

    for (size_t i = 0; i<poly.size(); i++){        
        float poly_scaled_x = poly[i].x*img_map_def.scale*TO_CM;
        float poly_scaled_y = poly[i].y*img_map_def.scale*TO_CM;                         
        poly_scaled.emplace_back(poly_scaled_x, poly_scaled_y);                   
    }
    v_poly_scaled = {poly_scaled};  //fillpoly works with vectors of polygons
    fillPoly(img_map_def.img_map, v_poly_scaled, cv::Scalar(255,0,0));
}

arc_param calculate_arc_drawing_angles(arc_extract arc){
    // Taking center as the origin of our coordinates system  
    // angle_cs_ce --> Angle btw the start-end point segment and the center-start_point segment
    //                 or ,equivalently, the center-end_point segment.
    // rotation_angle --> Rotation of ellipse w.r.t x-axis (>0 clockwise)    
    double rotation_angle, angle_cs_ce;
    double dist_center_mid; //distance btw center and mid point of star-end point segment
    double RAD2DEG = 180.0/M_PI; 
    
    // Rotation angle in absolute value, then apply sign depending on left or right curvature
    rotation_angle = abs(atan((arc.start_point.y - arc.end_point.y)
                                    /(arc.start_point.x - arc.end_point.x))*RAD2DEG);

    //ADD SIGN DEPENDING OF TURN DIRECTION

    // dist_center_mid = sqrt(pow(arc.radius,2) - (pow(arc.start_point.x - arc.end_point.x,2) +
    //                             pow(arc.start_point.y - arc.end_point.y,2))/4);    
    double xs_xe = pow(arc.start_point.x - arc.end_point.x,2);
    double ys_ye = pow(arc.start_point.y - arc.end_point.y,2);
    double sqrt_content = pow(arc.radius,2) - (xs_xe + ys_ye)/4;
    if (sqrt_content <= 0){ //avoid errors of negative numbers close to 0
        sqrt_content = 0;
    } 
    dist_center_mid = sqrt(sqrt_content);    
    //std::cout << "distance to midpoint: " << dist_center_mid << std::endl;
    
    angle_cs_ce = asin(dist_center_mid/arc.radius)*RAD2DEG;
    
    arc_param result = {rotation_angle, angle_cs_ce};

    return result;    
}

void draw_arc(arc_extract arc, img_map_def img_map_def){   
    // Draw an arc given the visual parameter of a map and the parameters of an arc
    arc_param arc_angles = calculate_arc_drawing_angles(arc);        
    double start_angle = 180 + arc_angles.angle_cs_ce;
    double end_angle = 360 - arc_angles.angle_cs_ce;

    cv::Point center_scaled;
    center_scaled.x = arc.center.x*img_map_def.scale*TO_CM;
    center_scaled.y = arc.center.y*img_map_def.scale*TO_CM;
    double radius_scaled = arc.radius*img_map_def.scale*TO_CM; 

    cv::ellipse(img_map_def.img_map, center_scaled, cv::Size(radius_scaled, radius_scaled), 
               arc_angles.rotation_angle, start_angle, end_angle, cv::Scalar(255, 255, 255),1,15,0);

    // cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(50,50), cv::Size2f(40,30), 30);
    // cv::ellipse(img_map_def.img_map, rRect, cv::Scalar(255, 255, 255),1,15);
}

line_extract to_line_extract_type(Point pt1, Point pt2, bool calc_length=false){
    //Transform a pair of Point type into a line_extract type
    // calc_length - (1) Calculate length (0) Do not, by default
    line_extract result;
    result.start_point = pt1;
    result.end_point = pt2;
    if (calc_length){
        result.length = sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2));
    }
    else{
        result.length = 0;
    }
    //test
    std::cout << "Result.start_point x before: " << result.start_point.x << std::endl;
    pt1.x = 0;
    std::cout << "Result.start_point x after: " << result.start_point.x << std::endl;

    return result;    
}

void draw_segment(line_extract segment, img_map_def img_map_def){
    cv::Point start_pt_scaled, end_pt_scaled;
    //Scale points for visualizing
    start_pt_scaled.x = segment.start_point.x*img_map_def.scale*TO_CM;
    start_pt_scaled.y = segment.start_point.y*img_map_def.scale*TO_CM;
    end_pt_scaled.x = segment.end_point.x*img_map_def.scale*TO_CM;
    end_pt_scaled.y = segment.end_point.y*img_map_def.scale*TO_CM;

    cv::line(img_map_def.img_map, start_pt_scaled, end_pt_scaled, cv::Scalar(255, 255, 255),1,16,0);
}



