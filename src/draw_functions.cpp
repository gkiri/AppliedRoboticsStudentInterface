#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <vector>
#include <math.h>
#include "Utils.hpp"

//Map object colours - BGR
cv::Scalar robot_colour(255,0,0);   //blue
cv::Scalar sample_colour(255,255,255);  //white
cv::Scalar victim_colour(0,255,0);  //green
cv::Scalar poly_colour(0,0,255);    //red
cv::Scalar path_colour(255,255,255);    //white

//Struct for map visualizer initialization
struct img_map_def{
    cv::Mat img_map;
    float scale;
    float coord_trans;
};
//Struct for ellipse calculations
struct arc_param{
    double start_angle;
    double end_angle;
};

int TO_CM = 100;    //Transform from m to cms

// /* Safe acos function -------------------------------------*/
// double SafeAcos (double x){
//   if (x < -1.0) x = -1.0 ;
//   else if (x > 1.0) x = 1.0 ;
//   return acos (x) ;
// }

/* Initialize map-------------------------------------------*/

img_map_def initialize_img_map(double map_w, double map_h, double img_map_w){
    // Initialize the map parameters for the visual representation of points, dubins curves
    // and polygons 
    
    float scale = img_map_w/(map_w*TO_CM); //Calculate the scale (relation btw pixels and cms)
    //Img(0,0)--> left top corner, Map(0,0) left down corner.
    // Coordinate transformation will be (x, map_h_scaled - y)
    float coord_trans = map_h*scale*TO_CM;
    cv::Mat img_map = cv::Mat::zeros(scale*map_h*TO_CM, img_map_w, CV_8UC3); //create empty map

    img_map_def result = {img_map, scale, coord_trans};
    return result;
}


/* Change coordinates of a point-------------------------------------------*/

cv::Point coord_map_to_img(cv::Point point_in_map_coord, img_map_def img_map_def){
    //Return point in image coordinates from a point in map coordinates
    cv::Point result;
    result.x = point_in_map_coord.x;
    result.y = img_map_def.coord_trans - point_in_map_coord.y;
    
    return result;
}


/* Draw a point-------------------------------------------*/

void draw_point(Point point, img_map_def img_map_def,
                        cv::Scalar colour = sample_colour){
    //Draw a point given the visual parameter of a map and a point    

    int radius = 3; //size of point
     
    cv::Point point_scaled;
    // IMPORTANT!!! Draw point is the only drawing function who get values in CMs
    // point_scaled.x = img_map_def.scale*point.x;
    // point_scaled.y = img_map_def.scale*point.y;
    point_scaled.x = img_map_def.scale*point.x*TO_CM;
    point_scaled.y = img_map_def.scale*point.y*TO_CM;

    //Change of coordinate system (Img(0,0)--> left top corner, Map(0,0) left down corner)
    point_scaled = coord_map_to_img(point_scaled, img_map_def);

    circle(img_map_def.img_map, point_scaled, radius, colour, -1, 8, 0);
}


/* Draw a polygon-------------------------------------------*/

void draw_polygon(Polygon poly, img_map_def img_map_def, cv::Scalar colour = poly_colour){    
    std::vector<std::vector<cv::Point>> v_poly_scaled; 
    std::vector<cv::Point> poly_scaled;   

    for (size_t i = 0; i<poly.size(); i++){        
        float poly_scaled_x = poly[i].x*img_map_def.scale*TO_CM;
        // Apply change of coordinates directly
        float poly_scaled_y = img_map_def.coord_trans - poly[i].y*img_map_def.scale*TO_CM;                     
        poly_scaled.emplace_back(poly_scaled_x, poly_scaled_y);                   
    }
    v_poly_scaled = {poly_scaled};  //fillpoly works with vectors of polygons
    fillPoly(img_map_def.img_map, v_poly_scaled, colour);
}


/* Calculate arc parameters for cv::ellipse function ---------------------------------*/

arc_param calculate_arc_drawing_angles(arc_extract arc){   
    
    double RAD2DEG = 180.0/M_PI;
    double start_angle, end_angle;
    double alpha, beta;
    Point s,e; //start and end point copies
    double d_x; //diff btw start point and center in x coordinates
    int quadrant;    

    //space
    //std::cout << "" << std::endl;
    
    // Calculate alpha = angle btw start and end point
    alpha = (arc.length/arc.radius)*RAD2DEG; 

    //std::cout << "ALPHA:"<< alpha << std::endl;

    //Copy of start and end point
    if(arc.start_point.x < arc.end_point.x){        
        //End point becomes the start point due to how cv::ellipse works
        s = arc.end_point;
        e = arc.start_point;
    }
    else{
        s = arc.start_point;
        e = arc.end_point; 
    }
    d_x = abs((s.x - arc.center.x)*10000); //abs return int
    d_x = d_x/10000; //d_x round to the 5th decimal (to increase smoothness of representation)
    
    
    // std::cout << "d_x"<< d_x << std::endl;
    // std::cout << "d_x/arc.radius"<< d_x/arc.radius << std::endl;
    // std::cout << "Start,end POINT: ("<< s.x << "," << s.y << "), " << "("<< e.x << "," << e.y << ")" << std::endl;  

    //Calculate beta = Angle btw positive x-axis and start    
    beta = SafeAcos(d_x/arc.radius)*RAD2DEG; //angle wrt to x-axis
    quadrant = compute_quadrant(s,arc.center);

    // std::cout << "BETA_pre:"<< beta << std::endl;    
    // std::cout << "Quadrant:"<< quadrant << std::endl;

    switch (quadrant){
    case 1:
        beta = -beta;
        break;
    case 2:
        beta = 180 + beta;
        break;
    case 3:
        beta = 180 - beta;
        break;
    case 4:
        // beta = beta;
        break;

    default:
        printf("ERROR in calculate_arc");
        break;
    }
    //std::cout << "BETA:"<< beta << std::endl;

    //Check for sign of turn (Left or Right) with the original start and end point
    if((arc.start_point.x > arc.end_point.x && arc.LSR == 2) || 
            ((arc.start_point.x < arc.end_point.x && arc.LSR == 0))){
        // Start point on the right wrt to end point and turn right OR
        // Start point on the left wrt to end point and turn left
        start_angle = beta;
        end_angle = start_angle + alpha;
    }
    else if((arc.start_point.x > arc.end_point.x && arc.LSR == 0) || 
            ((arc.start_point.x < arc.end_point.x && arc.LSR == 2))){
        // Start point on the right wrt to end point and turn left OR
        // Start point on the left wrt to end point and turn right
        start_angle = 360 + beta;
        end_angle = start_angle - alpha;
    }

    //std::cout << "START,END ANGLE:" << start_angle << "," << end_angle << std::endl;
    
    arc_param result = {start_angle, end_angle};

    return result; 
}    


/* Draw an arc-------------------------------------------*/

void draw_arc(arc_extract arc, img_map_def img_map_def, cv::Scalar colour = path_colour){   
    if(arc.radius != 0){
        // Draw an arc given the visual parameter of a map and the parameters of an arc
        arc_param arc_angles = calculate_arc_drawing_angles(arc);        

        cv::Point center_scaled;
        center_scaled.x = arc.center.x*img_map_def.scale*TO_CM;
        center_scaled.y = arc.center.y*img_map_def.scale*TO_CM;
        //Change of coordinate system (Img(0,0)--> left top corner, Map(0,0) left down corner)
        center_scaled = coord_map_to_img(center_scaled, img_map_def);
        double radius_scaled = arc.radius*img_map_def.scale*TO_CM; 
        
        cv::ellipse(img_map_def.img_map, center_scaled, cv::Size(radius_scaled, radius_scaled), 
                0, arc_angles.start_angle, arc_angles.end_angle, colour,1,15,0);
      
    }
}


/* Extract arc_extract parameters from pair of Points ------------------------*/

arc_extract to_arc_extract_type(Point pt1, Point pt2, bool calc_length=true){
    //Transform a pair of Point type into a arc_extract type
    // calc_length - (1) Calculate length (0) Do not, by default
    arc_extract result;
    result.start_point = pt1;
    result.end_point = pt2;
    result.radius = 0;
    result.center = Point(0,0);
    result.LSR = 1;

    if (calc_length){
        result.length = sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2));
    }
    else{
        result.length = 0;
    }   

    return result;    
}


/* Draw a line-------------------------------------------*/

void draw_line(arc_extract line, img_map_def img_map_def, cv::Scalar colour = path_colour){
    cv::Point start_pt_scaled, end_pt_scaled;
    //Scale points for visualizing
    start_pt_scaled.x = line.start_point.x*img_map_def.scale*TO_CM;
    start_pt_scaled.y = line.start_point.y*img_map_def.scale*TO_CM;
    end_pt_scaled.x = line.end_point.x*img_map_def.scale*TO_CM;
    end_pt_scaled.y = line.end_point.y*img_map_def.scale*TO_CM;
    //Change of coordinate system (Img(0,0)--> left top corner, Map(0,0) left down corner)
    start_pt_scaled = coord_map_to_img(start_pt_scaled, img_map_def);
    end_pt_scaled = coord_map_to_img(end_pt_scaled, img_map_def);

    cv::line(img_map_def.img_map, start_pt_scaled, end_pt_scaled, colour,1,16,0);
}


/* Draw one of the segment (left curve, right curve or arc)---------------------------*/
void draw_dubins_segment(arc_extract dubins_segment, img_map_def img_map_def, 
                                                    cv::Scalar colour = path_colour){
    if(dubins_segment.LSR == 1){   //Line
        draw_line(dubins_segment, img_map_def, colour);
    }
    else if(dubins_segment.LSR == 0 || dubins_segment.LSR == 2){  //Left or Right arc
        draw_arc(dubins_segment, img_map_def, colour);
    }
    else{
        printf("draw_dubins_segment ERROR: Unknown LSR");
    }
}


/* Draw robot position-------------------------------------------*/

void draw_robot(float robot_x, float robot_y, float robot_theta, img_map_def img_map_def, 
                            cv::Scalar colour = robot_colour, bool show_direction=false){
    if(show_direction){
        //TO_BE_IMPLEMENTED
        // Arrow pointing to orientation of the robot
        // cv::arrowedLine 	( 	InputOutputArray  	img,
		// Point  	pt1,
		// Point  	pt2,
		// const Scalar &  	color,
		// int  	thickness = 1,
		// int  	line_type = 8,
		// int  	shift = 0,
		// double  	tipLength = 0.1 )	
    }
    else{
        Point robot_pose = Point(robot_x,robot_y);
        draw_point(robot_pose, img_map_def, colour);
    }                  
}


/* Draw victim position and number -------------------------------------------*/

void draw_victim(std::pair<int,Polygon> victim, img_map_def img_map_def, 
                        bool show_contour = false){
    
    //char victim_number = victim.first;
    std::string victim_number = std::to_string(victim.first + 1); //start at 0
    Point rel_number_label = Point(0.01,0); //Relative label w.r.t victim centroid
    cv::Point abs_number_label_scaled;  //absolute position of label scaled.
    Point centroid;
    float centroid_x, centroid_y;
    centroid_x = 0;
    centroid_y = 0;    
    Polygon victim_poly = victim.second;

    //std::cout << "victim number: " << victim.first << std::endl;

    //Calculate centroid  
    float n_points = victim_poly.size();
    for (int i=0;i<n_points;i++){
      //std::cout << "Centroid value: " << centroid_x << ", " << centroid_y << std::endl;
      centroid_x += victim_poly[i].x;
      centroid_y += victim_poly[i].y;
      //std::cout << "Centroid sum: " << centroid_x << ", " << centroid_y << std::endl;   
    }
    centroid = Point(centroid_x/n_points, centroid_y/n_points);
    //std::cout << "Centroid: " << centroid_x/n_points << ", " << centroid_y/n_points << std::endl;

    if(show_contour){
        draw_polygon(victim_poly, img_map_def, victim_colour);
        draw_point(centroid, img_map_def, cv::Scalar(0,0,0));
    }
    else{        
        draw_point(centroid, img_map_def, victim_colour);
    }    
    //Draw number    
    //Calculate scaled
    abs_number_label_scaled.x = (centroid.x + rel_number_label.x)*img_map_def.scale*TO_CM;
    abs_number_label_scaled.y = (centroid.y + rel_number_label.y)*img_map_def.scale*TO_CM;

    //Change of coordinate system (Img(0,0)--> left top corner, Map(0,0) left down corner)
    abs_number_label_scaled = coord_map_to_img(abs_number_label_scaled, img_map_def);

    cv::putText(img_map_def.img_map, victim_number, abs_number_label_scaled,
                         0,0.8,victim_colour,1,16,false);

}


/* Draw tests-------------------------------------------*/

//show all the drawing functions in action in a single image.
void draw_test(std::vector<Polygon> poly_list, float x, 
              float y, float theta, std::vector<std::pair<int,Polygon>> victim_list,
              img_map_def map_param ){


    /* Drawing polygons-------------------------------------------*/

    Polygon poly;              
    // //Code for drawing a single polygon
    // poly = poly_list[0];
    // draw_polygon(poly, map_param);

    //Code for printing all polygons   
    for (size_t i = 0; i<poly_list.size(); i++){
      poly = poly_list[i];
      draw_polygon(poly, map_param);
    }

    
    /* Drawing points-------------------------------------------*/

    //Code for single point
    Point eg_point;
    eg_point.x = 0.75;
    eg_point.y = 0.5;
    draw_point(eg_point, map_param);

    // //Generate random pointst
    // std::vector<Point> eg_points;    
    // for(int i=0;i<1000;i++){
    //   int x_rand = rand() % 150 + 1; //Generate random sample
    //   int y_rand = rand() % 100 + 1; 
    //   //std::cout << x_rand << "," << y_rand << std::endl;
    //   eg_points.emplace_back(x_rand,y_rand);
    // }    
    // //Add points to map image    
    // for (int i=0;i<1000;i++){
    //   draw_point(eg_points[i], map_param);      
    // }


    /* Drawing semicircle-------------------------------------------*/
    arc_extract semi_circle;
    semi_circle.start_point = Point(0.1,0.5);
    semi_circle.end_point = Point(0.3,0.5);
    semi_circle.radius = 0.1;
    semi_circle.center = Point(0.2,0.5);
    semi_circle.length = 15;  
    semi_circle.LSR = 0;

    //draw_arc(semi_circle, map_param);
    draw_dubins_segment(semi_circle, map_param);

    // Test on calculated arc angles
    // arc_param semicircle_param;
    // semicircle_param = calculate_arc_drawing_angles(semi_circle);
    // std::cout << "SemiCircle Rotation angle: " << semicircle_param.rotation_angle << std::endl;
    // std::cout << "SemiCircle Angle btw cs & ce: " << semicircle_param.angle_cs_ce << std::endl;

    //Test on center
    // Point center_test;
    // int LSR = 1;
    // center_test = find_center(semi_circle.start_point,semi_circle.end_point,semi_circle.radius, LSR);
    // std::cout << "center_test: " << center_test.x << ", " << center_test.y << std::endl;

    /* Drawing arc-------------------------------------------*/
    arc_extract arc;
    arc.radius = 0.2;
    arc.center = Point(0.9,0.3);
    arc.start_point = Point(arc.center.x - arc.radius, arc.center.y);
    arc.end_point = Point(arc.center.x, arc.center.y + arc.radius);    
    arc.length = 15;  
    arc.LSR = 0;

    //draw_arc(arc, map_param);
    draw_dubins_segment(arc, map_param);
    

    // Test on calculated arc angles
    // arc_param arc_param;
    // arc_param = calculate_arc_drawing_angles(arc);
    // std::cout << "Arc Rotation angle: " << arc_param.rotation_angle << std::endl;
    // std::cout << "Arc Angle btw cs & ce: " << arc_param.angle_cs_ce << std::endl;

   
    /* Drawing a line-----------------------------------------*/    
    Point pt1 = Point(0.15,0.15);
    Point pt2 = Point(0.90,0.90);
    arc_extract line_test;
    //Transform pair of points into line_extract type
    line_test = to_arc_extract_type(pt1,pt2,true);

    // Test on to_arc_extract_type
    // std::cout << "start_point: " << line_test.start_point.x << ", " 
    //           << line_test.start_point.y << std::endl;
    // std::cout << "end_point: " << line_test.end_point.x << ", " 
    //           << line_test.end_point.y << std::endl;
    // std::cout << "lenght: " << line_test.length << std::endl;

    //drawing line
    //draw_line(line_test,map_param);
    draw_dubins_segment(line_test,map_param);
    
    
    /* Drawing robot position-----------------------------------------*/
    draw_robot(x,y,theta,map_param);       

    /* Drawing victims position and number-----------------------------*/    
    for(int i=0;i<victim_list.size();i++){      
      draw_victim(victim_list[i], map_param);
    }    
}


/* Draw Point in Polygon Test -------------------------------------------*/
bool point_liesin_polygon(Point pt,std::vector<Polygon> cv_poly_list)
{
    std::vector<cv::Point> contour;
    Polygon obstacle;
    cv::Point cv_point_temp;

    cv::Point2f test_pt;
	test_pt.x = pt.x*100*720/150;
	test_pt.y = pt.y*100*576/100;

    for (size_t j = 0; j<cv_poly_list.size(); j++){  

        obstacle=cv_poly_list[j];
        contour.clear();//vector clearing for next test

        for (size_t k = 0; k<obstacle.size(); k++){  

            cv_point_temp.x=obstacle[k].x*100*720/150;//img_map_w*x_ob/map_w
            cv_point_temp.y=obstacle[k].y*100*576/100;
            contour.push_back(cv_point_temp);

        }//inner for loop

        int result=cv::pointPolygonTest(contour, test_pt, false);
        double dist=cv::pointPolygonTest(contour, test_pt, true);

        switch(result)
        {
            case 1:  //1 = (point lies inside polygon)

                std::cout << "Gkiri:case 111111111 Inside polygon result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y<<  std::endl;
                return true;
                break;
            case 0:  //0 (point lies on edge of polygon)

                std::cout << "Gkiri:case 00000000 Inside polygon result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y<<  std::endl;
                return true;
                break;
            case -1:   //-1 (point lies outside polygon)

                std::cout << "Gkiri:case -1-1-1-1-1-1-1 Inside polygon result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y<<  std::endl;
                if((dist< 0.0 && dist > -10.0))
                {
                    return true;//Point outside polygon but bit closer to edge so remove this with some threshold distance   
                }
                break;
            default:
                std::cout << "ERROR from function" << std::endl ;
                break;
        } 

        std::cout << "Gkiri:point_polygon Outside polygon -----------------result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y <<  std::endl;
        
    }//outer loop

    return false;//false = point doesnot lie in polygon

}


/*   Visualise PRM motion planning */

void draw_motion_planning(const std::vector<Polygon>& obstacle_list,img_map_def *map_param)
{
    Polygon poly;
    std::vector<Polygon> poly_list = obstacle_list; //for printing the original polygons
    //std::vector<Polygon> poly_list = inflated_obstacle_list; //inflated polygons
    
    std::cout <<"draw_polygon in Map start st" <<  std::endl;
    for (size_t i = 0; i<poly_list.size(); i++){
      poly = poly_list[i];
      draw_polygon(poly, *map_param);
    }

    std::cout <<"Random Sampling in Map start st" <<  std::endl;
    /* Generate random pointst */
    std::vector<Point> random_points;    
    for(int i=0;i<1000;i++){
      float x_rand = (rand() / (double) RAND_MAX) * 1.50; //Generate random sample
      float y_rand = (rand() / (double) RAND_MAX) * 1.00;

      std::cout <<"Random Sampling in Map" << x_rand << "," << y_rand << std::endl;
      random_points.emplace_back(x_rand,y_rand);
    }

    int count=0;  
    for (int z=0;z<1000;z++){

        if(!point_liesin_polygon(random_points[z] ,  obstacle_list)) 
        {
            draw_point(random_points[z], *map_param); 
            count++;
            std::cout << "Gkiri :: Motion sample After count= " << count <<std::endl; 
        }
           
    }

}