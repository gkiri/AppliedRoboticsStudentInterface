#include <iostream>
#include <vector>
#include <map>
#include <cmath>

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "Utils.hpp"


/**
 * line line Intersection  detection
 *  * 
 * @param A - start point of line1
 * @param B - end point of line1
 * @param C - start point of line2
 * @param D - end point of line2
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool linelineIntersection(Point A,Point B,Point C,Point D,Point *X);


/**
 * line circle Intersection  detection
 *  * 
 * @param r - radius
 * @param a - line coeeficients a
 * @param b - line coeeficients b
 * @param c - line coeeficients c
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
int linecircleIntersection(double r, double a , double b, double c ,std::vector<Point>& X);
//bool lineArcIntersection(Point Line_Start,Point Line_End,Point Arc_Start,Point Arc_End,double r,Point center);


/**
 * line ARC Intersection  detection
 *  * 
 * @param line - line
 * @param arc - arc data
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool lineArcIntersection(struct arc_extract line,struct arc_extract arc);


/**
 * line arc Intersection  detection
 *  * 
 * @param line - line
 * @param arc - arc
 * @param intersect_points - intersect_points
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool lineArcIntersection_prof(struct arc_extract line,struct arc_extract arc,std::vector<Point>& intersect_points);
bool  Construct_Bounding_Box(Polygon& input , Polygon& output);
void construct_line_structure(arc_extract& line_data,Point Arc_Start ,Point Arc_End);
int get_line_circle_intersection(Point linestart, Point lineend,Point center, double radius,Point& firstIntersection,Point& secondIntersection);


/**
 * is_line_colliding_circle
 *  * 
 * @param linestart - start point of line
 * @param lineend - end point of line
 * @param center - center
 * @param radius - radius of circle
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool is_line_colliding_circle(Point linestart, Point lineend, Point center, double radius);

/**
 * Detect pointlies inside given polygons
 *  * 
 * @param cv_poly_list - obstacle_list
 * @param pt - point
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool Detect_point_liesin_polygon(Point pt,std::vector<Polygon> cv_poly_list);


/**
 * Dubins 3 segment check with globally using narrow and broad phase
 * 
 * @param obstacle_list - obstacle_list
 * @param segment - 3 segments
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool  Highlevel_Box_dubins_check(std::vector<Polygon>& box_obstacle_list,struct arc_extract *arc);


/**
 * construct bounding box from polygons
 * 
 * @param obstacle_list - obstacle_list
 * @param segment - line segment
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
void  Build_All_Bounding_Box(std::vector<Polygon>& obstacle_list,std::vector<Polygon>& Box_list);



/**
 * Broad check line with all boxes
 * 
 * @param obstacle_list - obstacle_list
 * @param segment - line segment
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool  Process_Box_line_check(std::vector<Polygon>& Box_list,struct arc_extract& segment);

/**
 * Broad check arc with all boxes
 * 
 * @param obstacle_list - obstacle_list
 * @param segment - line segment
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool  Process_Box_arc_check(std::vector<Polygon>& Box_list,struct arc_extract& segment);



/**
 * Broad check line with all obstacle_list but not boxes
 * 
 * @param obstacle_list - obstacle_list
 * @param segment - line segment
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool  Process_Box_line_check_obstacles(std::vector<Polygon>& obstacle_list,struct arc_extract& segment);



/**
 * Broad check arc with all obstacle_list but not boxes
 * 
 * @param obstacle_list - obstacle_list
 * @param segment - arc segment
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool  Process_Box_arc_check_obstacles(std::vector<Polygon>& obstacle_list,struct arc_extract& arc);



/**
 * Narrow check arc with all polygons in Narrow phase
 * 
 * @param obstacle_list - obstacle_list
 * @param segment - arc segment
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool  narrow_polygon_obstacles_arc_check(std::vector<Polygon>& obstacle_list,struct arc_extract& arc);


/**
 * Narrow check Line with all polygons in Narrow phase
 * 
 * @param obstacle_list - obstacle_list
 * @param segment - Line segment
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool  narrow_polygon_obstacles_line_check(std::vector<Polygon>& obstacle_list,struct arc_extract& segment);


/**
 * Globally check Line with all obstacles by checking broad phase and Narrow phase
 * Broad phase includes Box checks and Narrow phase inlcudes polygon checks
 * @param obstacle_list - obstacle_list
 * @param segment - Line segment
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool Global_Line_check(std::vector<Polygon>& obstacle_list,struct arc_extract& segment);



/**
 * Globally check arc with all obstacles by checking broad phase and Narrow phase
 * Broad phase includes Box checks and Narrow phase inlcudes polygon checks
 * @param obstacle_list - obstacle_list
 * @param segment - Arc segment
 * 
 * @output - return the TRUE = Collision happened ,FALSE = No collision
*/
bool Global_Arc_check(std::vector<Polygon>& obstacle_list,struct arc_extract& segment);



