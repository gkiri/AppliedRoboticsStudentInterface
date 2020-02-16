#pragma once

#include <vector>
#include <list>
#include "KDTree.hpp"
#include "DubinsCurvesHandler.hpp"

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string> 
#include <algorithm>


/*Variables*/
struct triangle_angles{ //Angles of a triangle
    double beta_1, beta_2, beta_3;
};

struct arc_extract{ // dubins segment structure for collision and drawing purposes
    Point start_point;
    Point end_point;
    float radius;
    Point center;
    float length;
    int LSR; // Letft - Straight - Right indicator (0,1,2)
};
//Struct for ellipse calculations
struct arc_param{
    double start_angle;
    double end_angle;
};
//PRM planners
struct dubins_param{
    double k_max, discretizer_size;
};
struct PRM_param{
    double map_w, map_h, max_dist, min_dist;
    std::vector<Polygon> obstacle_list;
    int rand_samples;
    int gauss_samples;
    double scale;
};
//Struct for line equation parameters
struct line_eq_param{
    double m,b; // y=mx+b
};


/**
 * Compute the angles within a triangle (in radians)
 * 
 * @param A - First vertex of the triangle
 * @param B - Second vertex of the triangle
 * @param C - Third vertex of the triangle
 * @output - return the triangle angles beta_1, beta_2 and beta_3
*/
triangle_angles compute_triangle_angles(Point A,Point B, Point C);

/**
 * Compute the arcosin of x taking into account the edges cases of x=1 and x=-1
 * 
 * @param x - Value to which calculate the arcosin 
 * @output - return the arcosin in radians
*/
double SafeAcos (double x);

/**
 * Calculates the aproximate optimal heading angle for the middle point in a triplet
 * of consecutive dubins points 
 * 
 * @param qs - dubins start point
 * @param qm - dubins mid point
 * @param qe - dubins end point
 * @output - updates qm and qe angles by reference
*/
void compute_heading_angle(double* qs,double* qm,double* qe);

/**
 * Given a point and a reference point (use as the origin), return in which quadrant
 * the given point is wrt the reference point
 * 
 * @param pt - given point
 * @param ref - reference point (origin)
 * @output - return integer which represents the quadrant (1,2,3,4)
*/
int compute_quadrant(Point pt, Point ref);

/**
 * Given data type Point, convert it into data type point_t
 * (Conversion required to use KDTree library)
 * 
 * @param Pt - given data type Point
 * @output - return data type point_t
*/
point_t Point_to_point_t(Point Pt);

/**
 * Given data type point_t, convert it into data type Point
 * (Conversion required to use KDTree library)
 * 
 * @param pt_t - given data type point_t
 * @output - return data type Point
*/
Point point_t_to_Point(point_t pt_t);


/**
 * Given two points of type point_t, determine whether they are 
 * the same point or not
 * 
 * @param pt1 - first point
 * @param pt2 - second point
 * @output - return true if they are the same point, false otherwise
*/
bool same_point(point_t pt1, point_t pt2);

/**
 * Convert a DubinsCurve into an arc_extract, suitable for collision
 * detection and drawing
 * 
 * @param three_seg - Array of 3 arc_extract
 * @param x0 - x of start point of dubins
 * @param y0 - y of start point of dubins
 * @param dubins_path - DubinsCurve composed by 3 DubinsArc
 * @output - return by reference the array of 3 arc_extract which represents a DubinsCurve
*/
void create_three_seg(struct arc_extract three_seg[3], double x0, double y0, DubinsCurve dubins_path);

/**
 * Given a DubinsCurve concatenate its discretization into a Path structure, updating 
 * accordingly the s counter of the Path
 * 
 * @param path - Path container
 * @param dubins_path - DubinsCurve container
 * @param discritizer_size - step size od discritization
 * @output - update by reference the Path with the concatenate version of both inputs
*/
void concatenate_dubins_path(Path& path, DubinsCurve dubins_path, double discritizer_size);



/**
 * Compute the center Point of a curve by passing the arc_extract values of it
 * 
 * @param start - start Point of curve
 * @param end - end Point of curve
 * @param radius - radius of curve
 * @param length  - left/straigth/right indicator of curve
 * @output Point - center Point
*/
Point compute_center(Point start,Point end,float radius,float length, int LSR);


/**
 * Compute start and end angles corresponding to an arc (taking the center as the origin
 * , angles are measured from the positive x-axis following clockwise direction)
 * 
  * @param arc  - arc_extract structure which includes all parameters to define an arc
 * @output arc_param - arc_param strcuture which includes start and end angle.
*/
arc_param calculate_arc_drawing_angles(arc_extract arc);


/**
 * Compute the centroid of any polygon
 * 
 * @param poly  - Given polygon with structure Polygon
 * @output Point - centroid of polygon
*/
Point get_polygon_centroid(Polygon poly);


/**
 * Load an given parameter from a configuration file .txt given the full directory of the
 * configuration file and the name of the parameter
 * Note: It returns only double type
 * 
 * @param config_dir  - full directory of configuration file
 * @param param_name - name of parameter
 * @returns - value of parameter
*/
double load_config_param(std::string config_dir, std::string param_name);


/**
 * Calculate euclidean distance between two points
 * 
 * @param pt1  - first given point
 * @param pt2 - second given point
 * @returns - euclidean distance between given points
*/
double dist_2points(Point pt1, Point pt2);