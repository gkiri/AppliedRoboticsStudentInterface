#include <vector>
#include <list>
#include "KDTree.hpp"

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"


/*Variables*/
struct triangle_angles{ //Angles of a triangle
    double beta_1, beta_2, beta_3;
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

// /**
//  * Given two points of type Point, determine whether they are 
//  * the same point or not
//  * 
//  * @param pt1 - first point
//  * @param pt2 - second point
//  * @output - return true if they are the same point, false otherwise
// */
// bool same_point(Point pt1, Point pt2);

/**
 * Given two points of type point_t, determine whether they are 
 * the same point or not
 * 
 * @param pt1 - first point
 * @param pt2 - second point
 * @output - return true if they are the same point, false otherwise
*/
bool same_point(point_t pt1, point_t pt2);

