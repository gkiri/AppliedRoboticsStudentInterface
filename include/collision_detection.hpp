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

#include "dubins_curve.hpp"


bool linelineIntersection(Point A,Point B,Point C,Point D,Point *X);
int linecircleIntersection(double r, double a , double b, double c ,std::vector<Point>& X);
//bool lineArcIntersection(Point Line_Start,Point Line_End,Point Arc_Start,Point Arc_End,double r,Point center);
bool lineArcIntersection(Point Line_Start,Point Line_End,Point Arc_Start,Point Arc_End,double r,Point center,std::vector<Point>& cal_points);
bool  Construct_Bounding_Box(Polygon& input , Polygon& output);
bool  Highlevel_Box_dubins_check(std::vector<Polygon> box_obstacle_list,struct arc_extract *arc);
void Build_All_Bounding_Box(std::vector<Polygon> obstacle_list,std::vector<Polygon>& Box_list);
bool  Process_Box_line_check(std::vector<Polygon>& Box_list,struct arc_extract& segment);
bool  Process_Box_arc_check(std::vector<Polygon>& Box_list,struct arc_extract& segment);
