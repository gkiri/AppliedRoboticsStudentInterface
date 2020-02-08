#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

/**
 * Return the final pose of the robot at the gate (keeping a safe distance to the wall
 * and with a proper angle depending on the location of the gate)
 * 
 * @param gate - polygon representing the gate location
 * @param gate_pose - pose at the gate (double gate_pose[3])
 * @param map_h - height of map (y-axis)
 * @param map_w - width of map (x-axis)
 * @output - pose at gate passed by reference
*/
void get_gate_pose(const Polygon& gate, double map_h, double map_w, double robot_length,  double* gate_pose);