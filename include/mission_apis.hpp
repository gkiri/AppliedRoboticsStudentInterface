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

/**
 * Perform mission 0: Robot goes from actual position to the gate (no obstacles considered)
 * 
 * @param start_pose - starting pose of the robot
 * @param gate_pose - pose at the gate 
 * @output Path - Return path to be followed by the robot to complete the mission
*/
Path mission_0(double start_pose[3], double gate_pose[3]);

/**
 * Perform mission 1: Robot goes from actual position to the gate avoiding obstacles
 * 
 * @param start_pose - starting pose of the robot
 * @param gate_pose - pose at the gate 
 * @param inflated_obstacle_list - inflated obstacles including walls
 * @output Path - Return path to be followed by the robot to complete the mission
*/
Path mission_1(double start_pose[3], double gate_pose[3], std::vector<Polygon> inflated_obstacle_list);


/**
 * Perform mission 1.5: Robot goes from actual position to the gate avoiding obstacles and collecting
 * victims depending on the cost, being cost = reward - distance to victim
 * 
 * @param start_pose - starting pose of the robot
 * @param gate_pose - pose at the gate 
 * @param bias_points - bias points over where the robot must pass.
 * @param inflated_obstacle_list - inflated obstacles including walls * 
 * @output Path - Return path to be followed by the robot to complete the mission
*/
Path mission_1_5(double start_pose[3], double gate_pose[3], std::vector<Point> bias_points, std::vector<Polygon> inflated_obstacle_list);


/**
 * Perform mission 2: Robot goes from actual position to the gate avoiding obstacles and collecting
 * victims depending on the cost, being cost = reward - distance to victim
 * 
 * @param start_pose - starting pose of the robot
 * @param gate_pose - pose at the gate 
 * @param victim_list - list of victims. Each victim represented by its number and a polygon
 * @param inflated_obstacle_list - inflated obstacles including walls * 
 * @output Path - Return path to be followed by the robot to complete the mission
*/
Path mission_2(double start_pose[3], double gate_pose[3], std::vector<std::pair<int,Polygon>> victim_list, std::vector<Polygon> inflated_obstacle_list);



