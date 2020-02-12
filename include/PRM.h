#pragma once

#include <vector>
#include <list>

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include "Utils.hpp"

class  PRM {

public:

    PRM(std::vector<Polygon> polygons_list);
    PRM(std::vector<Polygon> polygons_list, double cspace_width, double cspace_height, int N, double scale);

    ~PRM();    

    /*Function to check Point lies in polygon */
    bool point_liesin_polygon(Point pt,std::vector<Polygon> cv_poly_list, double scale);

    /*Function to return private variables*/
    std::vector<Point> get_free_space_points();
    std::vector<Point> get_global_planner_path(); 
    std::vector<std::pair<Point, std::vector<Point> >> get_prm_graph();

    /*Functions to set private variables*/
    void set_prm_graph(std::vector<std::pair<Point, std::vector<Point> >> prm_graph_test);
    void set_free_space_points(std::vector<Point> free_space_points_test);

    /*variables for testing purposes*/
    std::vector<arc_extract> path_final_draw;
    std::vector<arc_extract> failed_paths;
    std::vector<Point> knn_draw;
    std::vector<Point> collision_points;

    /*final path*/
    double path_length; 

    
    /**
     * Generate random point inside configuration space of map
     * updated to free_space_points vector
     *
     * Avoid points inside polygon     
     *
     * @returns - zero on successful completion, or the result of the callback
     */
    void generate_random_points();

    /**
     * Generate grpah from list of free_space_points by adding to vertices
     *
     * Iterate over vertices for connecting edges and paralelly check for collision while 
     * constructing roadmap graph
     *
     * implement Knearest neighbour of samples(q)
     * 
     * @param bias_points - Points through where the robots must pass (E.g start,end,victim1,etc)
     * @param max_dist - Maximum radius within it looks for nearest neighbors
     * @param min_dist - Minimum radius within it does not look for nearest neighbors.
     */
    void local_planner(std::vector<Point> bias_points, double max_dist, double min_dist);

    /**
     * Plan optimal path from roadmap graph
     *
     * Can use A* or Dijiktra
     * 
     * @param start - start position of required path
     * @param goal - goal position of required path
     * @output - updates to global_planner_path
     */
    void global_planner(Point start,Point goal);


    /**
     * Generates a vector of dubin's segments by connecting nodes in glopal_planner_path
     *  
     * @param start_theta - initial orientation of the robot
     * @param goal_theta - final orientation of the robot
     * @param dubins_param - k_max and discretize step size for dubins
     * @param delta - tune angle in degrees when a collision happen
     * @output - return final path to be followed by the robot
     */
    Path dubins_planner(float start_theta, float goal_theta, struct dubins_param, double delta);


    /**
     * Given a set of bias points and a vector of samples, it builds a roadmap 
     * (i.e, a graph of edges between samples)
     *  
     * @param bias_points - Points through where the robots must pass (E.g start,end,victim1,etc)
     * @param max_dist - Maximum radius within it looks for nearest neighbors
     * @param min_dist - Minimum radius within it does not look for nearest neighbors.     
     * @output - updated prm_graph (roadmap)
     */
    void build_roadmap(std::vector<Point> bias_points, double max_dist, double min_dist);


    /**
     * Given a starting pose, a goal pose and a vector of bias points (points through
     * where the robot is required to pass by), it updates the variable path used by 
     * the robot to understand the route to follow
     *  
     * @param start_pose  - initial pose of the robot
     * @param goal_pose - final pose of the robot
     * @param bias_points - vector of bias points 
     * @param max_dist - Maximum radius within it looks for nearest neighbors
     * @param min_dist - Minimum radius within it does not look for nearest neighbors.
     * @param dubins_param - k_max and discretize step size for dubins
     * @param delta - tune angle in degrees when a collision happen
     * @return Path - return final path to be followed by the robot
     */    
    Path prm_planner(double* start_pose, double* goal_pose, std::vector<Point> bias_points,
        struct dubins_param, double delta);


    /**
     * Given a global planner path, refines the path to have the least amount of nodes
     *  
     * @param tmp_global_planner_path - Given global planner path to be refined     
     * @returns - refined global planner path
    */
    std::vector<Point> refine_global_planner_path(std::vector<Point> tmp_global_planner_path);


private:

    std::vector<std::pair<Point, std::vector<Point> >> prm_graph; //G(V,E)
    std::vector<Point> free_space_points;
    std::vector<Polygon> obstacle_list;  //Store this in costructor
    std::vector<Point> global_planner_path;    
    double cspace_width; //width of sample space
    double cspace_height; //height of sample space
    int N; //Number of samples
    double scale; //scale for point lies in polygon algorithm
    
};