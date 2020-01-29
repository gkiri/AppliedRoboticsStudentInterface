#include <vector>
#include <list>

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"


#include "dubins_curve.hpp"

class  PRM {

public:

    PRM(std::vector<Polygon> polygons_list);

    ~PRM();    

    /*Function to check Point lies in polygon */
    bool point_liesin_polygon(Point pt,std::vector<Polygon> cv_poly_list);

    /*Function to return private variables*/
    std::vector<Point> get_free_space_points();
    std::vector<Point> get_global_planner_path(); 
    //@Ambike
    std::vector<std::pair<Point, std::vector<Point> >> get_prm_graph();

    /*Functions to set private variables*/
    void set_prm_graph(std::vector<std::pair<Point, std::vector<Point> >> prm_graph_test);
    void set_free_space_points(std::vector<Point> free_space_points_test);

    /*variables for testing purposes*/
    std::vector<arc_extract> final_path_draw;
    std::vector<Point> knn_draw;


    
    /**
     * Generate random point inside configuration space of map
     * updated to free_space_points vector
     *
     * Avoid points inside polygon
     * 
     * @param cspace_length - lengh of map - diameter of robot in circle
     * @param cspace_width  -width of map - diameter of robot in circle
     * @param N        - no:of samples to generate
     *
     * @returns - zero on successful completion, or the result of the callback
     */
    void generate_random_points(double cspace_length,double cspace_width, int N);

    /**
     * Generate grpah from list of free_space_points by adding  to vertices
     *
     * Iterate over vertices for conencting edges paralelly  check for collision while 
     * construct roadmap graph
     *
     * implement Knearest neighbour of samples(q)
     * 
     * @returns - zero on successful completion, or the result of the callback
     */
    void local_planner();

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
     * @param final_path - vector of dubin's segments 
     * @output - updates to final_path by reference
     */
    void dubins_planner(Path& final_path);


private:

    std::vector<std::pair<Point, std::vector<Point> >> prm_graph; //G(V,E)
    std::vector<Point> free_space_points;
    std::vector<Polygon> obstacle_list;  //Store this in costructor
    std::vector<Point> global_planner_path;// 
       
};