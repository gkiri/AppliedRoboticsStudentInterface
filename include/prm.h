#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>

class prm
{
private:
    /*vector of sample in free space (eliminate samples from obstacle space)*/
    std::vector<Point> sample_points;   
public:
    prm(/* args */);
    ~prm();
    /*Generate samples in the free space within the map*/
    sample_generator();
    /*Creating the graph G=(V,E)* --> check page3 http://disi.unitn.it/~palopoli/courses/ECLnew/ECLlect12.pdf/*/
    generate_graph();
    /*Generate road map from an init and goal point*/
    local_planner(); 
    /*Find the shortest path from roadmap* --> Use boost library A*, Djistra or implement yourself*/
    global_planner();
    /*Create dubins curve vector from global planner (Use ust list of vertexes)*/
    //Collision between dubins curve and obstacles has to be rechecked here.
    dubins_planner();
    //Output: Update variable associated with Path& path.

};

prm::prm(/* args */)
{
}

prm::~prm()
{
}


