#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <vector>

#include "mission_apis.hpp"

void get_gate_pose(const Polygon& gate, double map_h, double map_w, double robot_length, double* gate_pose){    
    double RAD2DEG = 180.0/M_PI;
    double x_sum = 0;
    double y_sum = 0;
    int n_vertex = 0;
    double delta = 0.025;
    std::vector<double> dist_to_wall; //[left,right,up,down]    
    double lowest_dist = 100;
    int closest_wall;

    // Center point of rectangle
    for(Point vertex : gate){
        std::cout << vertex.x <<","<<vertex.y<<std::endl;
        x_sum += vertex.x;
        y_sum += vertex.y;
        n_vertex++;
    }
    gate_pose[0] = x_sum/n_vertex; //xc
    gate_pose[1] = y_sum/n_vertex; //yc

    //gate_pose
    dist_to_wall.push_back(gate_pose[0]); //left
    dist_to_wall.push_back(map_w - gate_pose[0]); //right
    dist_to_wall.push_back(map_h - gate_pose[1]); //up    
    dist_to_wall.push_back(gate_pose[1]); //down
    
    //Get closest wall
    for(int i=0;i<dist_to_wall.size();i++){
        if(dist_to_wall[i] < lowest_dist){
            lowest_dist = dist_to_wall[i];
            closest_wall = i;
        }
    }
    //std::cout << "lowest dist:" << lowest_dist << std::endl;

    //Set pose and heading angle depending on closest wall
    switch (closest_wall){
    case 0: //left
        gate_pose[0] = robot_length/2 + delta;
        gate_pose[2] = M_PI;
        //std::cout << "closes wall: Left wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    case 1: //right
        gate_pose[0] = map_w - (robot_length/2 + delta);
        gate_pose[2] = 0;
        //std::cout << "closes wall: Right wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    case 2: //up
        gate_pose[1] = map_h - (robot_length/2 + delta);
        gate_pose[2] = M_PI/2;
        //std::cout << "closes wall: Up wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    case 3: //down
        gate_pose[1] = robot_length/2 + delta;
        gate_pose[2] = -M_PI/2;
        //std::cout << "closes wall: Down wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    
    default:
        printf("No closest wall found\n");
        break;
    } 
}

mission_output_0 mission_0(dubins_param dubins_param, double start_pose[3], double gate_pose[3]){

    DubinsCurvesHandler dubins_handler(dubins_param.k_max, dubins_param.discretizer_size);
    Path path;
    double three_seg[3];
    struct mission_output_0 mission_0;

    DubinsCurve dubins_path = dubins_handler.findShortestPath(start_pose[0],start_pose[1],start_pose[2],
                                                gate_pose[0], gate_pose[1], gate_pose[2]);

    for(int j = 0; j < dubins_path.discretized_curve.size(); j++){
        path.points.emplace_back(dubins_path.discretized_curve[j].s, 
              dubins_path.discretized_curve[j].xf, dubins_path.discretized_curve[j].yf, 
              dubins_path.discretized_curve[j].thf, dubins_path.discretized_curve[j].k);
    }   

    mission_0.path = path;
    mission_0.dubins_path = dubins_path;

    return mission_0;
}


mission_output_12 mission_1(PRM_param PRM_param, dubins_param dubins_param, double start_pose[3], 
    double gate_pose[3], std::vector<Point> bias_points, double delta){
    
    Path path;
    struct mission_output_12 mission_12;

    //create PRM instance
    PRM PRM_obj(PRM_param.obstacle_list, PRM_param.map_w, PRM_param.map_h, PRM_param.n_samples);
    //call prm planner
    path = PRM_obj.prm_planner(start_pose, gate_pose, bias_points, PRM_param.max_dist,
        PRM_param.min_dist, dubins_param, delta);
    
    //Retrieve PRM variables for drawing purposes
    std::vector<Point> free_space_points = PRM_obj.get_free_space_points();
    std::vector<std::pair<Point, std::vector<Point> >> prm_graph = PRM_obj.get_prm_graph();
    std::vector<Point> global_planner_path = PRM_obj.get_global_planner_path(); 

    //save output
    mission_12.path = path;
    mission_12.free_space_points = free_space_points;
    mission_12.prm_graph = prm_graph;
    mission_12.global_planner_path = global_planner_path;
    mission_12.path_final_draw = PRM_obj.path_final_draw;
    mission_12.failed_paths_draw = PRM_obj.failed_paths;

    return mission_12; 
}

mission_output_12 mission_2(PRM_param PRM_param, dubins_param dubins_param, double start_pose[3], 
    double gate_pose[3], std::vector<std::pair<int,Polygon>> victim_list, double delta){
    
    Path path;
    struct mission_output_12 mission_12;
    Point victim_centroid;
    std::vector<Point> bias_points;

    //create PRM instance
    PRM PRM_obj(PRM_param.obstacle_list, PRM_param.map_w, PRM_param.map_h, PRM_param.n_samples);
    
    //set bias points
    for(std::pair<int,Polygon> victim : victim_list){
      victim_centroid = get_polygon_centroid(victim.second);
      bias_points.push_back(victim_centroid);
      //draw
      //draw_victim(victim, map_param);
    }
    
    //call prm planner
    path = PRM_obj.prm_planner(start_pose, gate_pose, bias_points, PRM_param.max_dist,
        PRM_param.min_dist, dubins_param, delta);
    
    //Retrieve PRM variables for drawing purposes
    std::vector<Point> free_space_points = PRM_obj.get_free_space_points();
    std::vector<std::pair<Point, std::vector<Point> >> prm_graph = PRM_obj.get_prm_graph();
    std::vector<Point> global_planner_path = PRM_obj.get_global_planner_path(); 

    //save output
    mission_12.path = path;
    mission_12.free_space_points = free_space_points;
    mission_12.prm_graph = prm_graph;
    mission_12.global_planner_path = global_planner_path;
    mission_12.path_final_draw = PRM_obj.path_final_draw;
    mission_12.failed_paths_draw = PRM_obj.failed_paths;

    return mission_12; 
}
