#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <vector>

#include "mission_apis.hpp"

void get_gate_pose(const Polygon& gate, double map_h, double map_w, double robot_length, 
    double* gate_pose, double goal_delta){  

    double RAD2DEG = 180.0/M_PI;
    double x_sum = 0;
    double y_sum = 0;
    int n_vertex = 0;
    //double delta = 0.025;
    std::vector<double> dist_to_wall; //[left,right,up,down]    
    double lowest_dist = 100;
    int closest_wall;

    // Center point of rectangle
    for(Point vertex : gate){
        //std::cout << vertex.x <<","<<vertex.y<<std::endl;
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
        gate_pose[0] = robot_length/2 + goal_delta;
        gate_pose[2] = M_PI;
        //std::cout << "closes wall: Left wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    case 1: //right
        gate_pose[0] = map_w - (robot_length/2 + goal_delta);
        gate_pose[2] = 0;
        //std::cout << "closes wall: Right wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    case 2: //up
        gate_pose[1] = map_h - (robot_length/2 + goal_delta);
        gate_pose[2] = M_PI/2;
        //std::cout << "closes wall: Up wall, HA:" << gate_pose[2]*RAD2DEG << std::endl;
        break;
    case 3: //down
        gate_pose[1] = robot_length/2 + goal_delta;
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


mission_output_1 mission_1(PRM_param PRM_param, dubins_param dubins_param, double start_pose[3], 
    double gate_pose[3], std::vector<Point> bias_points, double delta){
    
    Path path;
    struct mission_output_1 mission_1;

    //create PRM instance
    PRM PRM_obj(PRM_param.obstacle_list, PRM_param.map_w, PRM_param.map_h, PRM_param.n_samples, PRM_param.scale);
    
    //Add start and end point to bias points    
    bias_points.insert(bias_points.begin(), Point(start_pose[0],start_pose[1]));
    bias_points.push_back(Point(gate_pose[0],gate_pose[1]));

    //Build the roadmap
    PRM_obj.build_roadmap(bias_points, PRM_param.max_dist, PRM_param.min_dist);    
   
    path = PRM_obj.prm_planner(start_pose, gate_pose, bias_points, dubins_param, delta);
    
    //Retrieve PRM variables for drawing purposes
    std::vector<Point> free_space_points = PRM_obj.get_free_space_points();
    std::vector<std::pair<Point, std::vector<Point> >> prm_graph = PRM_obj.get_prm_graph();
    std::vector<Point> global_planner_path = PRM_obj.get_global_planner_path(); 

    //save output
    mission_1.path = path;
    mission_1.free_space_points = free_space_points;
    mission_1.prm_graph = prm_graph;
    mission_1.global_planner_path = global_planner_path;
    mission_1.path_final_draw = PRM_obj.path_final_draw;
    mission_1.failed_paths_draw = PRM_obj.failed_paths;

    return mission_1; 
}

mission_output_1 mission_15(PRM_param PRM_param, dubins_param dubins_param, double start_pose[3], 
    double gate_pose[3], std::vector<std::pair<int,Polygon>> victim_list, double delta){
    
    Path path;
    struct mission_output_1 mission_1;
    std::vector<std::pair<int,Point>> victim_list_pos; //centroid instead of polygon
    Point start, goal, victim_centroid;
    //Point start_line_point, end_line_point;
    //std::vector<Point> victim_centroid_list, tmp_victim_centroid_list;
    std::vector<Point> bias_points;
    //DELETE start_to_victim_path, global_planner_path;    
    //int victim_index = 0;
    //double lowest_cost = 9999;    
    //std::vector<std::pair<double, int>> v_length_index_pair;
    //double L = 0;   
   
    //Set start and goal point
    start = Point(start_pose[0],start_pose[1]);
    goal = Point(gate_pose[0],gate_pose[1]); 

    //set bias points with victims in the order given by its number 
    //create vector of pairs: number + location of centroid   
    for(std::pair<int,Polygon> victim : victim_list){
      victim_centroid = get_polygon_centroid(victim.second);
      victim_list_pos.push_back(std::make_pair(victim.first, victim_centroid));            
    }    
    //Sort by number
    sort(victim_list_pos.begin(), victim_list_pos.end());
    //Create bias points: start + sorted victims + goal
    //Add start
    bias_points.push_back(start);
    //Add sorted victims
    for(std::pair<int,Point> victim_pos : victim_list_pos){
      bias_points.push_back(victim_pos.second);          
    }  
    //Add goal
    bias_points.push_back(goal);

    // //Save copy of only victim centroids for computing all combinations of victims
    // victim_centroid_list = bias_points;
    // //Add start and end point to bias points     
    // bias_points.insert(bias_points.begin(),start);
    // bias_points.push_back(goal);

    //create PRM instance
    PRM PRM_obj(PRM_param.obstacle_list, PRM_param.map_w, PRM_param.map_h, PRM_param.n_samples, PRM_param.scale);
        
    //Build the roadmap
    PRM_obj.build_roadmap(bias_points, PRM_param.max_dist, PRM_param.min_dist);

    // //Sort the victim list in terms of proximity to the start point
    // for(Point victim_location:victim_centroid_list){        
    //     L = 0; //Reset length
    //     //Call global planner
    //     PRM_obj.global_planner(start,victim_location);
    //     global_planner_path = PRM_obj.get_global_planner_path();
    //     start_to_victim_path = PRM_obj.refine_global_planner_path(global_planner_path);
    //     //Calculate distance for each victim
    //     for(int i=0; i<start_to_victim_path.size() - 1; i++){
    //         start_line_point = start_to_victim_path[i]; //start point of line
    //         end_line_point = start_to_victim_path[i + 1]; //end point of line

    //         //add up distance between points
    //         L += dist_2points(start_line_point, end_line_point);            
    //     }
    //     //save length and victim index pair into vector
    //     v_length_index_pair.push_back(std::make_pair(L, victim_index));
    //     victim_index++; 
    // }
    // //sort by distance
    // tmp_victim_centroid_list = victim_centroid_list; //make tmp copy
    // sort(v_length_index_pair.begin(), v_length_index_pair.end());
    // //save definitive order into victim centroid list
    // for(int i=0;i<v_length_index_pair.size();i++){
    //     victim_centroid_list[i] = tmp_victim_centroid_list[v_length_index_pair[i].second];
    // }      
   
    // //Bias points + victim list
    // bias_points = victim_centroid_list;
    // bias_points.insert(bias_points.begin(),start);
    // bias_points.push_back(goal);

    //call prm planner
    path = PRM_obj.prm_planner(start_pose, gate_pose, bias_points, dubins_param, delta);
    
    //Retrieve PRM variables for drawing purposes
    std::vector<Point> free_space_points = PRM_obj.get_free_space_points();
    std::vector<std::pair<Point, std::vector<Point> >> prm_graph = PRM_obj.get_prm_graph();
    std::vector<Point> global_planner_path = PRM_obj.get_global_planner_path(); 

    //save output
    mission_1.path = path;
    mission_1.free_space_points = free_space_points;
    mission_1.prm_graph = prm_graph;
    mission_1.global_planner_path = global_planner_path;
    mission_1.path_final_draw = PRM_obj.path_final_draw;
    mission_1.failed_paths_draw = PRM_obj.failed_paths;
    mission_1.collision_points = PRM_obj.collision_points;

    return mission_1; 
}

mission_output_2 mission_2(PRM_param PRM_param, dubins_param dubins_param, double start_pose[3], 
    double gate_pose[3], std::vector<std::pair<int,Polygon>> victim_list, double delta, 
    double victim_reward, double robot_speed){
    
    //variables
    Path path;
    struct mission_output_2 mission_2;       
    Point start, goal, start_line_point, end_line_point;
    Point victim_centroid; 
    std::vector<Point> victim_centroid_list, tmp_victim_centroid_list;
    std::vector<std::vector<Point>> v_comb;
    std::vector<std::pair<double, Path>> all_cost_path;
    std::vector<std::pair<double, std::vector<arc_extract>>> all_cost_pathdraw;
    std::pair<double, std::vector<arc_extract>> opt_cost_pathdraw;
    std::vector<Point> bias_points, start_to_victim_path, global_planner_path, free_space_points;
    std::vector<std::pair<Point, std::vector<Point> >> prm_graph;
    double path_cost;
    int opt_index; 
    int victim_index = 0;
    double lowest_cost = 9999;    
    std::vector<std::pair<double, int>> v_length_index_pair;
    double L_sv, L_vg;
    double L=0;


    //Set start and goal point
    start = Point(start_pose[0],start_pose[1]);
    goal = Point(gate_pose[0],gate_pose[1]);  

    //set bias points with the victim list centroids
    for(std::pair<int,Polygon> victim : victim_list){
      victim_centroid = get_polygon_centroid(victim.second);
      bias_points.push_back(victim_centroid);      
    }    
    //Save copy of only victim centroids for computing all combinations of victims
    victim_centroid_list = bias_points;
    //Add start and end point to bias points     
    bias_points.insert(bias_points.begin(),start);
    bias_points.push_back(goal);

    //create PRM instance
    PRM PRM_obj(PRM_param.obstacle_list, PRM_param.map_w, PRM_param.map_h, PRM_param.n_samples, PRM_param.scale);
        
    //Build the roadmap
    PRM_obj.build_roadmap(bias_points, PRM_param.max_dist, PRM_param.min_dist);

    //Sort the victim list in terms of proximity to the start point
    for(Point victim_location:victim_centroid_list){        
        L_sv = 0; //Reset length
        L_vg = 0; //Reset length
        L = 0;
        //Call global planner from start point to victim
        PRM_obj.global_planner(start,victim_location);
        global_planner_path = PRM_obj.get_global_planner_path();
        start_to_victim_path = PRM_obj.refine_global_planner_path(global_planner_path);
        //Calculate distance for each victim
        for(int i=0; i<start_to_victim_path.size() - 1; i++){
            start_line_point = start_to_victim_path[i]; //start point of line
            end_line_point = start_to_victim_path[i + 1]; //end point of line

            //add up distance between points
            L_sv += dist_2points(start_line_point, end_line_point); 
                             
        }
        //Call global planner from victim to gate
        PRM_obj.global_planner(victim_location,goal);
        global_planner_path = PRM_obj.get_global_planner_path();
        start_to_victim_path = PRM_obj.refine_global_planner_path(global_planner_path);
        //Calculate distance for each victim
        for(int i=0; i<start_to_victim_path.size() - 1; i++){
            start_line_point = start_to_victim_path[i]; //start point of line
            end_line_point = start_to_victim_path[i + 1]; //end point of line

            //add up distance between points
            L_vg += dist_2points(start_line_point, end_line_point); 
                             
        }
        //Sum up both lengths
        L = L_sv - L_vg;        
        //save length and victim index pair into vector
        v_length_index_pair.push_back(std::make_pair(L, victim_index));
        victim_index++; 
    }
    //sort by distance
    tmp_victim_centroid_list = victim_centroid_list; //make tmp copy
    sort(v_length_index_pair.begin(), v_length_index_pair.end());

    //save definitive order into victim centroid list
    for(int i=0;i<v_length_index_pair.size();i++){
        victim_centroid_list[i] = tmp_victim_centroid_list[v_length_index_pair[i].second];
    }  


    //Compute all possible combinations
    compute_all_combinations(v_comb, start, goal, victim_centroid_list);  
    
    
    for(int i=0; i<v_comb.size();i++){
        //For each combination
        bias_points = v_comb[i];

        //reset prm path_length
        PRM_obj.path_length = 9999;

        //call prm planner
        path = PRM_obj.prm_planner(start_pose, gate_pose, bias_points, dubins_param, delta);

        //Calculate cost of path
        path_cost = PRM_obj.path_length/robot_speed - victim_reward*(bias_points.size()-2); //- 2 :discount start and end

        //empty path checker
        if(!path.empty()){
            //Save cost and path
            all_cost_path.push_back(std::make_pair(path_cost, path));

            //Save path segments for drawing purposes
            all_cost_pathdraw.push_back(std::make_pair(path_cost, PRM_obj.path_final_draw));
        }
    }
    
    //empty path checker
    if(all_cost_path.size() != 0){
        //Check the optimal path
        for(int i=0; i< all_cost_path.size(); i++) {            
            if(all_cost_path[i].first < lowest_cost){
                opt_index = i;
                lowest_cost = all_cost_path[i].first;
            }        
        }    

        //Save optimal path
        path = all_cost_path[opt_index].second; 

        //Save optimal path segments for drawing purposes
        opt_cost_pathdraw = all_cost_pathdraw[opt_index];   

        //Retrieve PRM variables for drawing purposes
        free_space_points = PRM_obj.get_free_space_points();
        prm_graph = PRM_obj.get_prm_graph();    
    }

    //save output
    mission_2.path = path;   
    mission_2.all_cost_pathdraw = all_cost_pathdraw;
    mission_2.opt_cost_pathdraw = opt_cost_pathdraw;
    mission_2.free_space_points = free_space_points;
    mission_2.prm_graph = prm_graph; 

    return mission_2; 
}


void compute_all_combinations(std::vector<std::vector<Point>>& v_comb, 
        Point start, Point end, std::vector<Point> victim_centroid_list){
    
    //variables
    std::vector<int> v_index_permuted;
    std::vector<Point> comb;
    int N; //number of indexes

    //Set vector of indexes
    for(int j=0; j<victim_centroid_list.size() ;j++){
        v_index_permuted.push_back(j);
    }

    //permutations
    do {    
        N = v_index_permuted.size();     
        while(N > 0){
            comb.push_back(start); //Add start
            for(int i=0; i<N; i++){
                //add victim centroid for each permutation
                comb.push_back(victim_centroid_list[i]); 
            }
            comb.push_back(end); //add end
            v_comb.push_back(comb); // Add final combination to the vector of combinations
            N--; //remove last victim
            comb.clear(); //empty vector
        }        
    } while (std::next_permutation(v_index_permuted.begin(), v_index_permuted.end()));    
    v_comb.push_back({start,end}); //Last combination is from start to end
}


