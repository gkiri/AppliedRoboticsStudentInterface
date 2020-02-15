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
    double gate_pose[3], std::vector<std::pair<int,Polygon>> victim_list, double delta){
    
    Path path;
    struct mission_output_1 mission_1;    
    Point start, goal;    
    std::vector<std::pair<int,int>> v_id_index_pair;
    std::vector<int> v_victim_id;
    std::vector<Point> v_victim_centroid;
    Point victim_centroid;
    int victim_index;   
    std::vector<Point> bias_points;   
  
   
    //Set start and goal point
    start = Point(start_pose[0],start_pose[1]);
    goal = Point(gate_pose[0],gate_pose[1]);   

    //set bias points with victims in the order given by its number     
    //sort by number 
    victim_index = 0;
    for(std::pair<int,Polygon> victim : victim_list){        
        //Location of victim
        victim_centroid = get_polygon_centroid(victim.second);
        v_victim_centroid.push_back(victim_centroid);
        //Save pair of victim id and index
        v_id_index_pair.push_back(std::make_pair(victim.first, victim_index));
        //increase index
        victim_index++;
    }
    //sort by id
    sort(v_id_index_pair.begin(), v_id_index_pair.end());
    //Add start
    bias_points.push_back(start);
    //Add sorted victim locations (centroids)
    for(std::pair<int, int> id_index_pair: v_id_index_pair){
        bias_points.push_back(v_victim_centroid[id_index_pair.second]);
    }
    //Add end
    bias_points.push_back(goal);

    //create PRM instance
    PRM PRM_obj(PRM_param.obstacle_list, PRM_param.map_w, PRM_param.map_h, PRM_param.n_samples, PRM_param.scale);
        
    //Build the roadmap
    PRM_obj.build_roadmap(bias_points, PRM_param.max_dist, PRM_param.min_dist);

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
    Point start, goal, start_gp, goal_gp, start_line_point, end_line_point;
    Point victim_centroid; 
    std::vector<Point> victim_centroid_list, tmp_victim_centroid_list;
    std::vector<std::vector<Point>> v_comb;
    std::vector<std::pair<double, Path>> all_cost_path;
    std::vector<std::pair<double, std::vector<arc_extract>>> all_cost_pathdraw;
    std::pair<double, std::vector<arc_extract>> opt_cost_pathdraw;
    std::vector<Point> bias_points, refined_gp_path, global_planner_path, tmp_global_planner_path, free_space_points;
    std::vector<std::pair<Point, std::vector<Point> >> prm_graph;
    double path_cost;
    int opt_index; 
    int victim_index = 0;
    double lowest_cost = 9999;    
    std::vector<std::pair<double, int>> v_length_index_pair;
    double L_sv, L_vg, L_gp;
    double L=0;
    int comb_index, comb_counter;
    std::vector<std::vector<Point>> v_gppath;
    std::vector<std::pair<double, int>> v_cost; //
    //std::vector<std::pair<double, std::vector<Point>>> v_cost_gppath;


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

    //Sort the victim list in terms of proximity to the start point and gate remoteness
    for(Point victim_location:victim_centroid_list){        
        L_sv = 0; //Reset distance btw start and victim
        L_vg = 0; //Reset distance btw victim and goal
        L = 0;    // Reset tota cost distance
        //Call global planner from start point to victim
        PRM_obj.global_planner(start,victim_location);
        global_planner_path = PRM_obj.get_global_planner_path();
        refined_gp_path = PRM_obj.refine_global_planner_path(global_planner_path);
        //Calculate distance for each victim
        for(int i=0; i<refined_gp_path.size() - 1; i++){
            start_line_point = refined_gp_path[i]; //start point of line
            end_line_point = refined_gp_path[i + 1]; //end point of line
            //add up distance between points
            L_sv += dist_2points(start_line_point, end_line_point);                              
        }
        //Call global planner from victim to gate
        PRM_obj.global_planner(victim_location,goal);
        global_planner_path = PRM_obj.get_global_planner_path();
        refined_gp_path = PRM_obj.refine_global_planner_path(global_planner_path);
        //Calculate distance for each victim
        for(int i=0; i<refined_gp_path.size() - 1; i++){
            start_line_point = refined_gp_path[i]; //start point of line
            end_line_point = refined_gp_path[i + 1]; //end point of line
            //add up distance between points
            L_vg += dist_2points(start_line_point, end_line_point);                              
        }
        //Sum up both lengths
        L = L_sv - L_vg;        
        //save length and victim index pair into vector
        v_length_index_pair.push_back(std::make_pair(L, victim_index));
        //Increase index       
        victim_index++; 
    }
    //sort by distance cost
    tmp_victim_centroid_list = victim_centroid_list; //make tmp copy
    sort(v_length_index_pair.begin(), v_length_index_pair.end());
    //save definitive order into victim centroid list
    for(int i=0;i<v_length_index_pair.size();i++){
        victim_centroid_list[i] = tmp_victim_centroid_list[v_length_index_pair[i].second];
    }  

    //Compute all possible combinations    
    //compute_all_combinations(v_comb, victim_centroid_list); 
    compute_all_combinations(v_comb, start, goal, victim_centroid_list); 
    // std::cout << "comb size: " << v_comb.size() << std::endl;
    // for(std::vector<Point> victim_order:v_comb){
    //     for(Point victim: victim_order){
    //         std::cout << victim.x << "," << victim.y << "/";
    //     }
    //     std::cout << std::endl;
    // }

    //Calculate cost for all possible paths (global planner aproximation)    
    for(int i=0; i<v_comb.size();i++){
        //Empty bias points
        bias_points.clear();        
        //For each combination
        bias_points = v_comb[i];         

        // std::cout << "Bias points " << i << ":" <<std::endl;
        // for(Point bias:bias_points){
            
        //     std::cout << bias.x << "," << bias.y << "///";
        // }
        // std::cout << std::endl;
        
        
        //Save initial node into the tmp_global_planner_path
        tmp_global_planner_path.clear(); //empty container
        tmp_global_planner_path.push_back(start);

        //Go through each consecutive pair inside bias_points
        for(int z=0; z< bias_points.size() - 1; z++){
            //set start and goal Point
            start_gp = bias_points[z];
            goal_gp = bias_points[z + 1];            

            //Call global planner   
            PRM_obj.global_planner(start_gp,goal_gp);       
            global_planner_path = PRM_obj.get_global_planner_path();            
                        
            //Refine global_planner
            refined_gp_path = PRM_obj.refine_global_planner_path(global_planner_path);
            
            //Save concatenation of refined global planners
            tmp_global_planner_path.insert(tmp_global_planner_path.end(), 
                refined_gp_path.begin() + 1, refined_gp_path.end()); //+1 to not duplicate points (previous end = next start)
        }      

        //mission_2.global_planner_path = tmp_global_planner_path;    

        //Calculate cost for refined path
        //Get global planner length
        L_gp = 0; //reset length container
        for(int j=0; j<tmp_global_planner_path.size() - 1; j++){
            start_line_point = tmp_global_planner_path[j]; //start point of line
            end_line_point = tmp_global_planner_path[j + 1]; //end point of line
            //add up distance between points
            L_gp += dist_2points(start_line_point, end_line_point);                              
        }
        //Calculate cost
        path_cost = L_gp/robot_speed - victim_reward*(bias_points.size()-2); //- 2 :discount start and end

        //Save cost 
        v_cost.push_back(std::make_pair(path_cost, i));
        //Save global planner
        v_gppath.push_back(tmp_global_planner_path);
        //v_cost_gppath.push_back(std::make_pair(path_cost, tmp_global_planner_path));       
    }

    //std::cout << "global planner vector size: " << v_gppath.size() << std::endl; 
        
    //Sort vector by cost
    //sort(v_cost_gppath.begin(), v_cost_gppath.end());
    sort(v_cost.begin(), v_cost.end());
    
    // std::cout << "I sorted the thing, shorthest is " << v_cost[0].second 
    //     << " with cost " << v_cost[0].first;

    //Retrieve global planner combinations by ascending order (depending on cost)
    comb_counter = 0;
    while( path.empty() && comb_counter < v_gppath.size()){ //empty path checker
        //Get lowest cost combination index
        comb_index = v_cost[comb_counter].second;
        //std::cout << comb_counter << ": " << comb_index << std::endl;
        //Set global planner
        PRM_obj.set_global_planner_path(v_gppath[comb_index]);
        //Run dubins planner
        path = PRM_obj.dubins_planner(start_pose[2], gate_pose[2], dubins_param, delta);
        //Increase counter in case path is empty
        comb_counter++;
    }            

    //Retrieve PRM variables for drawing purposes
    free_space_points = PRM_obj.get_free_space_points();
    prm_graph = PRM_obj.get_prm_graph();
    opt_cost_pathdraw.first = v_cost[comb_index].first ; //opt cost
    opt_cost_pathdraw.second = PRM_obj.path_final_draw; //opt path drawing

    //Save output
    mission_2.path = path;   
    //mission_2.all_cost_pathdraw = all_cost_pathdraw;
    mission_2.opt_cost_pathdraw = opt_cost_pathdraw;
    mission_2.free_space_points = free_space_points;
    mission_2.prm_graph = prm_graph; 
    mission_2.global_planner_path = v_gppath[comb_index];
    

    return mission_2;
    
}

// void compute_all_combinations(std::vector<std::vector<Point>>& v_comb, 
//         Point start, Point end, std::vector<Point> victim_centroid_list){
    
//     //variables
//     std::vector<int> v_index_permuted;
//     std::vector<Point> comb;
//     int N; //number of indexes

//     //Set vector of indexes
//     for(int j=0; j<victim_centroid_list.size() ;j++){
//         v_index_permuted.push_back(j);
//     }

//     //permutations
//     do {    
//         N = v_index_permuted.size();     
//         while(N > 0){
//             comb.push_back(start); //Add start
//             for(int i=0; i<N; i++){
//                 //add victim centroid for each permutation
//                 comb.push_back(victim_centroid_list[i]); 
//             }
//             comb.push_back(end); //add end
//             v_comb.push_back(comb); // Add final combination to the vector of combinations
//             N--; //remove last victim
//             comb.clear(); //empty vector
//         }        
//     } while (std::next_permutation(v_index_permuted.begin(), v_index_permuted.end()));    
//     v_comb.push_back({start,end}); //Last combination is from start to end
// }

void compute_all_combinations(std::vector<std::vector<Point>>& v_comb, 
        Point start, Point end, std::vector<Point> victim_centroid_list){
    
    //variables
    std::vector<int> v_index_permuted;  
    std::vector<Point> comb; 
    int n_victims;

    //Number of victims
    n_victims = victim_centroid_list.size();

    //Set vector of indexes
    for(int j=0; j < n_victims ;j++){
        v_index_permuted.push_back(j);
    }

    //permutations
    do { 
        for(int n = n_victims; n > 0; n--){        
            comb.push_back(start); //Add start            
            for(int i=0; i < n; i++){
                //add victim centroid for each permutation            
                comb.push_back(victim_centroid_list[i]); 
            }        
            comb.push_back(end); //add end
            v_comb.push_back(comb); 
            
            comb.clear(); //empty container
        }
    } while (std::next_permutation(v_index_permuted.begin(), v_index_permuted.end()));    
    v_comb.push_back({start,end}); //Last combination is from start to end
}


