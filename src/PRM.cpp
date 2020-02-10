

#include <cstdlib>
#include <iostream>
#include <cfloat>
#include <algorithm> 

#include "PRM.h"
//#include "Utils.hpp"
#include "a_star.cpp"
#include "collision_detection.hpp"


PRM::PRM(std::vector<Polygon> polygons_list)
{
    //polygons
    for (size_t j = 0; j<polygons_list.size(); j++){  
        obstacle_list.push_back(polygons_list[j]);
    }
}


PRM::PRM(std::vector<Polygon> polygons_list, double cspace_width, double cspace_height, int N)
{
    //polygons
    for (size_t j = 0; j<polygons_list.size(); j++){  
        obstacle_list.push_back(polygons_list[j]);
    }
    //rest
    this->cspace_width = cspace_width;
    this->cspace_height = cspace_height;
    this->N = N;
}


PRM::~PRM()
{

}


/* Draw Point in Polygon Test -------------------------------------------*/
bool PRM::point_liesin_polygon(Point pt,std::vector<Polygon> cv_poly_list)
{
    std::vector<cv::Point> contour;
    Polygon obstacle;
    cv::Point cv_point_temp;

    cv::Point2f test_pt;
	  test_pt.x = pt.x*100*720/156;
	  test_pt.y = pt.y*100*576/106;

    for (size_t j = 0; j<cv_poly_list.size(); j++){  

        obstacle=cv_poly_list[j];
        contour.clear();//vector clearing for next test

        for (size_t k = 0; k<obstacle.size(); k++){  

            cv_point_temp.x=obstacle[k].x*100*720/150;//img_map_w*x_ob/map_w
            cv_point_temp.y=obstacle[k].y*100*576/100;
            contour.push_back(cv_point_temp);

        }//inner for loop

        int result=cv::pointPolygonTest(contour, test_pt, false);
        double dist=cv::pointPolygonTest(contour, test_pt, true);

        switch(result)
        {
            case 1:  //1 = (point lies inside polygon)

                //std::cout << "Gkiri:case 111111111 Inside polygon result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y<<  std::endl;
                return true;
                break;
            case 0:  //0 (point lies on edge of polygon)

                //std::cout << "Gkiri:case 00000000 Inside polygon result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y<<  std::endl;
                return true;
                break;
            case -1:   //-1 (point lies outside polygon)

                //std::cout << "Gkiri:case -1-1-1-1-1-1-1 Inside polygon result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y<<  std::endl;
                if((dist< 0.0 && dist > -10.0))
                {
                    return true;//Point outside polygon but bit closer to edge so remove this with some threshold distance   
                }
                break;
            default:
                //std::cout << "ERROR from function" << std::endl ;
                break;
        } 

       // std::cout << "Gkiri:point_polygon Outside polygon -----------------result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y <<  std::endl;
        
    }//outer loop

    return false;//false = point doesnot lie in polygon

}


void PRM::generate_random_points()
{

    //std::cout <<"Random Sampling in Map start st" <<  std::endl;
    Point test_pt;
    int count=0;  

    /* Generate random pointst */
    for(int i=0;i<N;i++){
        float x_rand = (rand() / (double) RAND_MAX) * cspace_width; //Generate random sample
        float y_rand = (rand() / (double) RAND_MAX) * cspace_height;

        //std::cout <<"Random Sampling in Map" << x_rand << "," << y_rand << std::endl;

        test_pt.x=x_rand;
        test_pt.y=y_rand;

        if(!point_liesin_polygon(test_pt ,  obstacle_list)){ //Global obstacle list
            //draw_point(random_points[z], *map_param); 
            free_space_points.emplace_back(test_pt);
            count++;
            //td::cout << "Gkiri :: Motion sample After count= " << count << std::endl; 
        }

    }//end for loop

}

void PRM::global_planner(Point start,Point goal){
    //input --> std::pair<Point, std::pair<Point ,Point > > prm_graph;
    //output --> std::vector<Point> global_planner_path;
    
    //Call astar  
    global_planner_path = globalplanner::astar(prm_graph, start, goal);
}


void PRM::local_planner(std::vector<Point> bias_points, double max_dist, double min_dist){    
    //****************************************************************************
    //**Example of creating a simple graph of a single vertex and 3 edges*********

    // //Example variables    
    // std::pair<Point, std::vector<Point> > graph_example_element;
    // Point Vertex_ex_1 = Point(0.1,0.1);
    // std::vector<Point> Edges_ex_1 = {Point(0.4,0.2),Point(0.7,0.5),Point(0.1,0.3)};
    // //Save into graph    
    // graph_example_element = std::make_pair(Vertex_ex_1, Edges_ex_1);    
    // prm_graph.push_back(graph_example_element);
    //****************************************************************************  
      
    //Variables    
    pointVec knn_points;
    bool remove_flag, collision;
    Point Pt;
    std::vector<Point> KNN_points;
    point_t pt_t;
    pointVec pt_t_points, aux_pt_t_points;
    std::pair<Point, std::vector<Point> > graph_element; // (V,E) element of prm_graph
    std::vector<Point> graph_element_edges; // edges of an element of prm_graph
    struct arc_extract edge_line; //line creation for collision detection

    /*Insert bias points into free_space_points to be included into the KDTree*/
    free_space_points.insert(free_space_points.begin(), bias_points.begin(), bias_points.end());

    /*Construct KDTree from sample points*/

    //Convert sample points to point_t    
    for(Point Pt : free_space_points){
        pt_t = Point_to_point_t(Pt);
        pt_t_points.push_back(pt_t);
    }
    //Create tree
    KDTree tree(pt_t_points);

    /*Construct Graph G(V,E) from KDTree and KNN search*/    
    //For each point, find edges    
    for(int i=0;i<pt_t_points.size();i++){   
        //neighborhood points            
        auto knn_point_all = tree.neighborhood_points(pt_t_points[i], max_dist);
        //neighborhood points below minimum distance
        auto knn_point_remove = tree.neighborhood_points(pt_t_points[i], min_dist);
        
        //Remove points within minimum distance 
        knn_points.clear(); //reset vector of candidates 
        for(point_t knn_candidate : knn_point_all){ 
            remove_flag = false; //reset flag   
            for(point_t knn_remove : knn_point_remove){                        
                if(same_point(knn_candidate, knn_remove)){ //points is within the minimum radius
                    remove_flag = true; //remove knn_point
                }
            }
            if(!remove_flag){
                knn_points.push_back(knn_candidate);
            }
        }        

        //Convert back to Points & Check for collision  
        edge_line.start_point = free_space_points[i]; //vertex as start point
        KNN_points.clear(); //reset vector of Point format                 
        for(point_t pt_t : knn_points){
            Pt = point_t_to_Point(pt_t);             
            //Check for collision 
            edge_line.end_point = Pt;
            //collision = Process_Box_line_check_obstacles(obstacle_list,edge_line);
            collision = Global_Line_check(obstacle_list,edge_line);
            if(!collision){ // if no collision, save edge
                KNN_points.push_back(Pt); 
            }                          
        }    
       
        // if Edges size = 0, increase radius and repeat process util edge.size() != 0
        // TBD 

        //Save into graph
        graph_element = std::make_pair(free_space_points[i], KNN_points);    
        prm_graph.push_back(graph_element);
    }  
}


Path PRM::dubins_planner(float start_theta, float goal_theta, struct dubins_param dubins_param, double delta = 15){
    //Inputs: global_planner_path, prm_graph
    //Outputs: vector of dubin's segments --> final_path  
    double DEG2RAD = M_PI/180.0; 
    double RAD2DEG = 180.0/M_PI;
    int node_pos = 0;
    bool repath, collision; 
    int N = 0;     
    int maxIter = 360/delta - 1; // number of maximum iterations to repath
    bool tuned_up; //flag to determine if we have to sum or substract the tune angle for this iteration
    //For output
    //std::vector<Pose> poses_final;
    Path path, no_path;
    //For dubins
    //double discretizer_size = 0.005;
    struct arc_extract three_seg[3]; 
    DubinsCurvesHandler dubins_handler(dubins_param.k_max, dubins_param.discretizer_size);       
    DubinsCurve dubins_path;
    double qs[3],qm[3],qe[3]; 
    double heading_angle_sum, heading_angle_sub; //trackers of the heading angles after summing and substracting
    
    while(!globalplanner::ifeq_point(global_planner_path[node_pos], global_planner_path.back())){ //while goal is not reached
        repath = true; //reset flag for collision detected
        N = 0; //reset repath counter    
        //Set values for dubins
        //std::cout << "Node number " << node_pos << ":" << std::endl;
        //start point
        qs[0] = global_planner_path[node_pos].x;
        qs[1] = global_planner_path[node_pos].y;
        //qs[2] = M_PI/2;
        //mid point
        qm[0] = global_planner_path[node_pos + 1].x;
        qm[1] = global_planner_path[node_pos + 1].y;
        //qm[2]= -M_PI/2;
        //std::cout << "qs,qm angle: " << qs[2] << "," << qm[2] << std::endl;

        if(node_pos == 0){ //set start angle
            qs[2] = start_theta; 
        }
        if(node_pos + 1 == global_planner_path.size() - 1){ //mid node is goal
            qm[2] = goal_theta; 
        }
        else{ 
            //set end point                
            qe[0] = global_planner_path[node_pos + 2].x;
            qe[1] = global_planner_path[node_pos + 2].y;
            //by default goal theta, it is not used at any point
            qe[2] = goal_theta; 
            //Calculate best heading angle for the mid point
            //compute_heading_angle(qs,qm,qe);
            qm[2] = atan2(qe[1] - qs[1], qe[0] - qs[0]); //atan2(y,x)
        }            
        // std::cout << "---> QS: (" << qs[0] << "," << qs[1] << "," << qs[2]*RAD2DEG << "), QM: ("
        //                 << qm[0] << "," << qm[1] << "," << qm[2]*RAD2DEG << "), QE: ("
        //                 << qe[0] << "," << qe[1] << "," << qe[2]*RAD2DEG << ")" << std::endl;
        
        //Save original heading angle of mid point in case of collision
        heading_angle_sum = qm[2];
        heading_angle_sub = qm[2];
        //std::cout << "HA original:" << heading_angle_sum*RAD2DEG << std::endl;
        while(repath && N < maxIter){
            //calculate dubin's segments between actual node and the following
            //dubins_wrapper_api(path_segment, three_seg, qs, qm, rho);
            dubins_path = dubins_handler.findShortestPath(qs[0],qs[1],qs[2],qm[0],qm[1],qm[2]);

            //Retrieve three_seg (dubins path representation)
            create_three_seg(three_seg, qs[0], qs[1], dubins_path);

            //Check for collision            
            repath = Highlevel_Box_dubins_check(obstacle_list, three_seg);
            std::cout << repath << std::endl;        

            //No collision
            if(!repath){
                std::cout << "---> QS: (" << qs[0] << "," << qs[1] << "," << qs[2]*RAD2DEG << "), QM: ("
                        << qm[0] << "," << qm[1] << "," << qm[2]*RAD2DEG << "), QE: ("
                        << qe[0] << "," << qe[1] << "," << qe[2]*RAD2DEG << ")" << std::endl;
        
                //Push segments for drawing purposes
                for(int i=0; i<3; i++){
                    path_final_draw.push_back(three_seg[i]);
                }
                //Insert new dubins path into path
                concatenate_dubins_path(path,dubins_path, dubins_param.discretizer_size);                                                  

                //Set up for next node            
                node_pos++; //next node
                qs[2] = qm[2]; //next start node angle is previous mid point angle 
            }   
            //Collision
            else{ 
                //tune heading angle of qm
                if(!tuned_up){                 
                    heading_angle_sum += delta*DEG2RAD;   //sum delta                    
                    qm[2] = heading_angle_sum; //set mid heading angle 
                    tuned_up = true; //set flag for next iteration
                    std::cout << "iteration " << N << ": New HA: " << heading_angle_sum*RAD2DEG << std::endl;
                }
                else{                    
                    heading_angle_sub -= delta*DEG2RAD;   //sub delta                    
                    qm[2] = heading_angle_sub; //set mid heading angle 
                    tuned_up = false; //set flag for next iteration
                    std::cout << "iteration " << N << ": New HA: " << heading_angle_sub*RAD2DEG << std::endl;
                }
            } 
            //increase repathing counter
            N++;    
            
        }
        if(N == maxIter){
            printf("It does NOT exist a non-colliding path with this roadmap\n");          
            return no_path; //return empty path
        }
    }
   
    return path;
}

Path PRM::prm_planner(double* start_pose, double* goal_pose, std::vector<Point> bias_points, 
                        double max_dist, double min_dist, struct dubins_param dubins_param, double delta){
    //Variables
    Path path;    
    std::vector<Point> tmp_global_planner_path, refined_path;   

    //Set variables
    Point start = Point(start_pose[0], start_pose[1]);
    Point goal = Point(goal_pose[0], goal_pose[1]);
    float start_theta = start_pose[2];
    float goal_theta = goal_pose[2];

    //Sample generation
    PRM::generate_random_points();   
    
    //Add start and end bias points    
    bias_points.insert(bias_points.begin(), start);
    bias_points.push_back(goal);    
    
    //Call local planner 
    PRM::local_planner(bias_points, max_dist, min_dist);

    //Save initial node into the tmp_global_planner_path
    tmp_global_planner_path.push_back(start);

    //Go through each consecutive pair inside bias_points
    for(int i=0; i< bias_points.size() - 1; i++){
        //set start and goal Point
        start = bias_points[i];
        goal = bias_points[i + 1];

        //debug
        std::cout << "pair of bias points " << i << ": " << start.x << "," << start.y 
            << " & " << goal.x << "," << goal.y << std::endl;

        //Call global planner   
        PRM::global_planner(start,goal);

        //Refine tmp_global_planner
        refined_path = refine_global_planner_path(global_planner_path);

        //Save concatenation of refined global planners
        tmp_global_planner_path.insert(tmp_global_planner_path.end(), 
            refined_path.begin() + 1, refined_path.end()); //+1 to not duplicate points (previous end = next start)
    } 

    // //Refine tmp_global_planner
    // global_planner_path = refine_global_planner_path(tmp_global_planner_path);

    //Save concatenate global planner into PRM variable
    global_planner_path = tmp_global_planner_path;

    //Call dubins planner
    path = PRM::dubins_planner(start_theta, goal_theta, dubins_param, delta);

    //return
    return path;
}

std::vector<Point> PRM::refine_global_planner_path(std::vector<Point> tmp_global_planner_path){
    
    std::vector<Point> refined_path;
    bool collision;
    struct arc_extract line_path;
    Point start_local;
    int next_node_counter = -1;    
    Point start_global = tmp_global_planner_path[0];
    Point end_local = tmp_global_planner_path.back();

    //Insert end_global
    refined_path.push_back(end_local);

    while(!globalplanner::ifeq_point(end_local, start_global)){ 
        collision = true; //reset flag
        while(collision){
            next_node_counter++;
            start_local = tmp_global_planner_path[next_node_counter];
            //set line connection btw end_local and start_local
            line_path.start_point = end_local;
            line_path.end_point = start_local;
            //Check for collision
            collision = Global_Line_check(obstacle_list,line_path);
        }
        //Found a non-colliding connection
        end_local = start_local;
        next_node_counter = -1; //reset counter
        //save node into refined path
        refined_path.insert(refined_path.begin(), end_local);
    }

    return refined_path;
}


/*Returns for unit tests --------------------------------------*/
std::vector<Point> PRM::get_free_space_points()
{
    return free_space_points;
}

std::vector<Point> PRM::get_global_planner_path(){
    return global_planner_path;
}

std::vector<std::pair<Point, std::vector<Point> >> PRM::get_prm_graph(){
    return prm_graph;
}

/*Set for unit tests---------------------------------------------*/
void PRM::set_prm_graph(std::vector<std::pair<Point, std::vector<Point> >> prm_graph_test){
    prm_graph = prm_graph_test;
}

void PRM::set_free_space_points(std::vector<Point> free_space_points_test){
    free_space_points = free_space_points_test;
}





