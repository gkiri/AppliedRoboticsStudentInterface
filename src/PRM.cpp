

#include <cstdlib>
#include <iostream>
#include <cfloat>
#include <algorithm> 

#include "PRM.h"
#include "Utils.hpp"
#include "a_star.cpp"


PRM::PRM(std::vector<Polygon> polygons_list)
{
  for (size_t j = 0; j<polygons_list.size(); j++){  
      obstacle_list.push_back(polygons_list[j]);
  }

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
	  test_pt.x = pt.x*100*720/150;
	  test_pt.y = pt.y*100*576/100;

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


void PRM::generate_random_points(double cspace_length,double cspace_width, int N)
{

    std::cout <<"Random Sampling in Map start st" <<  std::endl;
    Point test_pt;
    int count=0;  

    /* Generate random pointst */
    for(int i=0;i<N;i++){
        float x_rand = (rand() / (double) RAND_MAX) * cspace_length; //Generate random sample
        float y_rand = (rand() / (double) RAND_MAX) * cspace_width;

        std::cout <<"Random Sampling in Map" << x_rand << "," << y_rand << std::endl;

        test_pt.x=x_rand;
        test_pt.y=y_rand;

        if(!point_liesin_polygon(test_pt ,  obstacle_list)){ //Global obstacle list
            //draw_point(random_points[z], *map_param); 
            free_space_points.emplace_back(test_pt);
            count++;
            std::cout << "Gkiri :: Motion sample After count= " << count << std::endl; 
        }

    }//end for loop

}

void PRM::global_planner(Point start,Point goal){
    //input --> std::pair<Point, std::pair<Point ,Point > > prm_graph;
    //output --> std::vector<Point> global_planner_path;

    //Call astar
    std::cout << "previous" << prm_graph.size() << std::endl;
    global_planner_path = globalplanner::astar(prm_graph, start, goal);
}


void PRM::local_planner(){    
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
    //implement local planner here    
    //Variables
    //double max_dist = 0.31; //max distance
    double min_dist = 0.2001;
    //double max_dist = 0.11;
    double max_dist = 0.3001;
    pointVec knn_points;
    bool remove_flag;
    Point Pt;
    std::vector<Point> KNN_points;
    point_t pt_t;
    pointVec pt_t_points, aux_pt_t_points;
    std::pair<Point, std::vector<Point> > graph_element; // (V,E) element of prm_graph
    std::vector<Point> graph_element_edges; // edges of an element of prm_graph

    /*Insert bias points into free_space_points to be included into the KDTree*/
    //free_space_points.insert(free_space_points.begin(),bias_points.begin(), bias_points(end));

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


        // std::cout << "---------------------------" << std::endl;
        // std::cout << "Point " << i << ":" << pt_t_points[i][0] << "," <<pt_t_points[i][1] << std::endl;
        
        // if(knn_points.size() == 1){ //no points within the neighborhood (only itself)
        //     //Construct auxiliar tree            
        //     aux_pt_t_points = pt_t_points;            
        //     aux_pt_t_points.erase(aux_pt_t_points.begin() + i); //Eliminate the point to find the nearest            
            
        //     KDTree aux_tree(aux_pt_t_points);            
        //     auto nearest_point = aux_tree.nearest_point(pt_t_points[i]); //nearest point 
        //     //Pt = point_t_to_Point(knn_points);
        //     //std::cout << "nearest point:" << Pt.x << ", " << Pt.y << std::endl;
        //     //CONVERT NEAREST IN A VECTOR TO REDUCE CODE
        //     knn_points.clear();
        //     knn_points.push_back(nearest_point);
        //     // Pt = point_t_to_Point(nearest_point);
        //     // KNN_points.push_back(Pt);
        //     // std::cout << "edges:" << Pt.x << ", " << Pt.y << std::endl;         
        // }
                        
        // //Convert back to Points                     
        // for(point_t pt_t : knn_points){
        //     Pt = point_t_to_Point(pt_t);
        //     if(!same_point(Pt, free_space_points[i])){ //do not safe the vertex as an edge
        //         KNN_points.push_back(Pt); 
        //         std::cout << "edges:" << Pt.x << ", " << Pt.y << std::endl; 
        //     }               
        // }

        //Convert back to Points   
        KNN_points.clear(); //reset vector of Point format                 
        for(point_t pt_t : knn_points){
            Pt = point_t_to_Point(pt_t);            
            KNN_points.push_back(Pt); 
            //std::cout << "edges:" << Pt.x << ", " << Pt.y << std::endl;               
        }
        

        //Construct graph for each point being the edges the knn_points
        //Check collision for edges
        //TBD
        //When a collision happen, point is removed from edge
        // if Edges size = 0, increase radius and repeat process util edge.size() != 0
        //

        //Save into graph
        graph_element = std::make_pair(free_space_points[i], KNN_points);    
        prm_graph.push_back(graph_element);
    }           
    std::cout << prm_graph.size() << std::endl;
}


void PRM::dubins_planner(Path& final_path){
    //Inputs: global_planner_path, prm_graph
    //Outputs: vector of dubin's segments --> final_path
    int node_pos = 0;
    bool repath; 
    int N = 0;   
    int maxIter = 50; // number of maximum iterations to repath

    //For dubins
    Path path;
    struct arc_extract three_seg[3];
    double rho=0.05;    //@Alvaro: DANGER!!! DUPLICATED VALUE IN student_interface.cpp
    double qs[3],qm[3],qe[3]; 
    // Point goal = global_planner_path.back();
    // std::cout << "GOAL point:" << goal.x << ", " << goal.y << std::endl;
    while(!globalplanner::ifeq_point(global_planner_path[node_pos], global_planner_path.back())){ //while goal is not reached
        repath = true; //flag for collision detected
        while(repath && N < maxIter){

            //Set values for dubins
            std::cout << "Node number " << node_pos << ":" << std::endl;
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
                qs[2] = 0; // @Alvaro: SUBSTITUTE FOR STARTING ANGLE (retrieve in find_robot)
            }
            if(node_pos + 1 == global_planner_path.size() - 1){ //mid node is goal
                qm[2] = 0; //@Alvaro: SUBSTITUTE FOR ENDING ANGLE (not sure how we calculate this)
            }
            else{ 
                //set end point                
                qe[0] = global_planner_path[node_pos + 2].x;
                qe[1] = global_planner_path[node_pos + 2].y;
                //Calculate best heading angle for the mid point
                compute_heading_angle(qs,qm,qe);
            }            
            std::cout << "---> QS: (" << qs[0] << "," << qs[1] << "," << qs[2] << "), QM: ("
                            << qm[0] << "," << qm[1] << "," << qm[2] << "), QE: ("
                            << qe[0] << "," << qe[1] << "," << qe[2] << ")" << std::endl;
            

            //calculate dubin's segments between actual node and the following
            dubins_wrapper_api(path, three_seg, qs, qm, rho);

            //Check for collision
            for(int i=0; i<3; i++){
                //TBD --> COLLISION CHECKER
                repath = false; //set flag to no collision                
            }

            //No collision
            if(!repath){
                //Push segments for drawing purposes
                for(int i=0; i<3; i++){
                    final_path_draw.push_back(three_seg[i]);
                }

                //Push to final_path
                // TBD  

                //Set up for next node            
                node_pos++; //next node
                qs[2] = qm[2]; //next start node angle is previous mid point angle 
            }            
        }
    }

}


/*Returns for unit tests --------------------------------------*/
std::vector<Point> PRM::get_free_space_points()
{
    return free_space_points;
}

std::vector<Point> PRM::get_global_planner_path(){
    return global_planner_path;
}

//@Ambike
std::vector<std::pair<Point, std::vector<Point> >> PRM::get_prm_graph(){
    return prm_graph;
}

/*Set for unit tests---------------------------------------------*/
void PRM::set_prm_graph(std::vector<std::pair<Point, std::vector<Point> >> prm_graph_test){
    prm_graph = prm_graph_test;
}

//@Ambike
void PRM::set_free_space_points(std::vector<Point> free_space_points_test){
    free_space_points = free_space_points_test;
}





