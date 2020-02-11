// #include "student_image_elab_interface.hpp"
// #include "student_planning_interface.hpp"

// #include <vector>

// #include "Utils.hpp"
// #include "PRM.h"

// /************Input variables generation for Unit Testing**********************/
// Point add_points(Point pt1,Point pt2){
//   Point result;
//   result.x = pt1.x + pt2.x;
//   result.y = pt1.y + pt2.y;

//   return result;
// }

// Point sub_points(Point pt1,Point pt2){
//   Point result;
//   result.x = pt1.x - pt2.x;
//   result.y = pt1.y - pt2.y;

//   return result;
// }

// /*Generate graph example*/
// std::vector<std::pair<Point, std::vector<Point> >> generate_graph_test(){
//   std::vector<std::pair<Point, std::vector<Point> >> prm_graph_test;
//   std::pair<Point, std::vector<Point>> adj_list;
//   std::vector<Point> edges_list;
//   Point next_edge;
//   //std::pair<Point,Point > next_point;
//   Point start_point = Point(0.1,0.1);
//   int rows = 9; 
//   int cols = 14; 
//   Point x_step = Point(0.1,0);
//   Point y_step = Point(0,0.1);
  
//   //Initilize first point
//   Point actual_point;
//   for(int r=0;r<rows;r++){  //loop through rows
//     actual_point.y = start_point.y + r*y_step.y; //go up 1 row     
//     for(int c=0;c<cols;c++){
//       edges_list.clear(); //reset edges vector
//       // Next point
//       actual_point.x = start_point.x + c*x_step.x; //go right 1 col     
//       //std::cout << "Actual point: " << actual_point.x << ", " << actual_point.y << std::endl;
//       //Check left point
//       if(actual_point.x - x_step.x >= start_point.x){      
//         next_edge = sub_points(actual_point,x_step);
//         edges_list.push_back(next_edge);
//         //std::cout << "------------ left: " << next_edge.x << ", " << next_edge.y << std::endl;        
//       }
//       //Check up point
//       if(actual_point.y + y_step.y <= start_point.y + y_step.y*(rows - 1)){
//         next_edge = add_points(actual_point,y_step);
//         edges_list.push_back(next_edge);
//         //std::cout << "------------ up: " << next_edge.x << ", " << next_edge.y << std::endl;  
//       }
//       //Check right point
//       if(actual_point.x + x_step.x <= start_point.x + x_step.x*(cols - 1)){
//         next_edge = add_points(actual_point,x_step);
//         edges_list.push_back(next_edge);
//         //std::cout << "------------ right: " << next_edge.x << ", " << next_edge.y << std::endl;  
//       }
//       //Check down point
//       if(actual_point.y - y_step.y >= start_point.y){
//         next_edge = sub_points(actual_point,y_step);
//         edges_list.push_back(next_edge);   
//         //std::cout << "------------ down: " << next_edge.x << ", " << next_edge.y << std::endl;       
//       }
//     //Add all vertexes
//       adj_list = std::make_pair(actual_point,edges_list);
//       // std::cout << "prm_graph element " << c + r*cols << ": " << std::endl;
//       // Point V = adj_list.first;
//       // std::cout << "V: " <<  V.x << ", " << V.y << "; E: ";
//       // std::vector<Point> E_list = adj_list.second;
//       // for (int i=0;i<E_list.size();i++){
//       //    std::cout << "(" <<  E_list[i].x << ", " << E_list[i].y << "), ";
//       // }      
//       // std::cout << std::endl;
//       prm_graph_test.push_back(adj_list);
//     }   
//   }
//   //std::cout << "graph size: " << prm_graph_test.size() << std::endl;

//   return prm_graph_test;  
// }

// // @Ambike
// /*Generate points example (from graph)*/
// std::vector<Point> generate_free_space_points_test (std::vector<std::pair<Point, 
//                                                       std::vector<Point> >> prm_graph_test){
//   std::vector<Point> result;
//   for(int i=0;i<prm_graph_test.size();i++){
//     result.push_back(prm_graph_test[i].first); //first element of pair --> V
//   }

//   return result;
// }



// /* **********************Gkiri PRM space Unit Testing*************************/
// void UT_sample_generation(std::vector<Polygon> obstacle_list, double map_w, 
//           double map_h, int N, img_map_def *map_param)
// {
//     PRM obj(obstacle_list, map_w, map_h, N, map_param->scale);
//     obj.generate_random_points();
//     std::vector<Point> free_space_points = obj.get_free_space_points();
//     std::cout << " $$$$$$$$$$$$$$$$$$$$  no:of point= " << free_space_points.size() <<  std::endl;
//     for (size_t i = 0; i<obstacle_list.size(); i++){
//          draw_polygon(obstacle_list[i], *map_param);
//     }
//     for (int z=0;z<free_space_points.size();z++){
//         draw_point(free_space_points[z], *map_param); 
           
//     }

// }

// // void UT_sampling_motion_plan(std::vector<Polygon> obstacle_list ,img_map_def *map_param){
// //     /*Gkiri  PRM sampling motion planning debugging*/
// //     //draw_motion_planning(obstacle_list,&map_param);
// //     //draw_motion_planning(inflated_obstacle_list,&map_param);

// // }


// void UT_local_planner(std::vector<Polygon> obstacle_list, img_map_def *map_param){
//   // Set variables for your unit test
//   double min_dist = 0.1001;    
//   double max_dist = 0.3001;
//   PRM obj(obstacle_list);
//   std::vector<std::pair<Point, std::vector<Point> >> prm_graph_test; //Output 

//   // Generate free_space_points_test example
//   prm_graph_test = generate_graph_test();
//   std::vector<Point> free_space_points_test = generate_free_space_points_test(prm_graph_test);
//   // Save example points inside private variable of PRM.cpp
//   obj.set_free_space_points(free_space_points_test);

//   // //TEST FOR DRAWING THE SAVED POINTS// --> @AR: tested
//   // std::vector<Point> free_space_points_test_saved;
//   // free_space_points_test_saved = obj.get_free_space_points(); //retrieve
//   // for(Point p:free_space_points_test_saved){
//   //   draw_point(p,*map_param);
//   // }

//   //Call your implementation on PRM.cpp 
//   std::vector<Point> bias_points; 
//   obj.local_planner(bias_points, max_dist, min_dist); //Implement your local planner in PRM.cpp
  
//   //Retrieve the output of your function
//   std::vector<std::pair<Point, std::vector<Point> >> prm_graph = obj.get_prm_graph();


//   //****************************************************************************
//   //********Drawing and printing the result of your local planner*************** 
//   //draw polygons
//   for (size_t i = 0; i<obstacle_list.size(); i++){
//     draw_polygon(obstacle_list[i], *map_param);
//   }
//   //draw graph
//   std::pair<Point, std::vector<Point>> graph_node;
//   std::vector<Point> free_space_points;
//   Point V;
//   std::vector<Point> E; 
//   arc_extract edge_line;
//   Point E_point;
  
//   for(int i=0; i<prm_graph.size(); i++){
//     //std::cout << "prm raph size: " << prm_graph_test.size() << std::endl;
//     graph_node = prm_graph[i];
//     V = graph_node.first; //Vertex
//     //std::cout << "prm V: " << V.x << ", " << V.y << std::endl;
//     E = graph_node.second; //Edges
//     //Draw edges    
//     for(int j=0;j<E.size();j++){ 
//       //std::cout << "Edge: " << E[j].x << ", " << E[j].y << std::endl;
//       edge_line = to_arc_extract_type(V,E[j],true);
//       draw_line(edge_line, *map_param);
//     }
//     //Draw vertex
//     draw_point(V, *map_param, cv::Scalar(255,0,0));
//   }
  
// }


// void UT_global_planner(std::vector<Polygon> obstacle_list, img_map_def *map_param){
//   //Set variables for unit test
//   PRM obj(obstacle_list);
//   Point start = Point(0.1,0.1);
//   Point goal = Point(1.3,0.8);  

//   //Set prm_graph unit test example
//   std::vector<std::pair<Point, std::vector<Point> >> prm_graph_test;  
//   prm_graph_test = generate_graph_test();
//   obj.set_prm_graph(prm_graph_test);  //set prm_graph with the test one
//   prm_graph_test = obj.get_prm_graph(); //retrieve to check it was saved correctly    

//   //****************************************************************************
//   //********Global planner******************************************************    
//   obj.global_planner(start,goal);
//   std::vector<Point> global_planner_path = obj.get_global_planner_path(); 

//   // //****************************************************************************
//   // //********Drawing and printing the results*************************************  
//   //draw graph
//   std::pair<Point, std::vector<Point>> graph_node;
//   Point V;
//   std::vector<Point> E; 
//   arc_extract edge_line;
//   Point E_point;
//   for(int i=0; i<prm_graph_test.size(); i++){
//     //std::cout << "prm raph size: " << prm_graph_test.size() << std::endl;
//     graph_node = prm_graph_test[i];
//     V = graph_node.first; //Vertex
//     std::cout << "prm V: " << V.x << ", " << V.y << std::endl;
//     E = graph_node.second; //Edges
//     //Draw edges    
//     for(int j=0;j<E.size();j++){ 
//       std::cout << "Edge: " << E[j].x << ", " << E[j].y << std::endl;
//       edge_line = to_arc_extract_type(V,E[j],true);
//       draw_line(edge_line, *map_param);
//     }
//     //Draw vertex
//     draw_point(V, *map_param, cv::Scalar(255,0,0)); 
//   }

//   //Draw start and goal
//   draw_point(start, *map_param, cv::Scalar(0,255,0));
//   draw_point(goal, *map_param, cv::Scalar(0,255,0));

//   //Draw global planner path
//   for(int i=0;i<global_planner_path.size();i++){
//     //draw_point(global_planner_path[i], *map_param, cv::Scalar(0,255,0));
//     //Draw path
//     if(i<global_planner_path.size()-1){         
//       edge_line = to_arc_extract_type(global_planner_path[i],global_planner_path[i+1],true);
//       draw_line(edge_line, *map_param, cv::Scalar(0,255,0));    
//     }
//     std::cout << "gpp "<< i << ": " << global_planner_path[i].x << ", " << global_planner_path[i].y << std::endl;
//   }  
// }


// /*----------------------------- Dubins path test --------------------------------*/
// void UT_dubins_path(std::vector<Polygon> obstacle_list, img_map_def *map_param){
//   //Set variables for unit test  
//   PRM obj(obstacle_list);

//   //*****************GLOBAL PLANNER SET UP********************************
//   Point start = Point(0.1,0.1);
//   Point goal = Point(1.3,0.8);
//   //Point goal = Point(0.3,0.1);

//   //TEST VARIABLES
//   std::vector<std::pair<Point, std::vector<Point> >> prm_graph_test;  
//   prm_graph_test = generate_graph_test();
//   obj.set_prm_graph(prm_graph_test);  //set prm_graph with the test one

//   //Global planner
//   obj.global_planner(start,goal);
//   std::vector<Point> global_planner_path = obj.get_global_planner_path(); 

//   //****************************************************************************
//   //********Drawing and printing the global planner path************************
//   arc_extract edge_line;
//   for(int i=0;i<global_planner_path.size();i++){
//     //draw_point(global_planner_path[i], *map_param, cv::Scalar(0,255,0));
//     //Draw path
//     if(i<global_planner_path.size()-1){         
//       edge_line = to_arc_extract_type(global_planner_path[i],global_planner_path[i+1],true);
//       draw_line(edge_line, *map_param, cv::Scalar(0,255,0));    
//     }
//     //std::cout << "gpp "<< i << ": " << global_planner_path[i].x << ", " << global_planner_path[i].y << std::endl;
//   }
//   //*****************GLOBAL PLANNER SET UP********************************



//   //****************************************************************************
//   //********************Dubins path algorithm***********************************
//   struct dubins_param dubins_param;
//   dubins_param.k_max = 10;
//   dubins_param.discretizer_size = 0.005;
//   double delta = 15;
//   obj.dubins_planner(0, M_PI/2, dubins_param, delta);  

//   //****************************************************************************
//   //********Drawing and printing the dubins path************************
//   arc_extract dubins_path_seg;
//   for(int i=0; i<obj.path_final_draw.size(); i++){
//     dubins_path_seg = obj.path_final_draw[i]; //retrieve three_segments
//     //std::cout << "Dubins_path_" << i << std::endl;
  
//     //Draw
//     draw_dubins_segment(dubins_path_seg, *map_param);
//   }
//   //****************************************************************************

//   //****************************************************************************
//   //********Drawing and printing the initial prm graph (only vertexes)************
//   //draw graph
//   std::pair<Point, std::vector<Point>> graph_node;
//   Point V;   
//   for(int i=0; i<prm_graph_test.size(); i++){
//     //std::cout << "prm raph size: " << prm_graph_test.size() << std::endl;
//     graph_node = prm_graph_test[i];
//     V = graph_node.first; //Vertex  
//     //Draw vertex
//     draw_point(V, *map_param, cv::Scalar(255,0,0));  
//   }
// }


// void UT_overall_planner(double* start_pose, double* goal_pose, 
//       std::vector<Polygon> obstacle_list, double map_w, double map_h, int N, img_map_def *map_param){
//   //Set variables for unit test
//   PRM obj(obstacle_list, map_w, map_h, N, map_param->scale);
//   double min_dist = 0.1001;    
//   double max_dist = 0.3001;
//   //Point start = Point(0.1,0.1);
//   //Point goal = Point(1.3,0.8); 
//   Point start = Point(start_pose[0], start_pose[1]);
//   Point goal = Point(goal_pose[0], goal_pose[1]);
//   float start_theta = start_pose[2];
//   float goal_theta = goal_pose[2];

//   //**************************************************************************
//   //******** Sample generation ***********************************************

//   obj.generate_random_points();
//   std::vector<Point> free_space_points = obj.get_free_space_points();

//   //***************************************************************************
//   //******** Local planner ****************************************************
  
//   //Add start and end bias points
//   std::vector<Point> bias_points;
//   bias_points.push_back(start);
//   bias_points.push_back(goal);
  
//   //Call local planner 
//   obj.local_planner(bias_points, max_dist, min_dist);
  
//   //Retrieve the output of your function
//   std::vector<std::pair<Point, std::vector<Point> >> prm_graph = obj.get_prm_graph();
//   /*
//   //****************************************************************************
//   //********Global planner****************************************************** 
  
//   //Call global planner   
//   obj.global_planner(start,goal);

//   //Retrieve output of global planner
//   std::vector<Point> global_planner_path = obj.get_global_planner_path(); 
  
//   //****************************************************************************
//   //********Dubins planner******************************************************
//   //Path final_path; //container for dubins planner path outcome

//   //Call dubins planner
//   struct dubins_param dubins_param;
//   dubins_param.k_max = 10;
//   dubins_param.discretizer_size = 0.005;
//   double delta = 15;
//   obj.dubins_planner(start_theta, goal_theta, dubins_param, delta); 
//   */

//   //****************************************************************************
//   //********Drawing and printing the result ************************************

//   //Drawing variables
//   std::pair<Point, std::vector<Point>> graph_node;  
//   Point V;
//   std::vector<Point> E; 
//   arc_extract edge_line;
//   Point E_point;
//   arc_extract dubins_path_seg;  
  
//   //draw polygons
//   for (size_t i = 0; i<obstacle_list.size(); i++){
//     draw_polygon(obstacle_list[i], *map_param);
//   }
  
//   //Draw prm_graph
//   for(int i=0; i<prm_graph.size(); i++){
//     //std::cout << "prm raph size: " << prm_graph.size() << std::endl;
//     graph_node = prm_graph[i];
//     V = graph_node.first; //Vertex
//     //std::cout << "prm V: " << V.x << ", " << V.y << std::endl;
//     E = graph_node.second; //Edges
//     //Draw edges    
//     for(int j=0;j<E.size();j++){ 
//       //std::cout << "Edge: " << E[j].x << ", " << E[j].y << std::endl;
//       edge_line = to_arc_extract_type(V,E[j],true);
//       draw_line(edge_line, *map_param);
//     }
//     //Draw vertex
//     draw_point(V, *map_param, cv::Scalar(255,0,0));
//   }  

//   //Draw sample points  
//   for (int z=0;z<free_space_points.size();z++){
//       draw_point(free_space_points[z], *map_param, cv::Scalar(255,0,0));           
//   }
//   /*
//   //Draw global_planner path
//   for(int i=0;i<global_planner_path.size();i++){   
//     //Draw path
//     if(i<global_planner_path.size()-1){         
//       edge_line = to_arc_extract_type(global_planner_path[i],global_planner_path[i+1],true);
//       draw_line(edge_line, *map_param, cv::Scalar(0,255,0));    
//     }
//     std::cout << "gpp "<< i << ": " << global_planner_path[i].x << ", " << global_planner_path[i].y << std::endl;
//   }

//   //Draw dubins curve   
//   for(int i=0; i<obj.path_final_draw.size(); i++){
//     dubins_path_seg = obj.path_final_draw[i]; //retrieve three_segments
//     //std::cout << "Dubins_path_" << i << std::endl;
  
//     //Draw
//     draw_dubins_segment(dubins_path_seg, *map_param, cv::Scalar(0,0,255));
//   }
//   */
// }


// void UT_prm_planner(double* start_pose, double* goal_pose, std::vector<Point> bias_points, 
//          std::vector<Polygon> obstacle_list, double map_w, double map_h, int N, 
//          img_map_def *map_param){
//   //Create instance
//   PRM obj(obstacle_list, map_w, map_h, N, map_param->scale);

//   //call prm_planner
//   struct dubins_param dubins_param;
//   dubins_param.k_max = 10;
//   dubins_param.discretizer_size = 0.005;
//   double delta = 15;
//   double min_dist = 0.1001;    
//   double max_dist = 0.3001;
//   obj.prm_planner(start_pose, goal_pose, bias_points, dubins_param, delta);

//   //Retrieve all variables for drawing purposes
//   std::vector<Point> free_space_points = obj.get_free_space_points();
//   std::vector<std::pair<Point, std::vector<Point> >> prm_graph = obj.get_prm_graph();
//   std::vector<Point> global_planner_path = obj.get_global_planner_path(); 

//   //Draw output
//   //Drawing variables
//   std::pair<Point, std::vector<Point>> graph_node;  
//   Point V;
//   std::vector<Point> E; 
//   arc_extract edge_line;
//   Point E_point;
//   arc_extract dubins_path_seg;  
  
//   //draw polygons
//   for (size_t i = 0; i<obstacle_list.size(); i++){
//     draw_polygon(obstacle_list[i], *map_param);
//   }
  
//   //Draw prm_graph
//   for(int i=0; i<prm_graph.size(); i++){
//     //std::cout << "prm raph size: " << prm_graph.size() << std::endl;
//     graph_node = prm_graph[i];
//     V = graph_node.first; //Vertex
//     //std::cout << "prm V: " << V.x << ", " << V.y << std::endl;
//     E = graph_node.second; //Edges
//     //Draw edges    
//     for(int j=0;j<E.size();j++){ 
//       //std::cout << "Edge: " << E[j].x << ", " << E[j].y << std::endl;
//       edge_line = to_arc_extract_type(V,E[j],true);
//       draw_line(edge_line, *map_param);
//     }
//     //Draw vertex
//     draw_point(V, *map_param, cv::Scalar(255,0,0));
//   }  

//   //Draw sample points  
//   for (int z=0;z<free_space_points.size();z++){
//       draw_point(free_space_points[z], *map_param, cv::Scalar(255,0,0));           
//   }

//   //Draw global_planner path
//   for(int i=0;i<global_planner_path.size();i++){   
//     //Draw path
//     if(i<global_planner_path.size()-1){         
//       edge_line = to_arc_extract_type(global_planner_path[i],global_planner_path[i+1],true);
//       draw_line(edge_line, *map_param, cv::Scalar(0,255,0));    
//     }
//     std::cout << "gpp "<< i << ": " << global_planner_path[i].x << ", " << global_planner_path[i].y << std::endl;
//   }

//   //Draw dubins curve   
//   for(int i=0; i<obj.path_final_draw.size(); i++){
//     dubins_path_seg = obj.path_final_draw[i]; //retrieve three_segments
//     //std::cout << "Dubins_path_" << i << std::endl;
  
//     //Draw
//     draw_dubins_segment(dubins_path_seg, *map_param, cv::Scalar(0,0,255));
//   }  
// }



// /*----------------------------- Dubins section--------------------------------*/

// void UT_compute_triangle_angles(img_map_def *map_param){
//   triangle_angles t_angles;
//   double RAD2DEG = 180.0/M_PI;
//   Point qs,qm,qe;
//   //equilateral triangle pointing up (60,60,60)
//   // qs = Point(0.1,0.1);  
//   // qe = Point(0.9,0.1);
//   // qm = Point(0.5,0.793);
//   //equilateral triangle pointing down (60,60,60)
//   // qs = Point(0.1,0.793);
//   // qe = Point(0.9,0.793);  
//   // qm = Point(0.5,0.1);
//   //Rectangular triangle (90,45,45)
//   qs = Point(0.1,0.1);
//   qe = Point(0.5,0.1);  
//   qm = Point(0.1,0.5);
   
//   //draw triangle
//   arc_extract t_edges[3];
//   t_edges[0] = to_arc_extract_type(qs,qm,true);
//   t_edges[1] = to_arc_extract_type(qs,qe,true);
//   t_edges[2] = to_arc_extract_type(qe,qm,true);

//   for(int i=0;i<3;i++){
//     draw_line(t_edges[i],*map_param);
//   }

//   //compute angles
//   t_angles = compute_triangle_angles(qs,qe,qm);
//   std::cout << "beta1,beta2,beta3: " << (int)(t_angles.beta_1*RAD2DEG) << ","
//       << (int)(t_angles.beta_2*RAD2DEG) << "," << (int)(t_angles.beta_3*RAD2DEG) << std::endl;  
// }


// //////////Dubins arc collision test
// void UT_dubins_collision_test(struct arc_extract three_seg[3],
//           std::vector<Polygon> obstacle_list, img_map_def *map_param){
//   bool collision;
//   arc_extract line_data;

//   //printf("Noof obstacles = %d !!\n",obstacle_list.size());
  
//   #if 0
//   std::vector<Polygon> obstacle_list2;
//   obstacle_list2.push_back(obstacle_list[5]);//0 obstace has out of map and failing arc check
//   obstacle_list=obstacle_list2;
//   #endif

//   // //collision detection
//   collision = Highlevel_Box_dubins_check(obstacle_list, three_seg);
//   if(collision){
//     printf("COLLISION!!\n");
//   }
//   else{
//     printf("NO COLLISION!!\n");
//   }
//   // //Drawing
//   // //poly
//   // for (size_t i = 0; i<obstacle_list.size(); i++){
//   //   draw_polygon(obstacle_list[i], *map_param);
//   // }
//   // //dubins
 

//   Polygon input;
//   Polygon output;

//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         Construct_Bounding_Box(input , output);
//         draw_polygon(output, *map_param);
//         for (size_t j = 0; j<output.size()-1; j++){ //4corners of each box
//            //std::cout <<"Gkiri:BOX start_point.x= " << output[j].x << "start_point.y= " << output[j].y<< "end_point.y= " << output[j+1].x << "end_point.y= " << output[j+1].y << std::endl; 
//         }
//         output.clear();//clear pushback of output vector ref
//   }
  

//   for (size_t i = 0; i<obstacle_list.size(); i++){//no of boxes 
//         for (size_t j = 0; j<obstacle_list[i].size()-1; j++){ //4corners of each box
//             construct_line_structure(line_data,obstacle_list[i][j],obstacle_list[i][j+1]);
//             draw_line(line_data, *map_param, cv::Scalar(0,255,0)); 
//             //std::cout <<"Gkiri:Polygon start_point.x= " << obstacle_list[i][j].x << "start_point.y= " << obstacle_list[i][j].y<< "end_point.y= " << obstacle_list[i][j+1].x << "end_point.y= " << obstacle_list[i][j+1].y << std::endl; 
//         }
//     }

//   //dubins
//   for(int i=0;i<3;i++){
//     draw_dubins_segment(three_seg[i], *map_param);
//     std::cout <<"Gkiri:Dubins  start_point.x= " << three_seg[i].start_point.x << "start_point.y= " << three_seg[i].start_point.y<< "end_point.y= " << three_seg[i].end_point.x << "end_point.y= " << three_seg[i].end_point.y << std::endl; 
//     std::cout << "LSR:" << three_seg[i].LSR << ", radius:" << three_seg[i].radius << ", center:" << three_seg[i].center.x << "," << three_seg[i].center.y << std::endl;
//   }  

      

// }

// void UT_dubins_curve_test(struct arc_extract *three_seg,img_map_def *map_param)
// {
//     //Visualize dubins curve test
//     arc_extract dubins_line;

//     for(int i=0;i<3;i++){
//       switch (three_seg[i].LSR)
//       {
//       case 0: // Left arc
//         std::cout << "Left arc" << std::endl;
//         //Print values
//         std::cout << "Start point: " << three_seg[i].start_point.x 
//                                         << ", " << three_seg[i].start_point.y << std::endl;
//         std::cout << "End point: " << three_seg[i].end_point.x 
//                                         << ", " << three_seg[i].end_point.y << std::endl;
//         std::cout << "Radius: " << three_seg[i].radius << std::endl;
//         std::cout << "Center: " << three_seg[i].center.x 
//                           << ", " << three_seg[i].center.y <<std::endl;
//         std::cout << "Length: " << three_seg[i].length <<std::endl;
//         std::cout << "LSR: " << three_seg[i].LSR <<std::endl;

//         draw_dubins_segment(three_seg[i], *map_param);
                             
        
//         break;
//       case 1: // Straight line
//         std::cout << "Straight line" << std::endl;
//         dubins_line = to_arc_extract_type(three_seg[i].start_point,
//                                                 three_seg[i].end_point,true);
//         //Print values
//         std::cout << "Start point: " << three_seg[i].start_point.x 
//                                         << ", " << three_seg[i].start_point.y << std::endl;
//         std::cout << "End point: " << three_seg[i].end_point.x 
//                                         << ", " << three_seg[i].end_point.y << std::endl;
//         std::cout << "Radius: " << three_seg[i].radius << std::endl;
//         std::cout << "Center: " << three_seg[i].center.x 
//                           << ", " << three_seg[i].center.y << std::endl;
//         std::cout << "Length: " << three_seg[i].length << std::endl;
//         std::cout << "LSR: " << three_seg[i].LSR <<std::endl;
//         std::cout << "Calculated Length: " << dubins_line.length << std::endl;

//         draw_dubins_segment(dubins_line,*map_param);
//         break;
//       case 2: // Right arc
//         std::cout << "Right arc" << std::endl;
//         //Print values
//         std::cout << "Start point: " << three_seg[i].start_point.x 
//                                         << ", " << three_seg[i].start_point.y << std::endl;
//         std::cout << "End point: " << three_seg[i].end_point.x 
//                                         << ", " << three_seg[i].end_point.y << std::endl;
//         std::cout << "Radius: " << three_seg[i].radius << std::endl;
//         std::cout << "Center: " << three_seg[i].center.x 
//                           << ", " << three_seg[i].center.y << std::endl;
//         std::cout << "Length: " << three_seg[i].length <<std::endl;
//         std::cout << "LSR: " << three_seg[i].LSR <<std::endl;      

//         draw_dubins_segment(three_seg[i], *map_param);
//         break;
      
//       default:
//         std::cout << "Unknown LSR" << std::endl;
//         break;
//       }
//     }

// }


// void UT_cv_elipse_test(img_map_def *map_param){
//   //Ellipse works with a start angle and an end angle. Both are replaceable for each other,
//   //Angle goes from x-axis clockwise
//   arc_extract dt;
//   double start_angle, end_angle;    
//   dt.radius = 0.2;   

//   //Tests general
//   dt.center = Point(0.5, 0.5); 
//   double beta = -45; // From x-axis , clockwise > 0 , counter-clockwise < 0
//   double alpha_R = 180; // Angle difference between start and end
//   double alpha_L = 180;
//   int LSR = 2;

//   if(LSR == 2){ //Right
//     start_angle = beta;
//     end_angle = start_angle + alpha_R;
//   }
//   else{
//     start_angle = 360 + beta;
//     end_angle = start_angle - alpha_L;
//   }
  
//   //Draw ellipse
//   cv::Point center_scaled;
//   center_scaled.x = dt.center.x*map_param->scale*TO_CM;
//   center_scaled.y = dt.center.y*map_param->scale*TO_CM;
//   //Change of coordinate system (Img(0,0)--> left top corner, Map(0,0) left down corner)
//   center_scaled = coord_map_to_img(center_scaled, *map_param);
//   double radius_scaled = dt.radius*map_param->scale*TO_CM; 

//   cv::ellipse(map_param->img_map, center_scaled, cv::Size(radius_scaled, radius_scaled), 
//           0, start_angle, end_angle, cv::Scalar(255,255,255),1,15,0);
  
// }

// /* If we consider the lowest point as the starting point always, result of this test 
// should be from left to right: Small left arc, big right arc, big right arc, small left arc */
// void UT_draw_arc_test(img_map_def *map_param){
  
//   arc_extract dt;  
    
//   dt.radius = 0.05; //common radius
//   double length_small = 0.07854;
//   double length_big = 0.235362;

//   //Case 1: s.x > e.x

//   // //Right
//   // dt.center = Point(0.3, 0.15);
//   // dt.end_point = Point (0.3, 0.1);
//   // dt.start_point = Point (0.35, 0.15);
//   // dt.length = length_small;
//   // dt.LSR = 2;

//   // draw_arc(dt, *map_param);
//   // // start_angle = 0;
//   // // end_angle = 90; 

//   // //Left
//   // dt.center = Point(0.5, 0.15);
//   // dt.end_point = Point (0.5, 0.1);
//   // dt.start_point = Point (0.55, 0.15);
//   // dt.length = length_big;
//   // dt.LSR = 0;
  
//   // draw_arc(dt, *map_param);
//   // // start_angle = 360;
//   // // end_angle = 270; 

//   // //Case 2: s.x < e.x

//   // //Left
//   // dt.center = Point(0.7, 0.15);
//   // dt.start_point = Point (0.7, 0.1);
//   // dt.end_point = Point (0.75, 0.15); 
//   // dt.length = length_small;
//   // dt.LSR = 0;

//   // draw_arc(dt, *map_param);
//   // // start_angle = 0;
//   // // end_angle = 90; 

//   // //Right
//   // dt.center = Point(0.9, 0.15);
//   // dt.start_point = Point (0.9, 0.1);
//   // dt.end_point = Point (0.95, 0.15); 
//   // dt.length = length_big;   
//   // dt.LSR = 2;

//   // draw_arc(dt, *map_param);
//   // // start_angle = 360;
//   // // end_angle = 270;

//   //test on arc 20
//   dt.center = Point(0.75, 0.2); 
//   dt.end_point = Point (0.7, 0.2);
//   dt.start_point = Point (0.75, 0.25);
//   dt.length = length_small;
//   dt.LSR = 0;
  
//   draw_arc(dt, *map_param); 
// }


// ///////////////////////Collision workspace///////////////////////////////
// void UT_line_line_collision(img_map_def *map_param){


//   Point a,b,c,d,X;
//   bool intersection;

//   arc_extract line;
//   // a.x=0.1;
//   // a.y=0.1;
//   // b.x=0.5;
//   // b.y=0.5;
//   // c.x=0.5;
//   // c.y=0.1;
//   // d.x=0.1;
//   // d.y=0.5;

//   a.x=0.1;
//   a.y=0.1;
//   b.x=0.5;
//   b.y=0.5;
//   c.x=0.2;
//   c.y=0.2;
//   d.x=0.8;
//   d.y=0.8;
  
//   intersection=linelineIntersection(a,b,c,d,&X);
//   if(intersection)
//   {
//     std::cout << " $$$$$$$$$$$$$$$$$$$$  UT_line_line_collision True Intersected "  <<  std::endl;

//   }else
//   {
//     std::cout << " $$$$$$$$$$$$$$$$$$$$  UT_line_line_collision False Intersected "  <<  std::endl;
//   }
  
//   // draw_point(a, *map_param); 
//   // draw_point(b, *map_param); 
//   // draw_point(c, *map_param); 
//   // draw_point(d, *map_param); 
//   //draw_point(X, *map_param);
  

//   line = to_arc_extract_type(a,b,true);
//   draw_line(line, *map_param, cv::Scalar(0,255,0)); 
//   line = to_arc_extract_type(c,d,true);

//   draw_line(line, *map_param, cv::Scalar(255,255,0)); 
//   draw_point(X, *map_param, cv::Scalar(0,0,255));


// }

// //Test Case-2
// #if 0
// void UT_line_circle_collision(img_map_def *map_param){

//   Point X1,X2;
//   double a,b,c,r;
//   int intersection;
//   std::vector<Point> X;

//   cv::Point2f center,point_scaled; 
//   // a.x=0.1;
//   // a.y=0.1;
//   // b.x=0.5;
//   // b.y=0.5;
//   // c.x=0.5;
//   // c.y=0.1;
//   // d.x=0.1;
//   // d.y=0.5;

//   a=0.6;
//   b=0.8;
//   c=0.5;

//   r=0.3;
  
//   intersection=linecircleIntersection(r,a,b,c,X);
//   if(intersection==1)
//   {
//     std::cout << " $$$$$$$$$$$$$$$$$$$$  UT_line_line_collision Tangent "  <<  std::endl;

//   }else if(intersection==0)
//   {
//     std::cout << " $$$$$$$$$$$$$$$$$$$$  UT_line_line_collision False Intersected "  <<  std::endl;
//   }
//   else if(intersection==2)
//   {
//     std::cout << " $$$$$$$$$$$$$$$$$$$$  UT_line_line_collision Point Intersection"  <<  std::endl;
//   }
//   else{
//     std::cout << " $$$$$$$$$$$$$$$$$$$$  UT_line_line_collision Unknown "  <<  std::endl;
//   }
  
//   center.x=0.5;
//   center.y=0.5;

//   point_scaled.x = map_param->scale*center.x*TO_CM;
//   point_scaled.y = map_param->scale*center.y*TO_CM;

//   r = map_param->scale*r*TO_CM;
//   //cv::circle(*map_param,center,r);
//   point_scaled = coord_map_to_img(point_scaled, *map_param);
//   //circle( map_param->img_map, point_scaled, r, cv::Scalar( 255, 255, 255 ), 1, 8 );

//   circle( map_param->img_map, point_scaled, r, cv::Scalar( 0, 0, 255 ), 1, 8 );


//   arc_extract dt;
//   //line
//   // dt.start_point = Point (a*0.456287, b*0.599802);
//   // dt.end_point = Point (a*1.24375, b*0.650198);

//   dt.start_point = Point (0, 0);
//   dt.end_point = Point (0.9, 0.9);
//   dt.LSR = 1;

//   draw_line(dt, *map_param, cv::Scalar(0,255,0));  

// }

// #endif

// void UT_line_arc_collision_prof(img_map_def *map_param){

//   Point Line_Start, Line_End, Arc_Start, Arc_End,center;
//   double r,length ;
//   std::vector<Point> cal_points;
//   double rad = 0.05; //common radius
//   double rad2 = 2*sqrt(2)/5;
//   double length_small = 0.07854;
//   double length_big = 0.235362;
//   std::vector<Point> intersect_points;

//   Point firstIntersection, secondIntersection;

//   #if 0 //origin diagonal
//   Line_Start.x=0;
//   Line_Start.y=0;
//   Line_End.x=1.2605;//1.295;
//   Line_End.y=0.3578310535;//0.75;
//   #endif

//   #if 0 //origin diagonal
//   Line_Start.x=0.152;
//   Line_Start.y=0.182;
//   Line_End.x=0.4152605;//1.295;
//   Line_End.y=0.20578310535;//0.75;
//   #endif

//   #if 0 //Straight line diagonal
//   Line_Start.x=0.403;
//   Line_Start.y=0.041;
//   Line_End.x=0.403;//1.295;
//   Line_End.y=0.9;//0.367;
//   #endif

//   #if 1 //Straight line diagonal
//   Line_Start.x=0.53;
//   Line_Start.y=0.75;
//   Line_End.x=0.95;//1.295;
//   Line_End.y=0.75;//0.367;
//   #endif



//   #if 0 //small arc //Right
//   center= Point(0.3, 0.15);
//   Arc_End = Point (0.3, 0.1);
//   Arc_Start = Point (0.35, 0.15);
//   length = length_big;
//   r=rad;
//   #endif


//   #if 1 //dubins spl
//   center= Point(0.5, 0.75);
//   Arc_End = Point (0.7, 0.75);
//   Arc_Start = Point (0.5, 0.55);
//   r=0.1;
//   #endif

//   #if 0 //Big circle
//   center= Point(0.2, 0.2);
//   Arc_End = Point (0.4, 0.2);
//   Arc_Start = Point (0.2, 0.4);
//   //length = length_big;
//   r=0.2;
//   #endif


//   #if 0
//   center = Point(0.5, 0.15);
//   Arc_End = Point (0.5, 0.1);
//   Arc_Start = Point (0.55, 0.15);
//   length = length_small;
//   r=rad;
//   #endif

//   #if 0
//   center = Point(1, 0.3);
//   Arc_End = Point (1, 0.2);
//   Arc_Start = Point (1.1, 0.3);
//   length = length_big*2;
//   r=rad*2;
//   #endif


//   arc_extract dt;

//   dt.start_point = Line_Start;
//   dt.end_point = Line_End;
//   dt.LSR = 1;

//   draw_line(dt, *map_param, cv::Scalar(0,255,0));  

//   arc_extract curve;
//   curve.start_point = Arc_Start;
//   curve.end_point = Arc_End;
//   curve.LSR = 2;//R= clockwise from start L= anticlockwise from start
//   curve.center=center;
//   curve.radius=r;
//   curve.length=length;
//   draw_arc(curve, *map_param);

//   //bool intersect=lineArcIntersection_prof(dt,curve,intersect_points);
//   #if 0
//   bool intersect=get_line_circle_intersection( dt.start_point, dt.end_point ,curve.center,curve.radius,firstIntersection, secondIntersection);
//   intersect_points.push_back(firstIntersection);
//   intersect_points.push_back(secondIntersection);
//   #endif

//   #if 1
//   bool intersect=is_line_colliding_circle( dt.start_point, dt.end_point ,curve.center,curve.radius);

//   #endif

//   if(intersect){
//     std::cout <<"Gkiri:UT_line_arc_collision line-Arc INTERSECTION  " << std::endl;
//     //draw_point(intersect_points[0], *map_param,cv::Scalar(255,0,0)); 
//     //draw_point(intersect_points[1], *map_param,cv::Scalar(255,0,0)); 
//     // draw_point(cal_points[1], *map_param,cv::Scalar(255,0,0)); //Blue Point
//     // draw_point(cal_points[2], *map_param); 
//   }
//   else
//   {
//     std::cout <<"Gkiri:UT_line_arc_collision line-Arc NO INTERSECTION  " << std::endl;
//   }
  

// }


// // lineArcIntersection(Point Line_Start,Point Line_End,Point Arc_Start,Point Arc_End,double r,Point center)
// void UT_line_arc_collision(img_map_def *map_param){

//   Point Line_Start, Line_End, Arc_Start, Arc_End,center;
//   double r,length ;
//   std::vector<Point> cal_points;
//   double rad = 0.05; //common radius
//   double length_small = 0.07854;
//   double length_big = 0.235362;

//   #if 1 //origin diagonal
//   Line_Start.x=0;
//   Line_Start.y=0;
//   Line_End.x=0.45;//1.295;
//   Line_End.y=0.2535;//0.75;
//   #endif

//   #if 0 //Flat half line from Right
//   Line_Start.x=1.3;
//   Line_Start.y=0.5;
//   Line_End.x=0.75;
//   Line_End.y=0.5;
//   #endif
  
//   #if 0 //Flat full line(2 intersections) from Right
//   Line_Start.x=1.3;
//   Line_Start.y=0.95;
//   Line_End.x=0.25;
//   Line_End.y=0.95;
//   #endif


//   #if 0 //Flat line from Right
//   Line_Start.x=0.75 ;
//   Line_Start.y=0.5;
//   Line_End.x=1.3;
//   Line_End.y=0.5;
//   #endif

//   #if 0 //Flat line from left
//   Line_Start.x=0;
//   Line_Start.y=0.5;
//   Line_End.x=1.2995;
//   Line_End.y=0.5;
//   #endif


//   //ALVARO
//    #if 1
//   // center = Point(0.9, 0.3); 
//   // Arc_End = Point (1.2, 0.3);
//   // Arc_Start = Point (0.6, 0.3);
//   // r=0.3;
//   // length = M_PI*r;
//    #endif


//    #if 1 //small arc //Right
//   center= Point(0.3, 0.15);
//   Arc_End = Point (0.3, 0.1);
//   Arc_Start = Point (0.35, 0.15);
//   length = length_big;
//   r=rad;
//   #endif

//   arc_extract dt;

//   dt.start_point = Line_Start;
//   dt.end_point = Line_End;
//   dt.LSR = 1;

//   draw_line(dt, *map_param, cv::Scalar(0,255,0));  

//   arc_extract curve;
//   curve.start_point = Arc_Start;
//   curve.end_point = Arc_End;
//   curve.LSR = 2;
//   curve.center=center;
//   curve.radius=r;
//   curve.length=length;
//   draw_arc(curve, *map_param);

//   bool intersect=lineArcIntersection(dt,curve);

//   if(intersect){
//     std::cout <<"Gkiri:UT_line_arc_collision line-Arc INTERSECTION  " << std::endl;
//     // draw_point(cal_points[0], *map_param); 
//     // draw_point(cal_points[1], *map_param,cv::Scalar(255,0,0)); //Blue Point
//     // draw_point(cal_points[2], *map_param); 
//   }
//   else
//   {
//     std::cout <<"Gkiri:UT_line_arc_collision line-Arc NO INTERSECTION  " << std::endl;
//   }
  

// }


// //victims tes
// void UT_draw_victims(const std::vector<std::pair<int,Polygon>>& victim_list,img_map_def *map_param){

//   Polygon input;
//   Polygon output;
//   Point centroid;
//   for (size_t i = 0; i<victim_list.size(); i++){

//         input=victim_list[i].second;
//         draw_polygon(input, *map_param);
//         if(victim_list[i].first==1)
//         {
//           centroid=compute2DPolygonCentroid(input);
//           draw_point(centroid, *map_param);
//         }

//         if(victim_list[i].first==5){
//           centroid=compute2DPolygonCentroid(input);
//           draw_point(centroid, *map_param,cv::Scalar(255,0,0));//5 on blue

//         }
       

//   }

// }


// void UT_Bounding_Box(std::vector<Polygon> obstacle_list,img_map_def *map_param){

//   Polygon input;
//   Polygon output;

//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         Construct_Bounding_Box(input , output);
//         draw_polygon(output, *map_param);
//         output.clear();//clear pushback of output vector ref
//   }

// }


// /*Checking BoundingBox vs line  */
// void UT_Bounding_Box_line_check(std::vector<Polygon> obstacle_list,img_map_def *map_param){

//   Polygon input;
//   Polygon output;
//   std::vector<Polygon> Box_list;
//   bool Intersection;
//   struct arc_extract line_seg;
//   line_seg.start_point.x=0;//origin to TopRight
//   line_seg.start_point.y=0;
//   line_seg.end_point.x=1.0;
//   line_seg.end_point.y=0.2;
//   // line_seg.start_point.x=0.2;//TOPLEFT to Bottom right
//   // line_seg.start_point.y=0.85;
//   // line_seg.end_point.x=1.0;
//   // line_seg.end_point.y=0.1;
//   // line_seg.start_point.x=0.0;//flat line
//   // line_seg.start_point.y=0.5;
//   // line_seg.end_point.x=1.0;
//   // line_seg.end_point.y=0.5;
  
//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         Construct_Bounding_Box(input , output);
//         draw_polygon(output, *map_param);
//         output.clear();//clear pushback of output vector ref
//   }
//   draw_line(line_seg, *map_param);

//   Build_All_Bounding_Box(obstacle_list,Box_list);//make boxes
//   Intersection=Process_Box_line_check(Box_list,line_seg);
//   if(Intersection)
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-line INTERSECTION  " << std::endl;
//   else
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-line NO INTERSECTION  " << std::endl;

// }


// /*Checking BoundingBox vs line with obstacles API  */
// void UT_Bounding_Box_line_check_obstacles(std::vector<Polygon> obstacle_list,img_map_def *map_param){

//   Polygon input;
//   Polygon output;

//   bool Intersection;
//   struct arc_extract line_seg;
//   line_seg.start_point.x=0;//origin to TopRight
//   line_seg.start_point.y=0;
//   line_seg.end_point.x=1.0;
//   line_seg.end_point.y=0.8;
//   // line_seg.start_point.x=0.2;//TOPLEFT to Bottom right
//   // line_seg.start_point.y=0.85;
//   // line_seg.end_point.x=1.0;
//   // line_seg.end_point.y=0.1;
//   // line_seg.start_point.x=0.0;//flat line
//   // line_seg.start_point.y=0.5;
//   // line_seg.end_point.x=1.0;
//   // line_seg.end_point.y=0.5;
  
//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         Construct_Bounding_Box(input , output);
//         draw_polygon(output, *map_param);
//         output.clear();//clear pushback of output vector ref
//   }
//   draw_line(line_seg, *map_param);

//   Intersection=Process_Box_line_check_obstacles(obstacle_list,line_seg);
//   if(Intersection)
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-line INTERSECTION  " << std::endl;
//   else
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-line NO INTERSECTION  " << std::endl;

// }


// /*Checking BoundingBox vs Arc  */
// void UT_Bounding_Box_arc_check(std::vector<Polygon> obstacle_list,img_map_def *map_param){

//   Polygon input;
//   Polygon output;
//   std::vector<Polygon> Box_list;
//   bool Intersection;
//   struct arc_extract arc_seg;
//   Point Arc_Start, Arc_End, center;
//   double r,length ;
//   double rad = 0.1; //common radius


//   #if 0 //small arc //Right
//   center= Point(0.3, 0.15);
//   Arc_End = Point (0.3, 0.1);
//   Arc_Start = Point (0.35, 0.15);
//   r=rad;
//   #endif


//   #if 1 //small arc //Right
//   center= Point(0.75, 0.5);
//   Arc_End = Point (0.8, 0.4);
//   Arc_Start = Point (0.7, 0.6);
//   r=rad;
//   #endif

//   #if 0 //small arc //Right
//   center= Point(0.75, 0.5);
//   Arc_End = Point (0.75, 0.75);
//   Arc_Start = Point (0.75, 0.25);
//   r=0.25;
//   #endif

//   arc_seg.start_point = Arc_End;
//   arc_seg.end_point = Arc_Start;
//   arc_seg.LSR = 0;
//   arc_seg.center=center;
//   arc_seg.radius=r;
//   draw_arc(arc_seg, *map_param);
  
//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         Construct_Bounding_Box(input , output);
//         draw_polygon(output, *map_param);
//         output.clear();//clear pushback of output vector ref
//   }

//   Build_All_Bounding_Box(obstacle_list,Box_list);//make boxes
//   Intersection=Process_Box_arc_check_obstacles(Box_list,arc_seg);
//   if(Intersection)
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-arc INTERSECTION  " << std::endl;
//   else
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-arc NO INTERSECTION  " << std::endl;

// }


// /*Checking Polygons  vs line  */
// void UT_Polygons_line_check(std::vector<Polygon> obstacle_list,img_map_def *map_param){

//   Polygon input;
//   Polygon output;
//   bool Intersection;
//   struct arc_extract arc_seg;
//   Point Arc_Start, Arc_End, center;
//   double r,length ;
//   double rad = 0.1; //common radius

//   #if 0
//   Arc_End = Point (1.2, 0.7);
//   Arc_Start = Point (0.1, 0.15);
//   #endif

//   #if 0
//   Arc_End = Point (0.5, 0.7);
//   Arc_Start = Point (0.87, 0.4);
//   #endif

//   #if 1
//   Arc_End = Point (1.25, 0.7);
//   Arc_Start = Point (0.87, 0.4);
//   #endif

//   arc_seg.start_point = Arc_End;
//   arc_seg.end_point = Arc_Start;
//   arc_seg.LSR = 1;

//   draw_line(arc_seg, *map_param);
  
//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         draw_polygon(input, *map_param);
//         input.clear();//clear pushback of output vector ref
//   }

//   Intersection=narrow_polygon_obstacles_line_check(obstacle_list,arc_seg);
//   if(Intersection)
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-arc INTERSECTION  " << std::endl;
//   else
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-arc NO INTERSECTION  " << std::endl;

// }


// /*Checking Narrow Polygons vs Arc  */
// void UT_Polygons_arc_check(std::vector<Polygon> obstacle_list,img_map_def *map_param){

//   Polygon input;
//   Polygon output;
//   bool Intersection;
//   struct arc_extract arc_seg;
//   Point Arc_Start, Arc_End, center;
//   double r,length ;
//   double rad = 0.1; //common radius


//   #if 0 //small arc //Right
//   center= Point(0.3, 0.15);
//   Arc_End = Point (0.3, 0.1);
//   Arc_Start = Point (0.35, 0.15);
//   r=0.05;
//   #endif


//   #if 0 //small arc //Right
//   center= Point(0.75, 0.5);
//   Arc_End = Point (0.8, 0.4);
//   Arc_Start = Point (0.7, 0.6);
//   r=rad;
//   #endif

//   #if 0 //Big arc 
//   center= Point(0.75, 0.5);
//   Arc_End = Point (0.75, 0.75);
//   Arc_Start = Point (0.75, 0.25);
//   r=0.25;
//   #endif

//   #if 0
//   center = Point(0.9, 0.3); 
//   Arc_End = Point (1.2, 0.3);
//   Arc_Start = Point (0.6, 0.3);
//   r=0.3;
//   #endif

//   #if 0
//   center = Point(0.9, 0.6); 
//   Arc_End = Point (1.2, 0.6);
//   Arc_Start = Point (0.6, 0.3);
//   r=0.3;
//   #endif

//   #if 1
//   center = Point(1.0, 0.8); 
//   Arc_End = Point (1.2, 0.7);
//   Arc_Start = Point (0.9, 0.6);
//   r=0.2;
//   #endif


//   arc_seg.start_point = Arc_End;
//   arc_seg.end_point = Arc_Start;
//   arc_seg.LSR = 0;
//   arc_seg.center=center;
//   arc_seg.radius=r;
//   draw_arc(arc_seg, *map_param);
  
//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         draw_polygon(input, *map_param);
//         input.clear();//clear pushback of output vector ref
//   }

//   Intersection=narrow_polygon_obstacles_arc_check(obstacle_list,arc_seg);
//   if(Intersection)
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-arc INTERSECTION  " << std::endl;
//   else
//       std::cout <<"Gkiri:UT_Bounding_Box_line_check line-arc NO INTERSECTION  " << std::endl;

// }



// /*Checking BoundingBox vs Dubins 3 segments */
// void UT_Bounding_Box_dubins_check(std::vector<Polygon> obstacle_list,struct arc_extract *three_seg,img_map_def *map_param){

//   Polygon input;
//   Polygon output;
  
//   Highlevel_Box_dubins_check(obstacle_list,three_seg);
//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         Construct_Bounding_Box(input , output);
//         draw_polygon(output, *map_param);
//         output.clear();//clear pushback of output vector ref
//   }

// }


// /////Global Line Collision//////////////////////////////////////////
// void UT_Global_line_collision_check(std::vector<Polygon> obstacle_list,img_map_def *map_param){

//   Polygon input;
//   Polygon output;
//   bool Intersection;
//   struct arc_extract arc_seg;
//   Point Arc_Start, Arc_End, center;
//   double r,length ;
//   double rad = 0.1; //common radius

//   #if 0
//   Arc_End = Point (1.2, 0.7);
//   Arc_Start = Point (0.1, 0.15);
//   #endif

//   #if 0
//   Arc_End = Point (0.5, 0.7);
//   Arc_Start = Point (0.87, 0.4);
//   #endif

//   #if 0
//   Arc_End = Point (1.25, 0.7);
//   Arc_Start = Point (0.87, 0.4);
//   #endif

//   #if 0 //very close but not collsiion to box
//   Arc_End = Point (1.35, 0.5);
//   Arc_Start = Point (0.87, 0.4);
//   #endif

//   #if 0 //very close but not collsiion to box
//   Arc_End = Point (1.35, 0.5);
//   Arc_Start = Point (0.87, 0.45);
//   #endif

//   #if 1 //very close but not collsiion to box
//   Arc_End = Point (1.25, 0.4);
//   Arc_Start = Point (0.87, 0.4);
//   #endif

//   arc_seg.start_point = Arc_End;  
//   arc_seg.end_point = Arc_Start;
//   arc_seg.LSR = 1;


//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         Construct_Bounding_Box(input , output);
//         draw_polygon(output, *map_param);
//         output.clear();//clear pushback of output vector ref
//   }

//   draw_line(arc_seg, *map_param);
  
//   // for (size_t i = 0; i<obstacle_list.size(); i++){

//   //       input=obstacle_list[i];
//   //       draw_polygon(input, *map_param);
//   //       input.clear();//clear pushback of output vector ref
//   // }

//   Intersection=Global_Line_check(obstacle_list,arc_seg);
//   if(Intersection)
//       std::cout <<"Gkiri:UT_Global_line_collision_check COLLISION  " << std::endl;
//   else
//       std::cout <<"Gkiri:UT_Global_line_collision_check NO COLLISION  " << std::endl;

// }

// //--------------------------------------------------------------------------------------------

// /*Checking Narrow Polygons vs Arc  */
// void UT_Global_arc_collision_check(std::vector<Polygon> obstacle_list,img_map_def *map_param){

//   Polygon input;
//   Polygon output;
//   bool Intersection;
//   struct arc_extract arc_seg,line_data;
//   Point Arc_Start, Arc_End, center;
//   double r,length ;
//   double rad = 0.1; //common radius


//   #if 0 //small arc //Right
//   center= Point(0.3, 0.15);
//   Arc_End = Point (0.3, 0.1);
//   Arc_Start = Point (0.35, 0.15);
//   r=0.05;
//   #endif


//   #if 0 //small arc //Right
//   center= Point(0.75, 0.5);
//   Arc_End = Point (0.8, 0.4);
//   Arc_Start = Point (0.7, 0.6);
//   r=rad;
//   #endif

//   #if 0 //dubins spl
//   center= Point(0.5, 0.75);
//   Arc_End = Point (0.7, 0.75);
//   Arc_Start = Point (0.5, 0.55);
//   r=0.1;
//   #endif

//   #if 0 //Big arc 
//   center= Point(0.75, 0.5);
//   Arc_End = Point (0.75, 0.75);
//   Arc_Start = Point (0.75, 0.25);
//   r=0.25;
//   #endif

//   #if 0
//   center = Point(0.9, 0.3); 
//   Arc_End = Point (1.2, 0.3);
//   Arc_Start = Point (0.6, 0.3);
//   r=0.3;
//   #endif

//   #if 0
//   center = Point(0.9, 0.6); 
//   Arc_End = Point (1.2, 0.6);
//   Arc_Start = Point (0.6, 0.3);
//   r=0.3;
//   #endif

//   #if 1
//   center = Point(1.0, 0.8); 
//   Arc_End = Point (1.2, 0.7);
//   Arc_Start = Point (0.9, 0.6);
//   r=0.2;
//   #endif


//   arc_seg.start_point = Arc_End;
//   arc_seg.end_point = Arc_Start;
//   arc_seg.LSR = 0;
//   arc_seg.center=center;
//   arc_seg.radius=r;
//   draw_arc(arc_seg, *map_param);
  

//   for (size_t i = 0; i<obstacle_list.size(); i++){

//         input=obstacle_list[i];
//         Construct_Bounding_Box(input , output);
//         draw_polygon(output, *map_param);
//         for (size_t j = 0; j<output.size()-1; j++){ //4corners of each box
//            //std::cout <<"Gkiri:BOX start_point.x= " << output[j].x << "start_point.y= " << output[j].y<< "end_point.y= " << output[j+1].x << "end_point.y= " << output[j+1].y << std::endl; 
//         }
//         output.clear();//clear pushback of output vector ref
//   }
  

//   for (size_t i = 0; i<obstacle_list.size(); i++){//no of boxes 
//         for (size_t j = 0; j<obstacle_list[i].size()-1; j++){ //4corners of each box
//             construct_line_structure(line_data,obstacle_list[i][j],obstacle_list[i][j+1]);
//             draw_line(line_data, *map_param, cv::Scalar(0,255,0)); 
//             //std::cout <<"Gkiri:Polygon start_point.x= " << obstacle_list[i][j].x << "start_point.y= " << obstacle_list[i][j].y<< "end_point.y= " << obstacle_list[i][j+1].x << "end_point.y= " << obstacle_list[i][j+1].y << std::endl; 
//         }
//   }

//   Intersection=Global_Arc_check(obstacle_list,arc_seg);
//   if(Intersection)
//       std::cout <<"Gkiri:UT_Global_arc_collision_check COLLISION  " << std::endl;
//   else
//       std::cout <<"Gkiri:UT_Global_arc_collision_check NO COLLISION  " << std::endl;

// }













// ///////////////////////////////////////////////////////////////





// void UT_KDTree(img_map_def *map_param){
//   //Generate free_space_points
//   std::vector<std::pair<Point, std::vector<Point> >> graph_test = generate_graph_test();
//   std::vector<Point> free_space_points_test = generate_free_space_points_test(graph_test);

//   //Variables
//   double rad = .01; //max distance
//   Point Pt_eg = Point(0.85,0.25);
  
//   //Convert sample points to point_t
//   point_t pt_t;
//   pointVec pt_t_points;
//   for(Point Pt : free_space_points_test){
//       pt_t = Point_to_point_t(Pt);
//       pt_t_points.push_back(pt_t);
//   }
//   //Create tree
//   KDTree tree(pt_t_points);

//   //Convert to point_t
//   pt_t = Point_to_point_t(Pt_eg);    

//   //nearest points     
//   auto nearest_point = tree.nearest_point(pt_t);    

//   //neighborhood points     
//   auto neighborhood_points = tree.neighborhood_points(pt_t, rad);    

//   //Convert back to Points 
//   //Neighbors  
//   Point Pt;
//   std::vector<Point> Pt_points;
//   for(point_t pt_t : neighborhood_points){
//       Pt = point_t_to_Point(pt_t);
//       Pt_points.push_back(Pt);
//   }
//   //print
//   std::cout << "neighbors" << std::endl;
//   std::cout << "size: " << Pt_points.size() << std::endl;
//   for (Point b : Pt_points) {
//       std::cout << b.x << "," << b.y << std::endl;
//   } 

//   //empty test
//   if(!Pt_points.size()){printf("empty test sucessful\n");}

//   //Nearest
//   Pt = point_t_to_Point(nearest_point);
//   //print
//   std::cout << "nearest" << std::endl; 
//   std::cout << Pt.x << "," << Pt.y << std::endl;

//   //Draw
//   //All points
//   for(Point p:free_space_points_test){
//     draw_point(p,*map_param);
//   }
//   //Neighborhood points (red)
//   for(int i=0;i<Pt_points.size();i++){
//     if(i!=0){ //neighbors     
//       draw_point(Pt_points[i], *map_param, cv::Scalar(0,0,255));  
//     }    
//   }
//   //Nearest point (green)
//   draw_point(Pt, *map_param, cv::Scalar(0,255,0)); //nearest
// }


// // /******************** vec2d unit test ************************/
// // void UT_vec2d(Point qs, Point qm, Point qe, double map_w, double map_h, img_map_def *map_param){
  
  
// //   double minimum_distance(vec2d v, vec2d w, vec2d p);
// // }
