#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <iostream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>

#include "loadImage.cpp"

#include "findRobot.cpp"

#include "image_undistort.hpp"

#include "clipper.hpp"

#include "inflate_polygons.cpp"

#include "draw_functions.cpp"

//#include "PRM.h"

#include <collision_detection.hpp>

#include "process_map.cpp"

#include "unit-testing.cpp"

#include "mission_apis.hpp"


//#include "DubinsCurvesHandler.hpp"

//Unit test and printouts variables
#define DUBINS_CURVE 0
#define DUBINS_TEST 0
#define PRM_PLANNER_TEST 0
#define DRAW_GATE_TEST 0
#define UNIT_TEST 0

#define LAB_DIR 0


namespace student {

//  void loadImage(cv::Mat& img_out, const std::string& config_folder){  
//    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED-2" );
//  }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED-ImageListener" );
    std::cout << "Team4::---------------------genericImageListener----------------------------->>"  << std::endl;

    static size_t id = 0;

    const char *folder_path="/home/gkiri/Desktop/Computer_Vision/";

    imshow(topic,img_in);

    char key;
    key = cv::waitKey(30);

    std::cout<<"waitkey-"  << key << std::endl;

    std::stringstream img_file;

    if(key == 115 )
    {
      std::cout << "Team4:: Entered s"  << std::endl;
      img_file << folder_path << std::setfill('0') 
          << std::setw(3)  << (id++) << ".jpg";

      //std::cout << img_file.str() << std::endl;
      cv::imwrite( img_file.str(), img_in );

      std::cout << "Saved image " << img_file.str() << std::endl;
    }
 
    std::cout << "Team4:: Exit"  << std::endl;
       
  }

  //-------------------------------------------------------------------------
  //          EXTRINSIC CALIB IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Defintion of the function pickNPoints and the callback mouseCallback.
  // The function pickNPoints is used to display a window with a background
  // image, and to prompt the user to select n points on this image.
  static cv::Mat bg_img;
  static std::vector<cv::Point2f> result;
  static std::string name;
  static std::atomic<bool> done;
  static int n;
  static double show_scale = 1.0;

  void mouseCallback(int event, int x, int y, int, void* p)
  {
    if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;

    result.emplace_back(x*show_scale, y*show_scale);
    cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
    cv::imshow(name.c_str(), bg_img);

    if (result.size() >= n) {
      usleep(500*1000);
      done.store(true);
    }
  }

  std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)
  {
    result.clear();
    cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
    cv::resize(img, bg_img, small_size);
    //bg_img = img.clone();
    name = "Pick " + std::to_string(n0) + " points";
    cv::imshow(name.c_str(), bg_img);
    cv::namedWindow(name.c_str());
    n = n0;

    done.store(false);

    cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
    while (!done.load()) {
      cv::waitKey(500);
    }

    cv::destroyWindow(name.c_str());
    return result;
  }

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );   

    std::cout << "Team4::---------------------extrinsicCalib----------------------------->>"  << std::endl;
    //cv::solvePnP(object_points, , camera_matrix,);
    std::string file_path = config_folder + "/extrinsicCalib.csv";

    std::vector<cv::Point2f> image_points;

    if (!std::experimental::filesystem::exists(file_path)){

      std::experimental::filesystem::create_directories(config_folder);

      image_points = pickNPoints(4, img_in);
      // SAVE POINT TO FILE
      // std::cout << "IMAGE POINTS: " << std::endl;
      // for (const auto pt: image_points) {
      //   std::cout << pt << std::endl;
      // }
      std::ofstream output(file_path);
      if (!output.is_open()){
        throw std::runtime_error("Cannot write file: " + file_path);
      }
      for (const auto pt: image_points) {
        output << pt.x << " " << pt.y << std::endl;
      }
      output.close();
    }else{
      // LOAD POINT FROM FILE
      std::ifstream input(file_path);
      if (!input.is_open()){
        throw std::runtime_error("Cannot read file: " + file_path);
      }
      while (!input.eof()){
        double x, y;
        if (!(input >> x >> y)) {
          if (input.eof()) break;
          else {
            throw std::runtime_error("Malformed file: " + file_path);
          }
        }
        image_points.emplace_back(x, y);
      }
      input.close();
    }

    cv::Mat dist_coeffs;
    dist_coeffs   = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    if (!ok) {
      std::cerr << "FAILED SOLVE_PNP" << std::endl;
    }

    return ok;


  }


  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){

    static ImageUndistort image_undistort;

    if(!image_undistort.isInitialized()){
      image_undistort.initialize(img_in.size(), cam_matrix, dist_coeffs);
    }

    image_undistort.undistort(img_in, img_out);  
  } 


 //-------------------------------------------------------------------------
  //          FIND PLANE TRANSFORM
  //-------------------------------------------------------------------------
  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec,
                          const cv::Mat& tvec,
                          const std::vector<cv::Point3f>& object_points_plane,
                          const std::vector<cv::Point2f>& dest_image_points_plane,
                          cv::Mat& plane_transf, const std::string& config_folder){

    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
  }


 //-------------------------------------------------------------------------
  //          UNWARP TRANSFORM
  //-------------------------------------------------------------------------
  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf,
              const std::string& config_folder){
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){

    std::cout << "Gkiri::$$$$$$$$$$$$$$$ processMap-------------- "  << std::endl;
    bool arena=true;
    //const std::string& template_folder="/home/gkiri/Desktop/Applied_Robotics/Workspace/Team_Project/imgs/template/";
    //const std::string& template_folder = "/home/alvaro/workspace/AppliedRoboticsStudentInterface/imgs/template/";

    const std::string& template_folder = "/home/robotics/workspace/group_4/imgs/template/";

    //const std::string& template_folder="../imgs/template/";
    // Convert color space from BGR to HSV
    cv::Mat hsv_img,temp_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
    if(!res1) std::cout << "processObstacles return false" << std::endl;

    //const bool res2 = processGate(hsv_img, scale, gate);
    const bool res2 = Gate_Process_second(hsv_img, temp_img, scale,gate,arena);
    if(!res2) std::cout << "processGate return false" << std::endl;

    //const bool res3 = processVictims(hsv_img, scale, victim_list);
    const bool res3 = processVictims_student(img_in,hsv_img, scale, victim_list,template_folder);
    if(!res3) std::cout << "processVictims return false" << std::endl;


    //std::cout << "processMap ENd = " << (res1 && res2 && res3 )<< std::endl;
    return res1 && res2 && res3;

  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );

    //std::cout << "Gkiri:: findRobot:: Started" << std::endl;
    findRobot_api(img_in, scale, triangle, x, y, theta, config_folder);
    // Save robot parameters
    x = x;
    y = y;
    theta = theta;
  }


  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
                const std::vector<std::pair<int,Polygon>>& victim_list, 
                const Polygon& gate, const float x, const float y, const float theta, 
                Path& path,
                const std::string& config_folder)

  {   
    /*************CONFIG VARIABLES*********************/
    //Load config file
    int dir = 0; //0-Alvaro, 1-Kiran  
    std::string config_dir;
    #if LAB_DIR
  	config_dir = "/home/robotics/workspace/group_4/src/config_parameters.txt";
    #else    
    if(dir){
      config_dir = "/home/robotics/workspace/group_4/src/config_parameters.txt";
    }
    else{
      config_dir = "/home/alvaro/workspace/AppliedRoboticsStudentInterface/src/config_parameters.txt";
    }    
    #endif

    //Robot parameters
    double robot_length = load_config_param(config_dir, "robot_length"); //(m)    
    double robot_width = load_config_param(config_dir, "robot_width"); //(m)
    double robot_speed = load_config_param(config_dir, "robot_speed"); //(m/s)   

    //Visualising the map parameters
    double map_w = load_config_param(config_dir, "map_w"); //real map width in m    
    double map_h = load_config_param(config_dir, "map_h"); //real map height in m     
    double img_map_w = load_config_param(config_dir, "img_map_w"); //image map width in pixels
    //image map height is calculated through the other three parameters 
    
    //Sampling
    int n_samples = load_config_param(config_dir, "n_samples");
    
    //Local planner
    double min_dist = load_config_param(config_dir, "min_dist");     
    double max_dist = load_config_param(config_dir, "max_dist");
    
    //Dubins parameters
    double k_max = load_config_param(config_dir, "k_max");    
    double discretizer_size = load_config_param(config_dir, "discretizer_size");    

    //Dubins planner
    double delta = load_config_param(config_dir, "delta"); //tune angle in degrees    

    //mission parameters
    int mission_id = load_config_param(config_dir, "mission_id");
    double victim_reward = load_config_param(config_dir, "victim_reward"); //s
    double goal_delta = load_config_param(config_dir, "goal_delta"); //m
   
    //Drawing flag
    int drawing = load_config_param(config_dir, "drawing");
    int unit_test = load_config_param(config_dir, "unit_test");
    /*****************************************************/


    //Initialize map matrix and scale
    img_map_def map_param = initialize_img_map(map_w, map_h, img_map_w);    

    /* Inflate polygons and walls -------------------------------------------*/
    //Radius of circle approximating robot shape (m) 
    double OFFSET_walls = sqrt(pow(robot_length,2) + pow(robot_width,2))/2; //more restrictive case, diagonal of robot
    double OFFSET_polygon = robot_length/2; // less restrictive case --> length of robot                           
    std::vector<Polygon> clean_obstacle_list, inflated_obstacle_list, inflated_walls;

    //sanity check: Eliminate polygons with 2 or less edges
    for(Polygon poly:obstacle_list){   
      if(poly.size() > 2){
        clean_obstacle_list.push_back(poly);
      }
    }
    
    //Inflate polygon function   
    inflated_obstacle_list = inflate_polygons(clean_obstacle_list, OFFSET_polygon);
    //Inflate walls    
    inflated_walls = inflate_walls(map_w, map_h, OFFSET_polygon);
    //Add inflated walls to inflated_obstacle_list
    inflated_obstacle_list.insert(inflated_obstacle_list.end(),inflated_walls.begin(),
            inflated_walls.end());

   
    /****************** Missions **********************************************/
    
    //variables
    double start_pose[3], gate_pose[3];
    struct dubins_param dubins_param; 
    std::vector<Point> bias_points;

    //output for missions
    struct mission_output_0 miss_output_0;
    struct mission_output_1 miss_output_1; 
    struct mission_output_2 miss_output_2;  
    
    //set dubins param
    dubins_param.k_max = k_max;
    dubins_param.discretizer_size = discretizer_size;

    //set PRM params
    struct PRM_param PRM_param;
    PRM_param.map_h = map_h;
    PRM_param.map_w = map_w;
    PRM_param.obstacle_list = inflated_obstacle_list;
    PRM_param.n_samples = n_samples;
    PRM_param.max_dist = max_dist;
    PRM_param.min_dist = min_dist;
    PRM_param.scale = map_param.scale;
    
    //start point    
    start_pose[0] = x;
    start_pose[1] = y;
    start_pose[2] = theta;   

    //end point = gate_pose    
    get_gate_pose(gate, map_h, map_w, robot_length, gate_pose, goal_delta);

    

    /*--------mission selection ------------*/
    switch (mission_id){

    case 0:      
      miss_output_0 = mission_0(dubins_param, start_pose, gate_pose);     
      if(miss_output_0.path.empty()){
        printf("Empty path\n");   
        break;
      }
      else{
        //set path
        path = miss_output_0.path;
        if(drawing){       
          drawing_mission_0(start_pose,gate_pose,gate,miss_output_0,map_param);
        }
      }
      break;
      

    case 1:      

      miss_output_1 = mission_1(PRM_param, dubins_param, start_pose, gate_pose, 
                                  bias_points, delta);      
      if(miss_output_1.path.empty()){           
        printf("Empty path\n");        
        break;
      }
      else{
        //Set path
        path = miss_output_1.path;
        if(drawing){
          drawing_mission_1(inflated_obstacle_list, miss_output_1, map_param);
        }
      }      
      break;    
    
    case 15:
      
      miss_output_1 = mission_15(PRM_param, dubins_param, start_pose, gate_pose, 
                                  victim_list, delta);      
      if(miss_output_1.path.empty()){           
        printf("Empty path\n");        
        break;
      }
      else{
        //Set path
        path = miss_output_1.path;
        if(drawing){
          drawing_mission_1(inflated_obstacle_list, miss_output_1, map_param);
        }
      }      
      break;

    case 2:
      
      miss_output_2 = mission_2(PRM_param, dubins_param, start_pose, gate_pose, 
                          victim_list, delta, victim_reward, robot_speed);
      if(miss_output_2.path.empty()){   
        printf("Empty path\n");
        break;
      }
      else{
        //Set path
        path = miss_output_2.path;
        if(drawing){
          drawing_mission_2(inflated_obstacle_list, victim_list, miss_output_2, map_param);
        }
      }      
      break;
    
    
    default:
      printf("Student interface: No mission selected\n");
      break;
    }   
    





    /*********************UNIT TEST SECTION*******************************************************/
    #if UNIT_TEST
    #if DRAW_GATE_TEST
    //Drawing
    //polygons
    for (size_t i = 0; i<obstacle_list.size(); i++){
      draw_polygon(obstacle_list[i], map_param, cv::Scalar(255,0,0));
    }
    //gate
    draw_polygon(gate, map_param, cv::Scalar(255,0,0));
    //center point
    Point rect_center = Point(gate_pose[0], gate_pose[1]);
    std::cout << "Gate robot pose: " << gate_pose[0] << "," << gate_pose[1] << std::endl;
    draw_point(rect_center, map_param);    
    #endif
    /****************************************************************************/

    

    /*****************Alvaro overall and prm planner Unit testing ********************/
    if (unit_test){
      //double start_pose[3], goal_pose[3];
      //struct dubins_param dubins_param;
      //double RAD2DEG = 180.0/M_PI;
      //dubins_param.k_max = k_max;
      //dubins_param.discretizer_size = discretizer_size;



      //start point    
      start_pose[0] = x;
      start_pose[1] = y;
      start_pose[2] = theta;
      
      //end point
      gate_pose[0] = 1.34648;
      gate_pose[1] = 0.747094;
      gate_pose[2] = M_PI/2;    
      //bias points
      //std::vector<Point> bias_points;
      //Call planner

      //UT_overall_planner(start_pose, gate_pose, inflated_obstacle_list,map_w,map_h,n_samples, &map_param); 
      //UT_prm_planner(qs, qe, bias_points, inflated_obstacle_list, map_w,map_h,N, &map_param); 
      //Create instance
      /*
      PRM obj(inflated_obstacle_list, map_w, map_h, N);
      //call prm_planner   
      path = obj.prm_planner(start_pose, goal_pose, bias_points, dubins_param); 
      if(path.empty()){printf("Empty path\n");}
      else{
        for(int i=0;i<path.points.size();i++){
        std::cout << "point " << i << ": " << path.points[i].s <<","<< path.points[i].x <<","
        << path.points[i].y <<"," << path.points[i].theta <<","<< path.points[i].kappa << std::endl;
        }
      }
      
      //Retrieving outputs
      std::vector<Point> free_space_points = obj.get_free_space_points();
      std::vector<std::pair<Point, std::vector<Point> >> prm_graph = obj.get_prm_graph();
      std::vector<Point> global_planner_path = obj.get_global_planner_path(); 

      //Drawing

      //Drawing variables
      std::pair<Point, std::vector<Point>> graph_node;  
      Point V;
      std::vector<Point> E; 
      arc_extract edge_line;
      Point E_point;
      arc_extract dubins_path_seg;  
      
      //draw polygons
      for (size_t i = 0; i<inflated_obstacle_list.size(); i++){
        draw_polygon(inflated_obstacle_list[i], map_param);
      }
      
      //Draw prm_graph
      for(int i=0; i<prm_graph.size(); i++){
        //std::cout << "prm raph size: " << prm_graph.size() << std::endl;
        graph_node = prm_graph[i];
        V = graph_node.first; //Vertex
        //std::cout << "prm V: " << V.x << ", " << V.y << std::endl;
        E = graph_node.second; //Edges
        //Draw edges    
        for(int j=0;j<E.size();j++){ 
          //std::cout << "Edge: " << E[j].x << ", " << E[j].y << std::endl;
          edge_line = to_arc_extract_type(V,E[j],true);
          draw_line(edge_line, map_param);
        }
        //Draw vertex
        draw_point(V, map_param, cv::Scalar(255,0,0));
      }  

      //Draw sample points  
      for (int z=0;z<free_space_points.size();z++){
          draw_point(free_space_points[z], map_param, cv::Scalar(255,0,0));           
      }

      //Draw global_planner path
      for(int i=0;i<global_planner_path.size();i++){   
        //Draw path
        if(i<global_planner_path.size()-1){         
          edge_line = to_arc_extract_type(global_planner_path[i],global_planner_path[i+1],true);
          draw_line(edge_line, map_param, cv::Scalar(0,255,0));    
        }
        //std::cout << "gpp "<< i << ": " << global_planner_path[i].x << ", " << global_planner_path[i].y << std::endl;
      }

      //Draw dubins curve   
      for(int i=0; i<obj.path_final_draw.size(); i++){
        dubins_path_seg = obj.path_final_draw[i]; //retrieve three_segments
        //std::cout << "Dubins_path_" << i << std::endl;
      
        //Draw
        draw_dubins_segment(dubins_path_seg, map_param, cv::Scalar(0,0,255));
      } 
      */
    } 
    
    
    /*****************************************************************************/  
     
    
    /* Dubins Section-------------------------------------------*/
    #if DUBINS_TEST
    /* Dubins Section-------------------------------------------*/
    struct arc_extract three_seg[3];//segment extract
    //double start_pose[3], goal_pose[3];   

    //start point
    start_pose[0] = x;
    start_pose[1] = y;
    start_pose[2] = theta;

    // //start point
    // start_pose[0] = 1.4;
    // start_pose[1] = 0.4;
    // start_pose[2] = 0;
    
    //end point
    // goal_pose[0] = gate_pose[0];
    // goal_pose[1] = gate_pose[1];
    // goal_pose[2] = gate_pose[2];

    //down wall
    gate_pose[0] = x - 0.05;
    gate_pose[1] = y;
    gate_pose[2] = M_PI/4;

    //establish Dubins curve generator

    //DubinsCurvesHandler dubins_handler{};
    DubinsCurvesHandler dubins_handler(k_max, discretizer_size);
    Path path_container;

    DubinsCurve dubins_path = dubins_handler.findShortestPath(start_pose[0],start_pose[1],start_pose[2],
                                                gate_pose[0], gate_pose[1], gate_pose[2]);

    for(int j = 0; j < dubins_path.discretized_curve.size(); j++)
    {
        path.points.emplace_back(dubins_path.discretized_curve[j].s, 
              dubins_path.discretized_curve[j].xf, dubins_path.discretized_curve[j].yf, 
              dubins_path.discretized_curve[j].thf, dubins_path.discretized_curve[j].k);
    }

    //test for create_three_seg
    create_three_seg(three_seg, start_pose[0], start_pose[1], dubins_path);

    //test for dubins collision detection
    UT_dubins_collision_test(three_seg, inflated_obstacle_list, &map_param);

    // concatenate_dubins_path(path_container, dubins_path, discretizer_size);
    // path = path_container;
    // if(path.empty()){printf("Empty path\n");}
    // else{
    //   for(int i=0;i<path.points.size();i++){
    //   std::cout << "point " << i << ": " << path.points[i].s <<","<< path.points[i].x <<","
    //   << path.points[i].y <<"," << path.points[i].theta <<","<< path.points[i].kappa << std::endl;
    //   }
    // }

    #endif

    
    /*********************UNIT TEST SECTION*******************************************************/    
    //permute
    //UT_permute();
    //UT_print_all_comb(victim_list);

    /************************Draw test******************************************/
    //Print an example of th drawing functions in a single image
    //draw_test(obstacle_list,x,y,theta,victim_list,map_param);
    
    //UT_draw_victims(victim_list,&map_param);

    /*****************************************************************************/ 

    /*****************Alvaro PRM ellipse and draw Unit testing ******************/
    //UT_cv_elipse_test(&map_param);
    //UT_draw_arc_test(&map_param);
     /***************************************************************************/

    /***********************Gkiri arc_draw_test Unit Testing**********************/
    //UT_arc_draw_test(&map_param);
    /*****************************************************************************/

    /******************Gkiri UT_dubins_curve_test space Unit Testing*************/
    //UT_dubins_curve_test(three_seg,&map_param);
    //UT_Bounding_Box_dubins_check(obstacle_list,three_seg,&map_param);
    /*****************************************************************************/
    
    /* **********************Gkiri PRM space Unit Testing*************************/
    UT_sample_generation(inflated_obstacle_list,map_w,map_h,n_samples,&map_param);
    /*****************************************************************************/
    
    /************************Process map unit testing******************************/
    // //Draw original polygons on top of inflated ones
    // for (size_t i = 0; i<obstacle_list.size(); i++){
    //   draw_polygon(obstacle_list[i], map_param, cv::Scalar(255,0,0));
    // }

    // for (size_t i = 0; i<victim_list.size(); i++){
    //   draw_polygon(victim_list[i].second, map_param, cv::Scalar(255,0,0));
    // }  
    /*****************************************************************************/ 

    //* **********************Line-Line Collision Unit Testing********************/
    //UT_line_line_collision(&map_param);
    //UT_line_circle_collision(&map_param);
    /*****************************************************************************/ 


    /*****************GkiriCollision Unit testing ********************************/
    //UT_line_arc_collision_prof(&map_param);
    //UT_Bounding_Box(obstacle_list,&map_param);
    //UT_Bounding_Box_line_check(obstacle_list,&map_param);//boundingbox vs line
    //UT_Bounding_Box_line_check_obstacles(obstacle_list,&map_param);//boundingbox vs line
    //UT_Bounding_Box_arc_check(obstacle_list,&map_param);
    //UT_Polygons_arc_check(obstacle_list,&map_param);
    //UT_Polygons_line_check(obstacle_list,&map_param);

    //UT_Global_line_collision_check(obstacle_list,&map_param);
    //UT_Global_arc_collision_check(obstacle_list,&map_param);
    /*****************************************************************************/ 

    /*****************Alvaro dubins path Unit testing *************************/
    //UT_dubins_path(inflated_obstacle_list, &map_param);    

    /*****************Alvaro PRM local planner Unit testing **********************/
    //UT_local_planner(inflated_obstacle_list, &map_param);
    //UT_KDTree(&map_param);
    /*****************************************************************************/

    /*****************Alvaro PRM global planner Unit testing *************************/
    //UT_global_planner(inflated_obstacle_list, &map_param);    
    /*****************************************************************************/

    /*****************Alvaro dubins path Unit testing *****************************/
    //UT_dubins_path(inflated_obstacle_list, &map_param);  
    //UT_compute_triangle_angles(&map_param);
    /*****************************************************************************/

    #endif


    /* Map visualization -------------------------------------------*/    
    if(drawing){
      //Show map image
      cv::imshow("Image",map_param.img_map);
      cv::waitKey( 0.01 );
    }
      
    

    return true;

  }

}

