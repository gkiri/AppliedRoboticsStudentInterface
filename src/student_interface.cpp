#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>

#include "loadImage.cpp"

#include "findRobot.cpp"

#include "dubins_curve.hpp"

#include "image_undistort.hpp"

#include "clipper.hpp"

#include "inflate_polygons.cpp"

#include "draw_functions.cpp"

#include "PRM.h"

#include <collision_detection.hpp>

#include "unit-testing.cpp"

#include "process_map.cpp"

//Unit test and printouts variables
#define VISUALIZE_MAP 1   //(0)Deactivated - (1)Visualize elements in map
#define DUBINS_CURVE 1
#define DUBINS_TEST 0

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
    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
    if(!res1) std::cout << "processObstacles return false" << std::endl;
    const bool res2 = processGate(hsv_img, scale, gate);
    if(!res2) std::cout << "processGate return false" << std::endl;
    //const bool res3 = processVictims(hsv_img, scale, victim_list);
    const bool res3 = processVictims_student(img_in,hsv_img, scale, victim_list,config_folder);
    if(!res3) std::cout << "processVictims return false" << std::endl;

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
    //Robot parameters (move to another file?)
    const double robot_length = 0.2; //(m)
    const double robot_width = 0.14; //(m)
    struct arc_extract three_seg[3];//segment extract

    //Visualising the map parameters
    double map_w = 1.5; //real map width in m
    double map_h = 1; //real map height in m  
    double img_map_w = 720; //image map width in pixels
    //image map height is calculated through the other three parameters
    /* Initialize map image -------------------------------------------*/

    //Initialize map matrix and scale
    img_map_def map_param = initialize_img_map(map_w, map_h, img_map_w);
    //std::cout << "Scale: " << map_param.scale << std::endl; 

    /* Inflate polygons -------------------------------------------*/
    //Radius of circle approximating robot shape (m) 
    //double OFFSET = sqrt(pow(robot_length/2,2) + pow(robot_width/2,2)); //more restrictive case, diagonal of robot
    double OFFSET = 0.2/2; // less restrictive case --> length of robot                           
    std::vector<Polygon> inflated_obstacle_list;
    //Inflate polygon function --> Return std::vector<Polygon>
    inflated_obstacle_list = inflate_polygons(obstacle_list, OFFSET);

    //Print polygons    
    //print_polygons_out(obstacle_list, inflated_obstacle_list);

    /* **********************Gkiri PRM space Unit Testing*************************/
    //UT_sample_generation(inflated_obstacle_list ,&map_param);
    /*****************************************************************************/
    
    // //Draw original polygons on top of inflated ones
    // for (size_t i = 0; i<obstacle_list.size(); i++){
    //   draw_polygon(obstacle_list[i], map_param, cv::Scalar(255,0,0));
    // }

    for (size_t i = 0; i<victim_list.size(); i++){
      draw_polygon(victim_list[i].second, map_param, cv::Scalar(255,0,0));
    }

  /*****************Alvaro PRM global planner Unit testing *************************/
    //UT_global_planner(inflated_obstacle_list, &map_param);    
  /*****************************************************************************/

  /*****************Ambike PRM local planner Unit testing **********************/
    //UT_local_planner(inflated_obstacle_list, &map_param);
  /* **********************Ambikeya Line-Line Collision Unit Testing*************************/
    //UT_line_line_collision(&map_param);
    //UT_line_circle_collision(&map_param);

  /*****************GkiriCollision Unit testing **********************/
    //UT_line_arc_collision(&map_param);
    //UT_Bounding_Box(obstacle_list,&map_param);
    //UT_Bounding_Box_line_check(obstacle_list,&map_param);//boundingbox vs line
    //UT_Bounding_Box_arc_check(obstacle_list,&map_param);
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
 

  /*****************Alvaro dubins path Unit testing *************************/
    //UT_dubins_path(inflated_obstacle_list, &map_param);  
    //UT_compute_triangle_angles(&map_param);
  /*****************************************************************************/

    

    
    
    /* Draw test-------------------------------------------*/
    //Print an example of th drawing functions in a single image
    //draw_test(obstacle_list,x,y,theta,victim_list,map_param);

    /*****************Alvaro PRM ellipse and draw Unit testing *************************/
    //UT_cv_elipse_test(&map_param);
    //UT_draw_arc_test(&map_param);
     /*****************************************************************************/

    /***********************Gkiri arc_draw_test Unit Testing*************************/
    //UT_arc_draw_test(&map_param);
    /*****************************************************************************/
    
    /* Dubins Section-------------------------------------------*/
    //std::cout << "Before path: " << path.size() << std::endl;
    double q0[3];//start point
    double q1[3];//end point
    double rho=0.1; //turning radius

    //Robot position
    //std::cout << "Robot position: " << x << ", "<< y << ", " << theta << std::endl;
    //Dubins test
    #if DUBINS_TEST

    q0[0]=0;//start of dubins
    q0[1]=0;
    q0[2]=0;
    q1[0]=0.8;//end of dubins
    q1[1]=0.3;
    q1[2]=0;
    dubins_wrapper_api(path,three_seg,q0,q1,rho);
    /* **********************Gkiri UT_dubins_curve_test space Unit Testing*************************/
    //UT_dubins_curve_test(three_seg,&map_param);
    //UT_Bounding_Box_dubins_check(obstacle_list,three_seg,&map_param);
    /*********************************************************************************************/
    #endif

    /* Map visualization -------------------------------------------*/
    #if VISUALIZE_MAP
    //Show map image
    cv::imshow("Image",map_param.img_map);
    cv::waitKey( 0.01 );
    #endif
    
    //std::cout << "After path: " << path.size() << std::endl;
    return true;

  }

}

