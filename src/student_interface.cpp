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

//Unit test and printouts variables
#define PRINTOUT 0        //Print out polygons (0 -no, 1 -yes)
#define PRINTOUT_ALL 1   //(0)Print 1by1 - (1)Print all polygons
#define VISUALIZE_MAP 1   //(0)Deactivated - (1)Visualize elements in map
#define DRAW_TEST 1       //(0)Deactivated - (1)Draw function test

#define DUBINS_CURVE 1

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
    //throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );  
    //cv::Mat dst_mat;
    //cv::cvtColor(img_in, dst_mat, COLOR_BGR2HSV);
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
    //std::cout << "Gkiri::planPath " << std::endl;

    //Robot parameters (move to another file?)
    const double robot_length = 0.2; //(m)
    const double robot_width = 0.14; //(m)

    //Visualising the map parameters
    double map_w = 1.5; //real map width in m
    double map_h = 1; //real map height in m  
    double img_map_w = 720; //image map width in pixels
    //image map height is calculated through the other three parameters

    //Inflate polygons
    double OFFSET = sqrt(pow(robot_length/2,2) + pow(robot_width/2,2)); 
                              //Radius of circle approximating robot shape (m)  
    std::vector<Polygon> inflated_obstacle_list;
    //Inflate polygon function --> Return std::vector<Polygon>
    inflated_obstacle_list = inflate_polygons(obstacle_list, OFFSET);

    //Print polygons
    #if PRINTOUT 
    print_polygons_out(obstacle_list, inflated_obstacle_list);
    #endif      

    #if VISUALIZE_MAP
    //Visualize elements in a map image
    //Initialize map matrix and scale
    img_map_def map_param = initialize_img_map(map_w, map_h, img_map_w);
    //std::cout << "Scale: " << map_param.scale << std::endl;

    //Colours of objects represented by a point.
    //By default samples are white and robot is blue.
    cv::Scalar victim_colour(0,255,0); //green    
    

    #if DRAW_TEST   

    //Draw polygon
    Polygon poly;
    // //Code for drawing a single polygon
    // poly = obstacle_list[0];
    // draw_polygon(poly, map_param);

    //Code for printing all polygons
    std::vector<Polygon> poly_list = obstacle_list; //for printing the original polygons
    //std::vector<Polygon> poly_list = inflated_obstacle_list; //inflated polygons
    
    for (size_t i = 0; i<poly_list.size(); i++){
      poly = poly_list[i];
      draw_polygon(poly, map_param);
    }

    
    //Code for single point
    Point eg_point;
    eg_point.x = 0.75;
    eg_point.y = 0.5;
    draw_point(eg_point, map_param);

    // //Generate random pointst
    // std::vector<Point> eg_points;    
    // for(int i=0;i<1000;i++){
    //   int x_rand = rand() % 150 + 1; //Generate random sample
    //   int y_rand = rand() % 100 + 1; 
    //   //std::cout << x_rand << "," << y_rand << std::endl;
    //   eg_points.emplace_back(x_rand,y_rand);
    // }    
    // //Add points to map image    
    // for (int i=0;i<1000;i++){
    //   draw_point(eg_points[i], map_param);      
    // }


    //Draw segment

    /* Drawing semicircle-------------------------------------------*/
    arc_extract semi_circle;
    semi_circle.start_point = Point(0.1,0.5);
    semi_circle.end_point = Point(0.3,0.5);
    semi_circle.radius = 0.1;
    semi_circle.center = Point(0.2,0.5);
    semi_circle.length = 15;  

    draw_arc(semi_circle, map_param);
    arc_param semicircle_param;
    semicircle_param = calculate_arc_drawing_angles(semi_circle);
    std::cout << "SemiCircle Rotation angle: " << semicircle_param.rotation_angle << std::endl;
    std::cout << "SemiCircle Angle btw cs & ce: " << semicircle_param.angle_cs_ce << std::endl;

    /* Drawing arc-------------------------------------------*/
    arc_extract arc;
    arc.radius = 0.2;
    arc.start_point = Point(arc.center.x - arc.radius, arc.center.y);
    arc.end_point = Point(arc.center.x, arc.center.y + arc.radius);
    
    arc.center = Point(0.9,0.3);
    arc.length = 15;  

    draw_arc(arc, map_param);
    arc_param arc_param;
    arc_param = calculate_arc_drawing_angles(arc);
    std::cout << "Arc Rotation angle: " << arc_param.rotation_angle << std::endl;
    std::cout << "Arc Angle btw cs & ce: " << arc_param.angle_cs_ce << std::endl;

   
    /* Drawing a line-----------------------------------------*/    
    Point pt1 = Point(0.15,0.15);
    Point pt2 = Point(0.90,0.90);
    line_extract line_test;
    //Transform pair of points into line_extract type
    line_test = to_line_extract_type(pt1,pt2,true);
    std::cout << "start_point: " << line_test.start_point.x << ", " 
              << line_test.start_point.y << std::endl;
    std::cout << "end_point: " << line_test.end_point.x << ", " 
              << line_test.end_point.y << std::endl;
    std::cout << "lenght: " << line_test.length << std::endl;

    //drawing line
    draw_segment(line_test,map_param);
    
    /* Drawing robot position-----------------------------------------*/
    draw_robot(x,y,theta,map_param);    

    /* Drawing victims position and number-----------------------------*/    
    for(int i=0;i<victim_list.size();i++){      
      draw_victim(victim_list[i], map_param);
    }    

    #endif

    //Show map image
    cv::imshow("Image",map_param.img_map);
    cv::waitKey( 0.01 );
    #endif

    /* Dubins Secion-------------------------------------------*/


    std::cout << "Before path: " << path.size() << std::endl;
    dubins_wrapper_api(path);
    std::cout << "After path: " << path.size() << std::endl;

    return true;

  }


}

