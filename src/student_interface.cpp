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

#include <graphics.h>

#include "inflate_polygons.cpp"

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

    std::cout << "Gkiri:: findRobot:: Started" << std::endl;
    findRobot_api(img_in, scale, triangle, x, y, theta, config_folder);


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

    //Inflate polygons
    double OFFSET = sqrt(pow(robot_length/2,2) + pow(robot_width/2,2)); 
                              //Radius of circle approximating robot shape (m)  
    std::vector<Polygon> inflated_obstacle_list;
    //Inflate polygon function --> Return std::vector<Polygon>
    inflated_obstacle_list = inflate_polygons(obstacle_list, OFFSET);  

    return 0;

  }


}

