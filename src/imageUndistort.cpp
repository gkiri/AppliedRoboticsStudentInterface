 #include "image_undistort.hpp"
  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, 
          const std::string& config_folder){
    
    static ImageUndistort image_undistort;

    if(!image_undistort.isInitialized()){
      image_undistort.initialize(img_in.size(), cam_matrix, dist_coeffs);
    }

    image_undistort.undistort(img_in, img_out);

  }