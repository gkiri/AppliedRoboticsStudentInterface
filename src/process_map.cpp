#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <opencv2/opencv.hpp>

const double MIN_AREA_SIZE = 100;

//std::string template_folder = "../imgs/template/";
#define MIN_AREA_SIZE  500

#define TEMPLATE_DEBUG 0
  //-------------------------------------------------------------------------
  //          PROCESS MAP
  //-------------------------------------------------------------------------
bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){
    
    // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
    cv::Mat red_mask_low, red_mask_high, red_mask;     
    cv::inRange(hsv_img, cv::Scalar(0, 102, 86), cv::Scalar(40, 255, 255), red_mask_low);
    cv::inRange(hsv_img, cv::Scalar(164, 102, 86), cv::Scalar(180, 255, 255), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); 

    // cv::Mat img_small;
    // cv::resize(red_mask, img_small, cv::Size(640, 512));

    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    // Process red mask
    //contours_img = img_in.clone();
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //std::cout << "N. contours: " << contours.size() << std::endl;
    for (int i=0; i<contours.size(); ++i)
    {
      //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      approxPolyDP(contours[i], approx_curve, 3, true);

      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      obstacle_list.push_back(scaled_contour);
      //contours_approx = {approx_curve};
      //drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
      //std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
    }
    //std::cout << std::endl;
    // cv::imshow("Original", contours_img);
    // cv::waitKey(1);

    return true;
  }



bool processGate(const cv::Mat& hsv_img, const double scale, Polygon& gate){
    
    // Find purple regions
    cv::Mat purple_mask;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), purple_mask);    
    //cv::inRange(hsv_img, cv::Scalar(130, 10, 10), cv::Scalar(165, 255, 255), purple_mask);
    
    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    //cv::Mat contours_img;

    // Process purple mask
    //contours_img = hsv_img.clone();
    cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
    // std::cout << "N. contours: " << contours.size() << std::endl;

    
    bool res = false;

    for( auto& contour : contours){
      const double area = cv::contourArea(contour);
      //std::cout << "AREA " << area << std::endl;
      //std::cout << "SIZE: " << contours.size() << std::endl;
      if (area > 500){
        approxPolyDP(contour, approx_curve, 30, true);

        if(approx_curve.size()!=4) continue;

        // contours_approx = {approx_curve};
        // drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);


        for (const auto& pt: approx_curve) {
          gate.emplace_back(pt.x/scale, pt.y/scale);
        }
        res = true;
        break;
      }      
    }


    // cv::imshow("Original", contours_img);
    // cv::waitKey(1);
    
    return res;
  }


bool processVictims(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list){
    
    // Find green regions
    cv::Mat green_mask;
     
    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask);


    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    //cv::Mat contours_img;

    // Process red mask
    //contours_img = hsv_img.clone();
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //std::cout << "N. contours: " << contours.size() << std::endl;
    int victim_id = 0;
    for (int i=0; i<contours.size(); ++i)
    {

      const double area = cv::contourArea(contours[i]);

      if(area < 500) continue;

      //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      approxPolyDP(contours[i], approx_curve, 10, true);
      if(approx_curve.size() < 6) continue;

      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      victim_list.push_back({victim_id++, scaled_contour});
      //contours_approx = {approx_curve};
      //drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
      //std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
    }


    // cv::imshow("Original", contours_img);
    // cv::waitKey(1);
    
    return true;
  }



bool processVictims_student(const cv::Mat& img,const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list,const std::string& template_folder){
       
    // Find green regions
    cv::Mat green_mask;
    // store a binary image in green_mask where the white pixel are those contained in HSV rage (45,40,40) --> (75,255,255)  
    cv::inRange(hsv_img, cv::Scalar(45, 40, 40), cv::Scalar(75, 255, 255), green_mask);
    
    // Apply some filtering
    // Create the kernel of the filter i.e. a rectanble with dimension 3x3
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    // Dilate using the generated kernel
    cv::dilate(green_mask, green_mask, kernel);
    // Erode using the generated kernel
    cv::erode(green_mask,  green_mask, kernel);
    
    // Display the binary image
    #if TEMPLATE_DEBUG
    cv::imshow("GREEN_filter", green_mask);
    cv::waitKey(500);    //  wait 500 ms
    #endif
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours, contours_approx;  
    
    // Create an image which we can modify not changing the original image!
    cv::Mat contours_img;
    contours_img = img.clone();

    // Finds contours in a binary image.
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
        

        // Display the image 
    #if 0//TEMPLATE_DEBUG
    cv::imshow("Original", contours[0]);
    cv::waitKey(0);
    #endif

    // create an array of rectangle (i.e. bounding box containing the green area contour)  
    //std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Rect> boundRect;
    for (int i=0; i<contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives

        std::vector<cv::Point> approx_curve;
        approxPolyDP(contours[i], approx_curve, 10, true);
        if(approx_curve.size() < 6) continue; //fitler out the gate 

        /*****Store half part of victims*****/
        Polygon scaled_contour;
        for (const auto& pt: approx_curve) {
          scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
        }
        victim_list.push_back({-1, scaled_contour});
        ///////////////////////////////////////////////////////////

        contours_approx = {approx_curve};

        // Draw the contours on image with a line color of BGR=(0,170,220) and a width of 3
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

        // find the bounding box of the green blob approx curve
        boundRect.push_back(boundingRect(cv::Mat(approx_curve))); 
    }

    std::cout << "Green Blobs = : " <<boundRect.size() << std::endl; 

    // Display the image 
    #if TEMPLATE_DEBUG
    cv::imshow("Original", contours_img);
    cv::waitKey(0);
    #endif
        
    cv::Mat green_mask_inv;

    // Init a matrix specify its dimension (img.rows, img.cols), default color(255,255,255) 
    // and elemet type (CV_8UC3).
    cv::Mat filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255,255,255));

    // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
    cv::bitwise_not(green_mask, green_mask_inv); 

    // Show the inverted mask. The green area should be black here!
    #if TEMPLATE_DEBUG
    cv::imshow("Numbers", green_mask_inv);
    cv::waitKey(0);
    #endif
    
    // Load digits template images
    std::vector<cv::Mat> templROIs;
    for (int i=1; i<=5; ++i) {
        //std::cout << "Before Read folder: " << std::endl;   

        auto num_template = cv::imread(template_folder + std::to_string(i) + ".png");
        //std::cout << "After Read folder: "  << std::endl;   

        // mirror the template, we want them to have the same shape of the number that we
        // have in the unwarped ground image
        cv::flip(num_template, num_template, 1); 
    
        //Show the loaded template!
        #if TEMPLATE_DEBUG
        cv::imshow("Loaded template " + std::to_string(i) ,num_template);
        cv::waitKey(0);
        #endif

        // Store the template in templROIs (vector of mat)
        templROIs.emplace_back(num_template);
    }  
    
    img.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes
    

    // create a 3x3 recttangular kernel for img filtering
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
    
    std::cout << "Green Blobs with Digits only = : " << boundRect.size() << std::endl; 

    // For each green blob in the original image containing a digit
    for (int i=0; i<boundRect.size(); ++i)
    {
        // Constructor of mat, we pass the original image and the coordinate to copy and we obtain
        // an image pointing to that subimage
        cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit
        
        if (processROI.empty()) continue;
        
        // The size of the number in the Template image should be similar to the dimension
        // of the number in the ROI
        cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI 
        cv::threshold( processROI, processROI, 100, 255, 0 );   // threshold and binarize the image, to suppress some noise
        
        // Apply some additional smoothing and filtering
        cv::erode(processROI, processROI, kernel);
        cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
        cv::erode(processROI, processROI, kernel);
        
        // Show the actual image used for the template matching
        //cv::imshow("ROI", processROI);
        
        // Find the template digit with the best matching
        double maxScore = 0;
        int maxIdx = -1;    
        int angle_step = 15; //Angle of rotation
        int iImageHeight = processROI.size().height; 
        int iImageWidth = processROI.size().width;


        //Rotate the number
        for (int angle = 0; angle <= 360; angle += angle_step) {    
            //std::cout << "Angle value: " << angle << std::endl;   
            cv::Mat matRotation = cv::getRotationMatrix2D( cv::Point(iImageWidth/2, iImageHeight/2), (angle_step), 1 );
            cv::warpAffine(processROI, processROI, matRotation, processROI.size() );
            //Show actual number
            #if TEMPLATE_DEBUG
            cv::imshow("ROI", processROI);
            cv::waitKey(0);
            #endif

            for (int j=0; j<templROIs.size(); ++j) {
                cv::Mat result;

                //std::cout << "Before Match template : "  << std::endl;   

                // Match the ROI with the templROIs j-th
                cv::matchTemplate(processROI, templROIs[j], result, cv::TM_CCOEFF);
                //std::cout << "After  Match template: "  << std::endl;   

                double score;
                cv::minMaxLoc(result, nullptr, &score); 

                // Compare the score with the others, if it is higher save this as the best match!
                if (score > maxScore) {
                  maxScore = score;
                  maxIdx = j;
                }        
            }
      }    
      // Display the best fitting number
      std::cout << "Best fitting template: " << maxIdx + 1 << std::endl;    
      
      victim_list[i].first=maxIdx + 1;

      #if TEMPLATE_DEBUG
      cv::waitKey(0);
      #endif
    }

    return true;
}




//Centroid for Polygon vertices
Point compute2DPolygonCentroid(Polygon vertices)
{
    Point centroid = {0, 0};
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices
    int i=0;
    int vertexCount=vertices.size();

    for (i=0; i<vertexCount; ++i)
    {
        x0 = vertices[i].x;
        y0 = vertices[i].y;
        x1 = vertices[(i+1) % vertexCount].x;
        y1 = vertices[(i+1) % vertexCount].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);

    return centroid;
}





bool Gate_Process_second(const cv::Mat &img_in, cv::Mat &showImage, const double scale, Polygon &gate, bool arena ) {
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    // Find purple regions
    cv::Mat green_mask;

    if(!arena) { //Per simulatore
        cv::inRange(hsv_img, cv::Scalar(45, 50, 50), cv::Scalar(75, 255, 255), green_mask);
    }else{ //Arena
        cv::inRange(hsv_img, cv::Scalar(45, 40, 60), cv::Scalar(80, 255, 180), green_mask);
    }

    //cv::imshow("Gate", green_mask);
    //cv::waitKey(20);

    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    //cv::Mat contours_img;

    // Process purple mask
    //contours_img = hsv_img.clone();
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
    // std::cout << "N. contours: " << contours.size() << std::endl;

    bool res = false;

    for (auto &contour : contours) {
        const double area = cv::contourArea(contour);
        for (int i = 5; i < 10; i++) {
            approxPolyDP(contour, approx_curve, i, true);

            if ((area>500.0)&&(approx_curve.size() == 4) ){
                contours_approx = {approx_curve};
                drawContours(showImage, contours_approx, -1, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);

                for (const auto &pt: approx_curve) {
                    gate.emplace_back(pt.x / scale, pt.y / scale);
                }

                return true;
            }
        }
    }

    return res;
}
