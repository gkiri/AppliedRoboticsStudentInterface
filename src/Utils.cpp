#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <vector>
#include <math.h>
#include "Utils.hpp"

double SafeAcos (double x){
  if (x < -1.0) x = -1.0 ;
  else if (x > 1.0) x = 1.0 ;
  return acos (x) ;
}

triangle_angles compute_triangle_angles(Point A,Point B, Point C){    
    double AB,AC,BC;
    double CAB,ABC,BCA;
    //euclidean distances between points to the second power
    AB = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
    AC = sqrt(pow(A.x - C.x, 2) + pow(A.y - C.y, 2));
    BC = sqrt(pow(B.x - C.x, 2) + pow(B.y - C.y, 2));
    //std::cout << "AB,AC,BC: " << AB << "," << AC << "," << BC << std::endl;
    //angles between points
    CAB = SafeAcos((AC*AC + AB*AB - BC*BC)/(2*AB*AC));
    ABC = SafeAcos((AB*AB + BC*BC - AC*AC)/(2*AB*BC));
    BCA = SafeAcos((AC*AC + BC*BC - AB*AB)/(2*AC*BC));

    return {CAB,ABC,BCA};    
}


void compute_heading_angle(double* qs,double* qm,double* qe){
  double RAD2DEG = 180.0/M_PI;
  double alpha, d_s_e;
  triangle_angles triang;
  int quadrant;

  //Calculate beta1, beta2
  //triang = compute_triangle_angles(Point(qs[0],qs[1]), Point(qm[0],qm[1]), Point(qe[0],qe[1]));
  //std::cout << "Test for heading angle: " << triang.beta_1 << "," << triang.beta_2 << "," << triang.beta_3 << std::endl;

  //Calculate alpha
  d_s_e = sqrt(pow(qs[0]-qe[0],2) + pow(qs[1]-qe[1],2)); //euclidean distance
  alpha = SafeAcos(sqrt(pow(qs[0]-qe[0],2))/d_s_e); //angle btw start and end
  //std::cout << "ALPHA: " << alpha*RAD2DEG << std::endl;
  //Check for quadrants
  quadrant = compute_quadrant(Point(qe[0],qe[1]), Point(qs[0],qs[1]));
  switch (quadrant){
  case 1:
    //nothing
    break;
  case 2:
    alpha = M_PI - alpha;
    break;
  case 3:
    alpha = M_PI + alpha;
    break;
  case 4:
    alpha = -alpha;
    break;
  
  default:
  printf("ERROR in compute heading angle");
    break;
  }

  //save heading angle of mid point
  qm[2] = alpha;
  //std::cout << "HEADING ANGLE: " << alpha*RAD2DEG << std::endl;
} 


int compute_quadrant(Point pt, Point ref){
  double d_x, d_y;
  int quadrant = 0;

  d_x = pt.x - ref.x;
  d_y = pt.y - ref.y;

  if(d_x > 0 && d_y >= 0){ //1st quadrant
    quadrant = 1;
  }
  else if(d_x <= 0 && d_y > 0){ //2nd quadrant
    quadrant = 2;
  }
  else if(d_x < 0 && d_y <= 0){ //3rd quadrant
    quadrant = 3;
  }
  else if(d_x >= 0 && d_y < 0){ //4th quadrant
    quadrant = 4;
  }  

  return quadrant;
}


point_t Point_to_point_t(Point Pt){
  point_t pt_t;
  pt_t.push_back(Pt.x);
  pt_t.push_back(Pt.y);
  
  return pt_t;
}


Point point_t_to_Point(point_t pt_t){
  Point Pt;
  Pt.x = pt_t[0];
  Pt.y = pt_t[1];

  return Pt;
}

// bool same_point(Point pt1, Point pt2){
//   float epsilon = 0.00001;
//   return std::fabs(pt1.x - pt2.x) < epsilon && std::fabs(pt1.y - pt2.y) < epsilon? true:false;
// }  

bool same_point(point_t pt1, point_t pt2){
  float epsilon = 0.00001;
  return std::fabs(pt1[0] - pt2[0]) < epsilon && std::fabs(pt1[1] - pt2[1]) < epsilon? true:false;
}


void create_three_seg(struct arc_extract three_seg[3], double x0, double y0, DubinsCurve dubins_path){
  for(int i=0;i<3;i++){
    //Save length
    three_seg[i].length = dubins_path.arcs[i].L;
    //Save start/end point
    if(i==0){ //first dubins segment
      three_seg[i].start_point = Point(x0,y0);
      three_seg[i].end_point = Point(dubins_path.arcs[i].xf, dubins_path.arcs[i].yf);
    }
    else{
      // start point = end point of previous segment
      three_seg[i].start_point = Point(dubins_path.arcs[i-1].xf, dubins_path.arcs[i-1].yf); 
      three_seg[i].end_point = Point(dubins_path.arcs[i].xf, dubins_path.arcs[i].yf);
    }
    //Save LSR, radius and center
    if(dubins_path.arcs[i].k > 0){ //left cruve
      three_seg[i].LSR = 0;
      three_seg[i].radius = fabs(1/dubins_path.arcs[i].k);
      three_seg[i].center = compute_center(three_seg[i].start_point, three_seg[i].end_point,
                              three_seg[i].radius, three_seg[i].length, three_seg[i].LSR);
    }
    else if(dubins_path.arcs[i].k == 0){ //straight line
      three_seg[i].LSR = 1;
      three_seg[i].radius = 0;
      three_seg[i].center = Point(0,0);
    }
    else if(dubins_path.arcs[i].k < 0){ //right curve
      three_seg[i].LSR = 2;
      three_seg[i].radius = fabs(1/dubins_path.arcs[i].k);
      three_seg[i].center = compute_center(three_seg[i].start_point, three_seg[i].end_point,
                              three_seg[i].radius, three_seg[i].length, three_seg[i].LSR);
    }
    else{
      printf("Utils.cpp: Unknown LSR\n");
    }    
  }
}

void concatenate_dubins_path(Path& path, DubinsCurve dubins_path, double discritizer_size){
  //variables
  double updated_s, next_s;  
  next_s = path.size()*discritizer_size;

  for(int j = 0; j < dubins_path.discretized_curve.size(); j++){    
    updated_s = next_s + j*discritizer_size; //keep counting from last s with step size equal to the discretezer

    //concatenate paths
    path.points.emplace_back(updated_s, dubins_path.discretized_curve[j].xf, 
          dubins_path.discretized_curve[j].yf, dubins_path.discretized_curve[j].thf, 
          dubins_path.discretized_curve[j].k);
  }
}


Point compute_center(Point start,Point end,float radius,float length, int LSR){   
  //Check if angle between start and end is bigger than 180 degrees (M_PI rad)
  if((length/radius > M_PI)){ // swap start and end
      Point tmp = start;
      start = end;
      end = tmp;
  }

  //https://math.stackexchange.com/questions/27535/how-to-find-center-of-an-arc-given-start-point-end-point-radius-and-arc-direc

  int epsilon = 1; // Arc from start to end --> clockwise e=1 (LSR = 0), e=-1 counter-clockwise (LSR = 2)
  
  switch (LSR)
  {
  case 0: // counter-clockwise

      //epsilon = 1;
      break;
  case 1: // Straight line --> Center = 0
      return Point(0,0); 
      break;

  case 2: 
      epsilon = -1; 
      break;
      
  default:
      printf("unkown LSR");
      break;
  }    

  Point mid_point(0,0);
  mid_point.x=(start.x+end.x)/2.0 ;
  mid_point.y=(start.y+end.y)/2.0;
  //std::cout << "mid: " << mid_point.x << ", " << mid_point.y << std::endl;

  float distance;
  distance=sqrt(pow((end.x-start.x),2)+pow((end.y-start.y),2));
  if (distance <= 0){
      return Point(0,0);
  }
  //std::cout << "distance: " << distance << std::endl;

  /* 
  n(u,v) =unit normal in the direction z1 to z0
  n*(-v,u) =unit normal in the direction z0 to z1
  
  */
  Point normal_point(0,0);//
  normal_point.x=(end.x-start.x)/distance ;
  normal_point.y=(end.y-start.y)/distance ;
  //std::cout << "normal point module: " << normal_point.x << ", " << normal_point.y << std::endl;

  Point normal_pointstar(0,0);
  normal_pointstar.x= -normal_point.y;
  normal_pointstar.y= normal_point.x ;
  //std::cout << "normal point vector: " << normal_pointstar.x << ", " << normal_pointstar.y << std::endl;

  /*Let Distance H from midpoint to center */
  float h, h_round, sqrt_content;
  sqrt_content = pow(radius,2) - pow(distance,2)/4;   
  //h_round = std::ceil(sqrt_content * 100.0) / 100.0; //round to the 2nd decimal
  //std::cout << "sqrt_content: " << sqrt_content << std::endl;
  //std::cout << "h_round: " << h_round << std::endl;

  // if(sqrt_content < -0.0001) //residual threshold for r2-d2/4
  // {
  //     //std::cout << "Gkiri::Sqrt is NaN or 0 " << h << std::endl;
  //     sqrt_content =0;
  // }else if(sqrt_content < 0.0 && sqrt_content > -0.0001)
  // {
  //     //std::cout << "Gkiri::Sqrt is NaN or 0 " << h << std::endl;
  // }

  if(sqrt_content > -0.0001 && sqrt_content < 0.0)
  {
    sqrt_content =0;
    //std::cout << "Gkiri::Sqrt is NaN or 0 " << h << std::endl;
  } 

  h=sqrt(sqrt_content);   
  //std::cout << "h: " << h << std::endl;

  //c=𝐦+𝜖 ℎ 𝐧∗ 
  Point center;
  center.x=mid_point.x + epsilon*h*normal_pointstar.x;
  center.y=mid_point.y + epsilon*h*normal_pointstar.y;
  //std::cout << "center: " << center.x << ", " << center.y << std::endl;

  return center;
    
}


/* Calculate arc parameters for cv::ellipse function ---------------------------------*/
arc_param calculate_arc_drawing_angles(arc_extract arc){   
    
    double RAD2DEG = 180.0/M_PI;
    double start_angle, end_angle;
    double alpha, beta;
    Point s,e; //start and end point copies       

    //space
    //std::cout << "" << std::endl;    

    //start - end point
    s = arc.start_point;
    e = arc.end_point;

    //Calculate beta = Angle btw positive x-axis and start    
    alpha = atan2(s.y - arc.center.y, s.x - arc.center.x)*RAD2DEG;
    if(alpha > 0){
        alpha = 360 - alpha;
    }
    else{
        alpha = -alpha;
    }
    //std::cout << "ALPHA:"<< alpha << std::endl;

    //Calculate beta = Angle btw positive x-axis and start    
    beta = atan2(e.y - arc.center.y, e.x - arc.center.x)*RAD2DEG;
    if(beta > 0){
        beta = 360 - beta;
    }
    else{
        beta = -beta;
    }
    //std::cout << "BETA:"<< beta << std::endl;      

    //Check for turn (Left or Right)
    if(alpha < beta){
        if(arc.LSR == 0){ //Left
            start_angle = beta;
            end_angle = alpha + 360;
        }
        else if(arc.LSR == 2){//Right
            start_angle = alpha;
            end_angle = beta;            
        }
    }
    if(alpha > beta){
        if(arc.LSR == 0){ //Left
            // start_angle = alpha;
            // end_angle = beta;
            start_angle = beta;
            end_angle = alpha;
        }
        else if(arc.LSR == 2){//Right
            start_angle = alpha;
            end_angle = beta + 360;
        }
    }    

    //std::cout << "START,END ANGLE:" << start_angle << "," << end_angle << std::endl;
    
    arc_param result = {start_angle, end_angle};

    return result; 
}    

Point get_polygon_centroid(Polygon poly){
  //Variables
  double centroid_x, centroid_y, n_points;
  //Init values
  centroid_x = 0;
  centroid_y = 0;
  n_points = poly.size();

  for (int i=0;i<n_points;i++){    
    centroid_x += poly[i].x;
    centroid_y += poly[i].y;
    //std::cout << "Centroid sum: " << centroid_x << ", " << centroid_y << std::endl;   
  }
  Point centroid = Point(centroid_x/n_points, centroid_y/n_points);
  //std::cout << "Centroid: " << centroid_x/n_points << ", " << centroid_y/n_points << std::endl;

  return centroid;
}

//https://www.walletfox.com/course/parseconfigfile.php
double load_config_param(std::string config_dir, std::string param_name){
  double param;
  std::ifstream cFile;
  cFile.open(config_dir);
  if (cFile.is_open())
  {
    std::string line;
    while(getline(cFile, line)){
      line.erase(std::remove_if(line.begin(), line.end(), isspace),line.end());
      if(line[0] == '#' || line.empty())
          continue;
      auto delimiterPos = line.find("=");
      auto name = line.substr(0, delimiterPos);
      auto value = line.substr(delimiterPos + 1);
      //std::cout << name << " " << value << '\n';
      if(param_name == name){
        param = std::stod(value);

        return param;
      }
    }
    std::cerr << "Couldn't find the parameter\n";

    return 0;        
  }
  else {
    std::cerr << "Couldn't open config file for reading.\n";
    return 0;
  }  
}

