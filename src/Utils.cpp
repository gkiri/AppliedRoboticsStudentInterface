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

//Source: https://arxiv.org/pdf/1609.06662.pdf
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
  std::cout << "ALPHA: " << alpha*RAD2DEG << std::endl;
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
  std::cout << "HEADING ANGLE: " << alpha*RAD2DEG << std::endl;
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