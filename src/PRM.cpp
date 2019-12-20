#include <cstdlib>
#include <iostream>
#include <cfloat>
#include<algorithm> 

#include "PRM.h"

PRM::PRM(std::vector<Polygon>& polygons_list)
{
  // Make sure that the random generator has been initialize

  for (size_t j = 0; j<polygons_list.size(); j++){  
    obstacle_list.push_back(polygons_list[j]);
  }


}

PRM::~PRM()
{

}

// void PRM::generate_random_points(std::vector<Point> random_points,int size)
// {

//     std::cout <<"Random Sampling in Map start st" <<  std::endl;

//     /* Generate random pointst */
//     for(int i=0;i<size;i++){
//       float x_rand = (rand() / (double) RAND_MAX) * 1.50; //Generate random sample
//       float y_rand = (rand() / (double) RAND_MAX) * 1.00;

//       std::cout <<"Random Sampling in Map" << x_rand << "," << y_rand << std::endl;

//       random_points.emplace_back(x_rand,y_rand);
//     }

// }

// bool PRM::filter_samples_of_obstaclespace(Point p , std::vector<Polygon> inflated_obstacle_list)
// {
//     //https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon/17490923#17490923
//     bool isInside = false;
//      float minX ,maxX ,minY,maxY;

//     for (size_t i = 0; i<inflated_obstacle_list.size(); i++){

//       minX = inflated_obstacle_list[i][0].x, maxX = inflated_obstacle_list[i][0].x;
//       minY = inflated_obstacle_list[i][0].y, maxY = inflated_obstacle_list[i][0].y;
      
      
//       for (int n = 1; n < inflated_obstacle_list[i].size(); n++) {
//           Point q = inflated_obstacle_list[i][n];
//           minX = std::min(q.x, minX);
//           maxX = std::max(q.x, maxX);
//           minY = std::min(q.y, minY);
//           maxY = std::max(q.y, maxY);
//       }//inner forloop


//       if (p.x < minX || p.x > maxX || p.y < minY || p.y > maxY) {
//         return false;
//       }

//       for ( int u = 0, v = inflated_obstacle_list[u].size() - 1 ; u < inflated_obstacle_list[u].size() ; v = u++ )
//       {
//           if ( ( inflated_obstacle_list[i][u].y > p.y ) != ( inflated_obstacle_list[i][v].y > p.y ) && \
//            p.x < ( inflated_obstacle_list[i][v].x - inflated_obstacle_list[i][u].x ) * ( p.y - inflated_obstacle_list[i][u].y ) / ( inflated_obstacle_list[i][v].y - inflated_obstacle_list[i][u].y ) + inflated_obstacle_list[i][u].x )
//           {
//               isInside = !isInside;
//           }
//       }


//     }//outer forloop

//     return isInside;

// }