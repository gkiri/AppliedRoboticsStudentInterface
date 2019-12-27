#include <cstdlib>
#include <iostream>
#include <cfloat>
#include <algorithm> 

#include "PRM.h"
#include "a_star.cpp"


PRM::PRM(std::vector<Polygon> polygons_list)
{
  for (size_t j = 0; j<polygons_list.size(); j++){  
      obstacle_list.push_back(polygons_list[j]);
  }

}


PRM::~PRM()
{

}


/* Draw Point in Polygon Test -------------------------------------------*/
bool PRM::point_liesin_polygon(Point pt,std::vector<Polygon> cv_poly_list)
{
    std::vector<cv::Point> contour;
    Polygon obstacle;
    cv::Point cv_point_temp;

    cv::Point2f test_pt;
	  test_pt.x = pt.x*100*720/150;
	  test_pt.y = pt.y*100*576/100;

    for (size_t j = 0; j<cv_poly_list.size(); j++){  

        obstacle=cv_poly_list[j];
        contour.clear();//vector clearing for next test

        for (size_t k = 0; k<obstacle.size(); k++){  

            cv_point_temp.x=obstacle[k].x*100*720/150;//img_map_w*x_ob/map_w
            cv_point_temp.y=obstacle[k].y*100*576/100;
            contour.push_back(cv_point_temp);

        }//inner for loop

        int result=cv::pointPolygonTest(contour, test_pt, false);
        double dist=cv::pointPolygonTest(contour, test_pt, true);

        switch(result)
        {
            case 1:  //1 = (point lies inside polygon)

                //std::cout << "Gkiri:case 111111111 Inside polygon result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y<<  std::endl;
                return true;
                break;
            case 0:  //0 (point lies on edge of polygon)

                //std::cout << "Gkiri:case 00000000 Inside polygon result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y<<  std::endl;
                return true;
                break;
            case -1:   //-1 (point lies outside polygon)

                //std::cout << "Gkiri:case -1-1-1-1-1-1-1 Inside polygon result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y<<  std::endl;
                if((dist< 0.0 && dist > -10.0))
                {
                    return true;//Point outside polygon but bit closer to edge so remove this with some threshold distance   
                }
                break;
            default:
                //std::cout << "ERROR from function" << std::endl ;
                break;
        } 

       // std::cout << "Gkiri:point_polygon Outside polygon -----------------result= " << result << "measdistance= " << dist << "test point (x,y)= "<<test_pt.x << " , " << test_pt.y <<  std::endl;
        
    }//outer loop

    return false;//false = point doesnot lie in polygon

}


void PRM::generate_random_points(double cspace_length,double cspace_width, int N)
{

    std::cout <<"Random Sampling in Map start st" <<  std::endl;
    Point test_pt;
    int count=0;  

    /* Generate random pointst */
    for(int i=0;i<N;i++){
        float x_rand = (rand() / (double) RAND_MAX) * cspace_length; //Generate random sample
        float y_rand = (rand() / (double) RAND_MAX) * cspace_width;

        std::cout <<"Random Sampling in Map" << x_rand << "," << y_rand << std::endl;

        test_pt.x=x_rand;
        test_pt.y=y_rand;

        if(!point_liesin_polygon(test_pt ,  obstacle_list)){ //Global obstacle list
            //draw_point(random_points[z], *map_param); 
            free_space_points.emplace_back(test_pt);
            count++;
            std::cout << "Gkiri :: Motion sample After count= " << count << std::endl; 
        }

    }//end for loop

}

void PRM::global_planner(Point start,Point goal){
    //input --> std::pair<Point, std::pair<Point ,Point > > prm_graph;
    //output --> std::vector<Point> global_planner_path;

    //Call astar
    global_planner_path = globalplanner::astar(prm_graph, start, goal);

}

//@Ambike
void PRM::local_planner(){
  set_prm_graph(std::pair<Point, std::vector<Point> >> free_space_points);
  //std::vector<Point> free_space_points;
  //Point n_pt;
  //std::vector<std::pair<Point, std::vector<Point> >> prm_graph;  
  //prm_graph = n_pt, free_space_points.emplace_back(n_pt);
  //std::vector<std::pair<n_pt, free_space_points> >>prm_graph ;
  //set_prm_graph(prm_graph);
}


/*Returns for unit tests --------------------------------------*/
std::vector<Point> PRM::get_free_space_points()
{
    return free_space_points;
}

std::vector<Point> PRM::get_global_planner_path(){
    return global_planner_path;
}

//@Ambike
std::vector<std::pair<Point, std::vector<Point> >> PRM::get_prm_graph(){
    return prm_graph;
}

/*Set for unit tests---------------------------------------------*/
void PRM::set_prm_graph(std::vector<std::pair<Point, std::vector<Point> >> prm_graph_test){
    prm_graph = prm_graph_test;
}




