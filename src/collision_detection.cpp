
#include <collision_detection.hpp>
#define ARC_DEBUG 1
#define RAD2DEG 180.0/M_PI
#define COLLISION_DEBUG 1

//  Determines the intersection point of the line defined by points A and B with the
//  line defined by points C and D.
//
//  Returns YES if the intersection point was found, and stores that point in X,Y.
//  Returns NO if there is no determinable intersection point, in which case X,Y will
//  be unmodified.
#define EPS 1e-9

bool isIntersecting(Point& p1, Point& p2, Point& q1, Point& q2) {

    return (((q1.x-p1.x)*(p2.y-p1.y) - (q1.y-p1.y)*(p2.x-p1.x))
            * ((q2.x-p1.x)*(p2.y-p1.y) - (q2.y-p1.y)*(p2.x-p1.x)) < 0)
            &&
           (((p1.x-q1.x)*(q2.y-q1.y) - (p1.y-q1.y)*(q2.x-q1.x))
            * ((p2.x-q1.x)*(q2.y-q1.y) - (p2.y-q1.y)*(q2.x-q1.x)) < 0);
}

bool linelineIntersection(Point A,Point B,Point C,Point D,Point *X)
{
    bool intersection;
    intersection=isIntersecting(A,B,C,D);

    return intersection;
}

// radius of circle // first parameter of line ax+by+c // second parameter of line ax+by+c
// third parameter of line ax+by+c
int linecircleIntersection(double r, double a , double b, double c ,std::vector<Point>& X)
{
    Point Temp;
    double x0 = -a*c/(a*a+b*b), y0 = -b*c/(a*a+b*b);
    if (c*c > r*r*(a*a+b*b)+EPS){
        puts ("no points");
        return 0;//no intersection
    }
    else if (abs (c*c - r*r*(a*a+b*b)) < EPS) {
          puts ("1 point");
          //cout << x0 << ' ' << y0 << '\n';
          return 1;//Tangent 
    }
    else {
      double d = r*r - c*c/(a*a+b*b);
      double mult = sqrt (d / (a*a+b*b));
      double ax, ay, bx, by;
      ax = x0 + b * mult;
      bx = x0 - b * mult;
      ay = y0 - a * mult;
      by = y0 + a * mult;

      Temp.x=ax;
      Temp.y=ay;
      X.push_back(Temp);

      Temp.x=bx;
      Temp.y=by;
      X.push_back(Temp);

      puts ("2 points");
      //cout << ax << ' ' << ay << '\n' << bx << ' ' << by << '\n';
      return 2;// collision and intersection point
    }

}

// Find the points of intersection. 
int FindLineCircleIntersections(Point point1, Point point2,Point center, double radius,Point& intersection1,Point& intersection2)
{
    double dx, dy, A, B, C, det, t;

    dx = point2.x - point1.x;
    dy = point2.y - point1.y;

    A = dx * dx + dy * dy;
    B = 2 * (dx * (point1.x - center.x) + dy * (point1.y - center.y));
    C = (point1.x- center.x) * (point1.x - center.x) +(point1.y - center.y) * (point1.y - center.y) - radius * radius;

    det = B * B - 4 * A * C;
    if ((A <= 0.0000001) || (det < 0))
    {
        // No real solutions.
        intersection1 = Point(-1, -1);
        intersection2 = Point(-1, -1);
        return 0;
    }
    else if (det == 0)
    {
        // One solution.
        t = -B / (2 * A);
        intersection1 =Point(point1.x + t * dx, point1.y + t * dy);
        intersection2 =Point(-1, -1);
        return 1;
    }
    else
    {
        // Two solutions.
        t = (double)((-B + sqrt(det)) / (2 * A));
        intersection1 =Point(point1.x + t * dx, point1.y + t * dy);
        t = (double)((-B - sqrt(det)) / (2 * A));
        intersection2 =Point(point1.x + t * dx, point1.y + t * dy);
        return 2;
    }

}


bool lineArcIntersection_prof(struct arc_extract line,struct arc_extract arc,std::vector<Point>& intersect_points)
{

    Point q1,q2,q3,intersection1,intersection2;
    int ret_val;

    struct arc_param test_param;

    /*Find Circle new method */
    ret_val=FindLineCircleIntersections(line.start_point,line.end_point,arc.center,arc.radius,intersection1,intersection2);
    //std::cout <<"Gkiri:FindLineCircle**  intersect1.x " << intersection1.x <<" intersect1.y " << intersection1.y << std::endl;
    //std::cout <<"Gkiri:FindLineCircle** intersect2.x " << intersection2.x <<" intersect2.y " << intersection2.y << std::endl;
    intersect_points.push_back(intersection1);
    intersect_points.push_back(intersection2);

    test_param=calculate_arc_drawing_angles(arc);

    if(ret_val==0){

        std::cout <<"Gkiri:lineArcIntersection No intersection "  << std::endl;
        return false;

    }

    if(ret_val==1 ){
        double beta = atan2(intersection1.y - arc.center.y, intersection1.x - arc.center.x)*RAD2DEG;

        if(beta > 0){
            beta = 360 - beta;
        }
        else{
            beta = -beta;
        }

        std::cout <<"Gkiri:lineArcIntersection prof =beta angle " << beta << std::endl;

        if((beta > test_param.start_angle) && (beta < test_param.end_angle))
        {
                std::cout <<"Gkiri:lineArcIntersection prof Intersection ret_val==1 " << std::endl; 
                return true; 
        }
        else {
                std::cout <<"Gkiri:lineArcIntersection prof NO Intersection ret_val==1 " << std::endl;  
                return false;  
        }
    }

    if(ret_val==2){ //2 intersection

        double beta = atan2(intersection1.y - arc.center.y, intersection1.x - arc.center.x)*RAD2DEG;

        if(beta > 0){
            beta = 360 - beta;
        }
        else{
            beta = -beta;
        }

        std::cout <<"Gkiri:lineArcIntersection prof =beta angle " << beta << std::endl;

        if((beta > test_param.start_angle) && (beta < test_param.end_angle))
        {
                std::cout <<"Gkiri:lineArcIntersection prof Intersection ret_val==2 " << std::endl; 
                return true; 
        }
        else {
                std::cout <<"Gkiri:lineArcIntersection prof NO Intersection ret_val==2 " << std::endl;  
                //return false;  //check for 2nd point too
        }

        ///////////////////////////////2nd intersection///////////////////////////////
        double beta2 = atan2(intersection2.y - arc.center.y, intersection2.x - arc.center.x)*RAD2DEG;

        if(beta2 > 0){
            beta2 = 360 - beta2;
        }
        else{
            beta2 = -beta2;
        }

        std::cout <<"Gkiri:lineArcIntersection prof =beta angle " << beta2 << std::endl;

        if((beta2 > test_param.start_angle) && (beta2 < test_param.end_angle))
        {
                std::cout <<"Gkiri:lineArcIntersection prof Intersection2 " << std::endl; 
                return true;
        }
        else {
                std::cout <<"Gkiri:lineArcIntersection prof NO Intersection2 " << std::endl;
                return false;  

        }
    }

}



//https://stackoverflow.com/questions/9070752/getting-the-bounding-box-of-a-vector-of-points 

bool  Construct_Bounding_Box(Polygon& input , Polygon& output)
{

    //Note: xExtremes is point with Min x(xExtremes.first->x) and Max x(xExtremes.second->x)
    //Note: yExtremes is point with Min y and Max y

    auto xExtremes = std::minmax_element(input.begin(), input.end(),
                                     [](const Point& lhs, const Point& rhs) {
                                        return lhs.x < rhs.x;
                                     });

    auto yExtremes = std::minmax_element(input.begin(), input.end(),
                                     [](const Point& lhs, const Point& rhs) {
                                        return lhs.y < rhs.y;
                                     });
    //std::cout <<"Gkiri:Construct_Bounding_Box Min-Points.x " << xExtremes.first->x <<" Min-Points.y " << yExtremes.first->y << std::endl;
    //std::cout <<"Gkiri:Construct_Bounding_Box Max-Points.x " << xExtremes.second->x <<" Max-Points.y " << yExtremes.second->y << std::endl;

    /*Direct points */
    Point Top_Right,Bottom_Left;//caluclated above
    Top_Right.x=xExtremes.second->x;
    Top_Right.y= yExtremes.second->y;

    Bottom_Left.x=xExtremes.first->x;
    Bottom_Left.y=yExtremes.first->y ;

    /*Indirect points */
    Point Top_Left,Bottom_Right;
    Top_Left.x=xExtremes.first->x;
    Top_Left.y=yExtremes.second->y;

    Bottom_Right.x=xExtremes.second->x;
    Bottom_Right.y=yExtremes.first->y;

    output.push_back(Bottom_Left);//4 points in sequence
    output.push_back(Top_Left);
    output.push_back(Top_Right);
    output.push_back(Bottom_Right);
    
    return true;

}


/*construct bounding boxes for all obstacles */
void Build_All_Bounding_Box(std::vector<Polygon>& obstacle_list,std::vector<Polygon>& Box_list){

  Polygon input;
  Polygon output;

  for (size_t i = 0; i<obstacle_list.size(); i++){

        input=obstacle_list[i];
        Construct_Bounding_Box(input , output);
        Box_list.push_back(output);
        output.clear();//clear pushback of output vector ref
  }

}

bool  Process_Box_arc_check(std::vector<Polygon>& Box_list,struct arc_extract& segment)
{
    // bool intersection;
    // std::vector<Point> cal_points;
    // Point Line_start_point,Line_end_point,X;
    // int k=0;
    // for (size_t i = 0; i<Box_list.size(); i++){//no of boxes
        
    //     for (size_t j = 0; j<Box_list[i].size(); j++){ //4corners of each box

    //         if(j==3) {
    //             intersection=lineArcIntersection(Box_list[i][j],Box_list[i][0],segment.start_point,segment.end_point,segment.radius,segment.center,cal_points);
    //             if(intersection)
    //                 return true;

    //         }
    //         else{
    //             intersection=lineArcIntersection(Box_list[i][j],Box_list[i][j+1],segment.start_point,segment.end_point,segment.radius,segment.center,cal_points);
    //             if(intersection)
    //                 return true;
    //         }
            
    //     }//Inner forloop
    // }//Outer forloop

     return false;//No collision   

}

/*Box vs line */
bool  Process_Box_line_check(std::vector<Polygon>& Box_list,struct arc_extract& segment)
{
    bool intersection;
    Point Line_start_point,Line_end_point,X;
    int k=0;
    for (size_t i = 0; i<Box_list.size(); i++){//no of boxes
        
        for (size_t j = 0; j<Box_list[i].size(); j++){ //4corners of each box

            if(j==3) {
                intersection=linelineIntersection(segment.start_point,segment.end_point,Box_list[i][j],Box_list[i][0],&X);
                if(intersection)
                    return true;

            }
            else {
                intersection=linelineIntersection(segment.start_point,segment.end_point,Box_list[i][j],Box_list[i][j+1],&X);
                if(intersection)
                    return true;
            }
            
        }//Inner forloop
    }//Outer forloop

    return false;//No collision
    
}


/*Box vs line  for obstacle list*/
bool  Process_Box_line_check_obstacles(std::vector<Polygon>& obstacle_list,struct arc_extract& segment)
{
    bool intersection;
    Point Line_start_point,Line_end_point,X;
    int k=0;

    std::vector<Polygon> Box_list;
    Build_All_Bounding_Box(obstacle_list,Box_list);//gets lists of bounding boxes

    for (size_t i = 0; i<Box_list.size(); i++){//no of boxes
        
        for (size_t j = 0; j<Box_list[i].size(); j++){ //4corners of each box

            if(j==3) {
                intersection=linelineIntersection(segment.start_point,segment.end_point,Box_list[i][j],Box_list[i][0],&X);
                if(intersection)
                    return true;

            }
            else {
                intersection=linelineIntersection(segment.start_point,segment.end_point,Box_list[i][j],Box_list[i][j+1],&X);
                if(intersection)
                    return true;
            }
            
        }//Inner forloop
    }//Outer forloop

    return false;//No collision
    
}




/*High level box vs each arc of dubins check */
bool  Highlevel_Box_dubins_check(std::vector<Polygon>& obstacle_list,struct arc_extract *three_seg)
{
    bool Intersection;

    //threeseg lenght is 3
    for (size_t i = 0; i<3; i++){

        switch (three_seg[i].LSR)
        {
        case 0: // counter-clockwise
            
            Intersection=Global_Arc_check(obstacle_list,three_seg[i]);

            if(Intersection)
            {
                std::cout <<"Gkiri:Highlevel_Box_dubins_check ARC COLLISION  " << std::endl;
                return true;
            }
            else
            {
                std::cout <<"Gkiri:Highlevel_Box_dubins_check NO ARC COLLISION  " << std::endl;
                return false;
            }
                

            break;
        case 1: // Straight line --> Center = 0
            Intersection=Global_Line_check(obstacle_list,three_seg[i]);

            if(Intersection)
            {
                std::cout <<"Gkiri:Highlevel_Box_dubins_check ARC COLLISION  " << std::endl;
                return true;
            }
            else
            {
                std::cout <<"Gkiri:Highlevel_Box_dubins_check NO ARC COLLISION  " << std::endl;
                return false;
            }
            
            break;

        case 2: 
            Intersection=Global_Arc_check(obstacle_list,three_seg[i]);

            if(Intersection)
            {
                std::cout <<"Gkiri:Highlevel_Box_dubins_check ARC COLLISION  " << std::endl;
                return true;
            }
            else
            {
                std::cout <<"Gkiri:Highlevel_Box_dubins_check NO ARC COLLISION  " << std::endl;
                return false;
            }

            break;
            
        default:
            printf("unkown LSR");
            break;
        }  
    }//forloop


}

void construct_line_structure(arc_extract& line_data,Point Arc_Start ,Point Arc_End){

    line_data.start_point = Arc_Start;
    line_data.end_point = Arc_End;
    line_data.LSR = 1;//R= clockwise from start L= anticlockwise from start

}

/*Box vs ARC  for obstacle list*/
bool  Process_Box_arc_check_obstacles(std::vector<Polygon>& obstacle_list,struct arc_extract& arc)
{
    bool intersection;
    Point Line_start_point,Line_end_point,X;
    int k=0;
    arc_extract line_data;
    std::vector<Point> intersect_points;

    std::vector<Polygon> Box_list;
    Build_All_Bounding_Box(obstacle_list,Box_list);//gets lists of bounding boxes

    for (size_t i = 0; i<Box_list.size(); i++){//no of boxes
        
        for (size_t j = 0; j<Box_list[i].size(); j++){ //4corners of each box

            if(j==3) {
                construct_line_structure(line_data,Box_list[i][j],Box_list[i][0]);
                intersection=lineArcIntersection_prof(line_data,arc,intersect_points);
                if(intersection)
                    return true;
            }
            else {
                construct_line_structure(line_data,Box_list[i][j],Box_list[i][j+1]);
                intersection=lineArcIntersection_prof(line_data,arc ,intersect_points);
                if(intersection)
                    return true;
            }
            
        }//Inner forloop
    }//Outer forloop

    return false;//No collision
    
}

/*Narrow polygon check vs ARC  for obstacle list*/
bool narrow_polygon_obstacles_arc_check(std::vector<Polygon>& obstacle_list,struct arc_extract& arc)
{
    bool intersection;
    Point Line_start_point,Line_end_point,X;
    int k=0;
    arc_extract line_data;
    std::vector<Point> intersect_points;

    for (size_t i = 0; i<obstacle_list.size(); i++){//no of boxes
        
        for (size_t j = 0; j<obstacle_list[i].size(); j++){ //4corners of each box

            if(j==obstacle_list[i].size()-1) {
                construct_line_structure(line_data,obstacle_list[i][j],obstacle_list[i][0]);
                intersection=lineArcIntersection_prof(line_data,arc,intersect_points);
                if(intersection)
                    return true;
            }
            else {
                construct_line_structure(line_data,obstacle_list[i][j],obstacle_list[i][j+1]);
                intersection=lineArcIntersection_prof(line_data,arc ,intersect_points);
                if(intersection)
                    return true;
            }
            
        }//Inner forloop
    }//Outer forloop

    return false;//No collision
}


/*Narrow polygon vs line  for obstacle list*/
bool  narrow_polygon_obstacles_line_check(std::vector<Polygon>& obstacle_list,struct arc_extract& segment)
{
    bool intersection;
    Point Line_start_point,Line_end_point,X;
    int k=0;

    for (size_t i = 0; i<obstacle_list.size(); i++){//no of boxes
        
        for (size_t j = 0; j<obstacle_list[i].size(); j++){ //4corners of each box

            if(j==obstacle_list[i].size()-1) {
                intersection=linelineIntersection(segment.start_point,segment.end_point,obstacle_list[i][j],obstacle_list[i][0],&X);
                if(intersection)
                    return true;

            }
            else {
                intersection=linelineIntersection(segment.start_point,segment.end_point,obstacle_list[i][j],obstacle_list[i][j+1],&X);
                if(intersection)
                    return true;
            }
            
        }//Inner forloop
    }//Outer forloop

    return false;//No collision 
}

//Global LINE check which includes Broad phase and Narrow Phase
bool Global_Line_check(std::vector<Polygon>& obstacle_list,struct arc_extract& segment)
{
    bool line_collision,narrow_collision;
    line_collision = Process_Box_line_check_obstacles(obstacle_list,segment);
    if(!line_collision){

        #if COLLISION_DEBUG
        std::cout <<"Gkiri:Global_Line_check BOX is Not colliding " << std::endl;
        #endif

        return false;
    }   
    else {

        #if COLLISION_DEBUG
        std::cout <<"Gkiri:Global_Line_check BOX is colliding " << std::endl;
        #endif

        narrow_collision = narrow_polygon_obstacles_line_check(obstacle_list,segment);
        if(narrow_collision){

            #if COLLISION_DEBUG
            std::cout <<"Gkiri:Global_Line_check NARROW polygon is also colliding " << std::endl;
            #endif

            return true;
        }
        else{

            #if COLLISION_DEBUG
            std::cout <<"Gkiri:Global_Line_check NARROW polygon is not colliding " << std::endl;
            #endif

            return false;
        }
        
    }

}


//Global ARC check which includes Broad phase and Narrow Phase
bool Global_Arc_check(std::vector<Polygon>& obstacle_list,struct arc_extract& segment)
{
    bool arc_collision,narrow_collision;
    arc_collision = Process_Box_arc_check_obstacles(obstacle_list,segment);
    if(!arc_collision){

        #if COLLISION_DEBUG
        std::cout <<"Gkiri:Global_Arc_check BOX is Not colliding " << std::endl;
        #endif

        return false;
    }   
    else {

        #if COLLISION_DEBUG
        std::cout <<"Gkiri:Global_Arc_check BOX is colliding " << std::endl;
        #endif

        narrow_collision = narrow_polygon_obstacles_arc_check(obstacle_list,segment);
        if(narrow_collision){

            #if COLLISION_DEBUG
            std::cout <<"Gkiri:Global_Arc_check NARROW polygon is also colliding " << std::endl;
            #endif

            return true;
        }
        else{

            #if COLLISION_DEBUG
            std::cout <<"Gkiri:Global_Arc_check NARROW polygon is not colliding " << std::endl;
            #endif

            return false;
        }
        
    }

}