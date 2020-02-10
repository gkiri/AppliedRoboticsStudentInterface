
#include <collision_detection.hpp>
#define ARC_DEBUG 0
#define RAD2DEG 180.0/M_PI
#define COLLISION_DEBUG 0



#define RHO 0.1

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


////////////////////////////////////////////////////////////////////////////////////////////
double dist(double x1, double y1, double x2, double y2)
{
  double distX = x2 - x1;
  double distY = y2 - y1;
  double distance = sqrt( (distX*distX) + (distY*distY) );
  return distance;
}

// check if point is inside circle
bool pointCircle(double px, double py, double cx, double cy, double r) {

  // get distance between the point and circle's center
  // using the Pythagorean Theorem
  double distX = px - cx;
  double distY = py - cy;
  double distance = sqrt( (distX*distX) + (distY*distY) );

  // if the distance is less than the circle's
  // radius the point is inside!
  if (distance <= r) {
    return true;
  }
  return false;
}

// LINE/POINT
bool linePoint(double x1, double y1, double x2, double y2, double px, double py) {

  // get distance from the point to the two ends of the line
  double d1 = dist(px,py, x1,y1);
  double d2 = dist(px,py, x2,y2);

  // get the length of the line
  double lineLen = dist(x1,y1, x2,y2);

  // since floats are so minutely accurate, add
  // a little buffer zone that will give collision
  double buffer = 0.1;    // higher # = less accurate

  // if the two distances are equal to the line's
  // length, the point is on the line!
  // note we use the buffer here to give a range,
  // rather than one #
  if (d1+d2 >= lineLen-buffer && d1+d2 <= lineLen+buffer) {
    return true;
  }
  return false;

}


// LINE/CIRCLE
bool lineCircle(double x1, double y1, double x2, double y2, double cx, double cy, double r) {

  // is either end INSIDE the circle?
  // if so, return true immediately
  bool inside1 = pointCircle(x1,y1, cx,cy,r);
  bool inside2 = pointCircle(x2,y2, cx,cy,r);
  if (inside1 || inside2) return true;

  // get length of the line
  double distX = x1 - x2;
  double distY = y1 - y2;
  double len = sqrt( (distX*distX) + (distY*distY) );

  // get dot product of the line and circle
  double dot = ( ((cx-x1)*(x2-x1)) + ((cy-y1)*(y2-y1)) ) / pow(len,2);

  // find the closest point on the line
  double closestX = x1 + (dot * (x2-x1));
  double closestY = y1 + (dot * (y2-y1));

  // is this point actually on the line segment?
  // if so keep going, but if not, return false
  bool onSegment = linePoint(x1,y1,x2,y2, closestX,closestY);
  if (!onSegment) return false;

  // get distance to closest point
  distX = closestX - cx;
  distY = closestY - cy;
  double distance = sqrt( (distX*distX) + (distY*distY) );

  if (distance <= r) {
    return true;
  }
  return false;
}


/* Line circle just collision detection*/
bool is_line_colliding_circle(Point linestart, Point lineend, Point center, double radius)
{
    bool colliding;

    colliding = lineCircle(linestart.x, linestart.y, lineend.x,lineend.y, center.x, center.y, radius);
    if(colliding)
    {
        return true;
    }
    else {

        return false;
    }

}

////////////////////////////////////////////////////////////////////////////////////////////

//COnsider vertical straight lines as special case and handle
int get_line_circle_intersection(Point linestart, Point lineend,Point center, double radius,Point& firstIntersection,Point& secondIntersection)
{
    float dx = lineend.x-linestart.x, dy = lineend.y-linestart.y;
    float l = sqrtf(dx*dx+dy*dy); // Length of line
    float dxl = dx/l, dyl = dy/l;
    float t = dxl*(center.x-linestart.x) + dyl*(center.y-linestart.y); // Projection of circle center on line

    float ex = t*dxl + linestart.x, ey = t*dyl + linestart.y; // Coordinates of e on line and closest to circle center
    float lec = sqrtf((ex-center.x)*(ex-center.x) + (ey-center.y)*(ey-center.y)); // Distance e to circle center
   
    //std::cout <<"Gkiri:get_line_circle_intersection  lec " << lec <<"RHO " << RHO  << "arc radius " << radius << std::endl;

    if (lec < RHO) { // Intersection
        float dt = sqrtf(RHO*RHO - lec*lec); // Distance to to circle intersection point
        int nbi = 0;
        if (t-dt >=0 && t-dt <= l) {
            
            firstIntersection.x = (t-dt) * dxl + linestart.x;
            firstIntersection.y = (t-dt) * dyl + linestart.y;
            nbi = 1;
        }
        if (t+dt >=0 && t+dt <= l) {
            if (nbi == 0) {
                
                firstIntersection.x = (t+dt) * dxl + linestart.x;
                firstIntersection.y = (t+dt) * dyl + linestart.y;
                nbi = 1;
            }
            else {
                    secondIntersection.x = (t+dt) * dxl + linestart.x;
                    secondIntersection.y = (t+dt) * dyl + linestart.y;
                nbi = 2;
            }
        }
        return nbi;
    }
    else if (lec == RHO && t >=0 && t <= l) { // Tangent
            firstIntersection.x = ex;
            firstIntersection.y = ex;
        return 1;
    }
    
    return 0;


}

// Find the points of intersection. 
int FindLineCircleIntersections(Point point1, Point point2,Point center, double radius,Point& intersection1,Point& intersection2)
{

    #if ARC_DEBUG
    std::cout <<"Gkiri:FindLineCircleIntersections line start.x= " << point1.x <<"linestart.y " << point1.y <<"lineend.x " <<point2.x <<"lineend.y " <<point2.y  << std::endl;
    #endif

    double dx, dy, A, B, C, det, t;

    dx = point2.x - point1.x;
    dy = point2.y - point1.y;

    A = dx * dx + dy * dy;
    B = 2 * (dx * (point1.x - center.x) + dy * (point1.y - center.y));
    C = (point1.x- center.x) * (point1.x - center.x) +(point1.y - center.y) * (point1.y - center.y) - radius * radius;


    det = B * B - 4 * A * C;
    #if ARC_DEBUG
    std::cout <<"Gkiri:FindLineCircleIntersections DET= " << det << "line distance"<< sqrt(A)<< "Radius= "<< radius << "centerX= " <<center.x << "centerY= " << center.y<< std::endl;
    #endif

    if(sqrt(A)<0.03) //Corner case for small lines
    {
        //Corner case for small lines-Ignoring small lines
        intersection1 = Point(-1, -1);
        intersection2 = Point(-1, -1);
        //std::cout <<"Gkiri:FindLineCircleIntersections  NEGLIGIBLE LINE"  << std::endl;
        return 0;
    }

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
    bool final_flag;
    struct arc_param test_param;

    /*1st level of Just collision check */
    if(is_line_colliding_circle(line.start_point,line.end_point,arc.center,arc.radius))
    {
        #if ARC_DEBUG
        std::cout <<"Gkiri:is_line_colliding_circle COLLISION DETECTED "  << std::endl;
        #endif

    }
    else{
        #if ARC_DEBUG
        std::cout <<"Gkiri:is_line_colliding_circle COLLISION FREE "  << std::endl;
        std::cout <<"Alvaro:lineArcIntersection> start.x " << arc.start_point.x << "start.y "  <<  arc.start_point.y  <<"end.x= "  << arc.end_point.x << " end.y"  <<  arc.end_point.y <<std::endl;
        #endif

        return false;//return false as there is no collision so no further arc checks needed
    }

    /*Find Circle new method */
    //ret_val=get_line_circle_intersection(line.start_point,line.end_point,arc.center,arc.radius,intersection1,intersection2);

    ret_val=FindLineCircleIntersections(line.start_point,line.end_point,arc.center,arc.radius,intersection1,intersection2);
    #if ARC_DEBUG
    std::cout <<"Gkiri:FindLineCircle**  intersect1.x " << intersection1.x <<" intersect1.y " << intersection1.y << std::endl;
    std::cout <<"Gkiri:FindLineCircle** intersect2.x " << intersection2.x <<" intersect2.y " << intersection2.y << std::endl;
    #endif

    intersect_points.push_back(intersection1);
    intersect_points.push_back(intersection2);

    test_param=calculate_arc_drawing_angles(arc);  

    #if ARC_DEBUG
    std::cout <<"Alvaro:lineArcIntersection> " << "Start_angle= "<<test_param.start_angle <<" end_angle= " << test_param.end_angle << std::endl;
    #endif

    if(ret_val==0){

        //std::cout <<"Gkiri:lineArcIntersection No intersection "  << std::endl;
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

        

    }

    if(ret_val==2){ //2 intersection

        double beta = atan2(intersection1.y - arc.center.y, intersection1.x - arc.center.x)*RAD2DEG;

        if(beta > 0){
            beta = 360 - beta;
        }
        else{
            beta = -beta;
        }
        #if ARC_DEBUG
        std::cout <<"Gkiri:lineArcIntersection =beta angle " << beta <<"Start_angle= "<<test_param.start_angle <<" end_angle= " << test_param.end_angle << std::endl;
        #endif

        if(test_param.end_angle>=360){

            if((test_param.end_angle-360 > beta)  || ( test_param.start_angle < beta)){
                
                #if ARC_DEBUG
                std::cout <<"Gkiri:lineArcIntersection COLLISIONX ret_val==2  LSR 2 intersection1.x= " << intersection1.x << " intersection1.y = " << intersection1.y << std::endl;
                #endif
                
                return true;

            }
            else{
                 #if ARC_DEBUG
                 std::cout <<"Gkiri:lineArcIntersection NO COLLISIONX ret_val==2  LSR 2 intersection1.x= " << intersection1.x << " intersection1.y = " << intersection1.y << std::endl;
                 #endif
                    final_flag= false;

            }
        }else{
            if((beta >= test_param.start_angle) || (beta < test_param.end_angle))
            {
                    #if ARC_DEBUG
                    std::cout <<"Gkiri:lineArcIntersection COLLISION2X ret_val==2  intersection1.x= " << intersection1.x << " intersection1.y = " << intersection1.y << std::endl;
                    #endif

                    return true; 
            }
            else {  
                    #if ARC_DEBUG
                    std::cout <<"Gkiri:lineArcIntersection NO COLLISION2X ret_val==2  LSR 2 intersection1.x= " << intersection1.x << " intersection1.y = " << intersection1.y << std::endl; 
                    #endif

                    final_flag= false;  //check for 2nd point too 
            }

        }



    ///////////////////////////////2nd intersection///////////////////////////////
        double beta2 = atan2(intersection2.y - arc.center.y, intersection2.x - arc.center.x)*RAD2DEG;

        if(beta2 > 0){
            beta2 = 360 - beta;
        }
        else{
            beta2 = -beta2;
        }

        #if ARC_DEBUG
        std::cout <<"Gkiri:lineArcIntersection =beta angle " << beta <<"Start_angle= "<<test_param.start_angle <<" end_angle= " << test_param.end_angle << std::endl;
        #endif

       if(test_param.end_angle>=360){

            if((test_param.end_angle-360 > beta2) || ( test_param.start_angle < beta2)){
                
                #if ARC_DEBUG
                std::cout <<"Gkiri:lineArcIntersection COLLISIONX ret_val==2  LSR 2 intersection2.x= " << intersection2.x << " intersection2.y = " << intersection2.y << std::endl;
                #endif

                return true;

            }
            else{
                 #if ARC_DEBUG
                 std::cout <<"Gkiri:lineArcIntersection NO COLLISIONX ret_val==2  LSR 2 intersection2.x= " << intersection2.x << " intersection2.y = " << intersection2.y << std::endl;
                 #endif

                final_flag= false;

            }
        }else{
            if((beta2 >= test_param.start_angle) || (beta2 < test_param.end_angle))
            {
                    #if ARC_DEBUG
                    std::cout <<"Gkiri:lineArcIntersection COLLISION2X ret_val==2  intersection2.x= " << intersection2.x << " intersection1.y = " << intersection2.y << std::endl;
                    #endif

                    return true; 
            }
            else {
                    #if ARC_DEBUG
                    std::cout <<"Gkiri:lineArcIntersection NO COLLISION2X ret_val==2  LSR 2 intersection2.x= " << intersection2.x << " intersection1.y = " << intersection2.y << std::endl; 
                    #endif
                    final_flag= false;  //check for 2nd point too 
            }

        }

         return final_flag; //Indicates No collision
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

            if(j==Box_list[i].size()-1) {
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
    bool point_lies_inside1,point_lies_inside2;

    std::vector<Polygon> Box_list;
    Build_All_Bounding_Box(obstacle_list,Box_list);//gets lists of bounding boxes


    /*Corner case-1 :Check for two points of line lies inside Bounding Box */
    point_lies_inside1=Detect_point_liesin_polygon(segment.start_point,Box_list);
    point_lies_inside2=Detect_point_liesin_polygon(segment.end_point,Box_list);

    if(point_lies_inside1 && point_lies_inside2)
    {
        //std::cout <<"Gkiri:Both points of line Lies inside Bounding Box" << std::endl;
        return true;//consider it as colliding because line is inside Bounding Box

    }
    
    for (size_t i = 0; i<Box_list.size(); i++){//no of boxes
        
        for (size_t j = 0; j<Box_list[i].size(); j++){ //4corners of each box

            if(j==Box_list[i].size()-1) {
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
bool  Highlevel_Box_dubins_check(std::vector<Polygon>& obstacle_list,struct arc_extract three_seg[3])
{
    bool Intersection;
    bool final_flag;

    //threeseg lenght is 3
    for (size_t i = 0; i<3; i++){

        switch (three_seg[i].LSR)
        {
        case 0: // counter-clockwise
            
            Intersection=Global_Arc_check(obstacle_list,three_seg[i]);

            if(Intersection)
            {
                //std::cout <<"Gkiri:Highlevel_Box_dubins_check ARC COLLISION  " << std::endl;
                return true;
            }
            else
            {
                //std::cout <<"Gkiri:Highlevel_Box_dubins_check NO ARC COLLISION  " << std::endl;
                final_flag= false;
            }
                
            break;
            
        case 1: // Straight line --> Center = 0
            Intersection=Global_Line_check(obstacle_list,three_seg[i]);

            if(Intersection)
            {
                //std::cout <<"Gkiri:Highlevel_Box_dubins_check LINE COLLISION  " << std::endl;
                return true;
            }
            else
            {
                //std::cout <<"Gkiri:Highlevel_Box_dubins_check NO LINE COLLISION  " << std::endl;
                final_flag= false;
            }
            
            break;

        case 2: 
            Intersection=Global_Arc_check(obstacle_list,three_seg[i]);

            if(Intersection)
            {
                //std::cout <<"Gkiri:Highlevel_Box_dubins_check ARC COLLISION  " << std::endl;
                return true;
            }
            else
            {
                //std::cout <<"Gkiri:Highlevel_Box_dubins_check NO ARC COLLISION  " << std::endl;
                final_flag= false;
            }

            break;
            
        default:
            printf("unkown LSR");
            break;
        }  
    }//forloop

    return final_flag;//This will reach only when there is no Collision .Also it should false

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
    bool point_lies_inside1,point_lies_inside2;

    std::vector<Polygon> Box_list;
    Build_All_Bounding_Box(obstacle_list,Box_list);//gets lists of bounding boxes


    /*Corner case-1 :Check for two points of ARC lies inside Bounding Box */
    point_lies_inside1=Detect_point_liesin_polygon(arc.start_point,Box_list);
    point_lies_inside2=Detect_point_liesin_polygon(arc.end_point,Box_list);

    if(point_lies_inside1 && point_lies_inside2)
    {
        std::cout <<"Gkiri:Both points of ARC Lies inside Bounding Box" << std::endl;
        return true;//consider it as colliding because line is inside Bounding Box

    }

    for (size_t i = 0; i<Box_list.size(); i++){//no of boxes
        
        for (size_t j = 0; j<Box_list[i].size(); j++){ //4corners of each box

            if(j==Box_list[i].size()-1) {
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
                if(intersection){
                    //std::cout <<"Gkiri:NARROW intersection at start_point.x= " << i <<"---"<< line_data.start_point.x << "start_point.y= " << line_data.start_point.y <<"end_point.x= " << line_data.end_point.x  << " end_point.y = " << line_data.end_point.y << std::endl; 
                    return true;
                }
                   
            }
            else {
                construct_line_structure(line_data,obstacle_list[i][j],obstacle_list[i][j+1]);
                intersection=lineArcIntersection_prof(line_data,arc ,intersect_points);
                if(intersection){
                    //std::cout <<"Gkiri:NARROW intersection at start_point.x= " << i <<"---"<< line_data.start_point.x << "start_point.y= " << line_data.start_point.y <<"end_point.x= " << line_data.end_point.x  << " end_point.y = " << line_data.end_point.y << std::endl; 
                    return true;
                }
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
        //std::cout <<"Gkiri:Global_Line_check BOX is Not colliding " << std::endl;
        #endif

        return false;
    }   
    else {

        #if COLLISION_DEBUG
        //std::cout <<"Gkiri:Global_Line_check BOX is colliding " << std::endl;
        #endif

        narrow_collision = narrow_polygon_obstacles_line_check(obstacle_list,segment);
        if(narrow_collision){

            #if COLLISION_DEBUG
            //std::cout <<"Gkiri:Global_Line_check NARROW polygon is also colliding " << std::endl;
            #endif

            return true;
        }
        else{

            #if COLLISION_DEBUG
            //std::cout <<"Gkiri:Global_Line_check NARROW polygon is not colliding " << std::endl;
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



/* Draw Point in Polygon Test -------------------------------------------*/
bool Detect_point_liesin_polygon(Point pt,std::vector<Polygon> cv_poly_list)
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