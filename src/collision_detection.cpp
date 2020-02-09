
#include <collision_detection.hpp>
#define ARC_DEBUG 1
#define RAD2DEG 180.0/M_PI
#define COLLISION_DEBUG 1

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

    std::cout <<"Gkiri:FindLineCircleIntersections line start.x= " << point1.x <<"linestart.y " << point1.y <<"lineend.x " <<point2.x <<"lineend.y " <<point2.y  << std::endl;

    double dx, dy, A, B, C, det, t;

    dx = point2.x - point1.x;
    dy = point2.y - point1.y;

    A = dx * dx + dy * dy;
    B = 2 * (dx * (point1.x - center.x) + dy * (point1.y - center.y));
    C = (point1.x- center.x) * (point1.x - center.x) +(point1.y - center.y) * (point1.y - center.y) - radius * radius;


    det = B * B - 4 * A * C;
    std::cout <<"Gkiri:FindLineCircleIntersections DET= " << det << "line distance"<< sqrt(A)<< "Radius= "<< radius << "centerX= " <<center.x << "centerY= " << center.y<< std::endl;
    if(sqrt(A)<0.03) //Corner case for small lines
    {
        //Corner case for small lines-Ignoring small lines
        intersection1 = Point(-1, -1);
        intersection2 = Point(-1, -1);
        std::cout <<"Gkiri:FindLineCircleIntersections  NEGLIGIBLE LINE"  << std::endl;
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

    /*Find Circle new method */
    ret_val=get_line_circle_intersection(line.start_point,line.end_point,arc.center,arc.radius,intersection1,intersection2);

    //ret_val=FindLineCircleIntersections(line.start_point,line.end_point,arc.center,arc.radius,intersection1,intersection2);
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

        std::cout <<"Gkiri:lineArcIntersection =beta angle " << beta << std::endl;

        if((beta > test_param.start_angle) && (beta < test_param.end_angle))
        {
                std::cout <<"Gkiri:lineArcIntersection  Intersection ret_val==1 " << std::endl; 
                return true; 
        }
        else {
                std::cout <<"Gkiri:lineArcIntersection  NO Intersection ret_val==1 " << std::endl;  
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

        std::cout <<"Gkiri:lineArcIntersection =beta angle " << beta << std::endl;

        if((beta > test_param.start_angle) && (beta < test_param.end_angle))
        {
                std::cout <<"Gkiri:lineArcIntersection Intersection ret_val==2  intersection1.x= " << intersection1.x << " intersection1.y = " << intersection1.y << std::endl;
                return true; 
        }
        else {
                std::cout <<"Gkiri:lineArcIntersection NO Intersection ret_val==2  intersection1.x= " << intersection1.x << " intersection1.y = " << intersection1.y << std::endl; 
                final_flag= false;  //check for 2nd point too
        }

        ///////////////////////////////2nd intersection///////////////////////////////
        double beta2 = atan2(intersection2.y - arc.center.y, intersection2.x - arc.center.x)*RAD2DEG;

        if(beta2 > 0){
            beta2 = 360 - beta2;
        }
        else{
            beta2 = -beta2;
        }

        std::cout <<"Gkiri:lineArcIntersection =beta angle " << beta2 << std::endl;

        if((beta2 > test_param.start_angle) && (beta2 < test_param.end_angle))
        {
                std::cout <<"Gkiri:lineArcIntersection Intersection ret_val==2  intersection2.x= " << intersection2.x << " intersection2.y = " << intersection2.y << std::endl; 
                return true;
        }
        else {
                std::cout <<"Gkiri:lineArcIntersection NO Intersection ret_val==2  intersection2.x= " << intersection2.x << " intersection2.y = " << intersection2.y << std::endl; 
                final_flag= false  ;

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

    std::vector<Polygon> Box_list;
    Build_All_Bounding_Box(obstacle_list,Box_list);//gets lists of bounding boxes

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
                std::cout <<"Gkiri:Highlevel_Box_dubins_check ARC COLLISION  " << std::endl;
                return true;
            }
            else
            {
                std::cout <<"Gkiri:Highlevel_Box_dubins_check NO ARC COLLISION  " << std::endl;
                final_flag= false;
            }
                
            break;
            
        case 1: // Straight line --> Center = 0
            Intersection=Global_Line_check(obstacle_list,three_seg[i]);

            if(Intersection)
            {
                std::cout <<"Gkiri:Highlevel_Box_dubins_check LINE COLLISION  " << std::endl;
                return true;
            }
            else
            {
                std::cout <<"Gkiri:Highlevel_Box_dubins_check NO LINE COLLISION  " << std::endl;
                final_flag= false;
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
                if(intersection){
                    std::cout <<"Gkiri:NARROW intersection at start_point.x= " << i <<"---"<< line_data.start_point.x << "start_point.y= " << line_data.start_point.y <<"end_point.x= " << line_data.end_point.x  << " end_point.y = " << line_data.end_point.y << std::endl; 
                    return true;
                }
                   
            }
            else {
                construct_line_structure(line_data,obstacle_list[i][j],obstacle_list[i][j+1]);
                intersection=lineArcIntersection_prof(line_data,arc ,intersect_points);
                if(intersection){
                    std::cout <<"Gkiri:NARROW intersection at start_point.x= " << i <<"---"<< line_data.start_point.x << "start_point.y= " << line_data.start_point.y <<"end_point.x= " << line_data.end_point.x  << " end_point.y = " << line_data.end_point.y << std::endl; 
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