
#include <collision_detection.hpp>
#define ARC_DEBUG 0


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

bool areClockwise(Point v1, Point v2) {
  return (-v1.x*v2.y + v1.y*v2.x) > 0;
}
bool isWithinRadius(Point v, double radiusSquared) {
  return ((v.x*v.x + v.y*v.y) <= radiusSquared);
}
//https://stackoverflow.com/questions/13652518/efficiently-find-points-inside-a-circle-sector
bool point_inside_arc(Point center,Point Test,double r,Point sectorStart,Point sectorEnd)
{
 
    Point relPoint;
    relPoint.x= Test.x - center.x;
    relPoint.y= Test.y - center.y;
    
    #if ARC_DEBUG
    std::cout <<"Gkiri:lineArcIntersection =areClockwise " << areClockwise(sectorEnd, relPoint) << std::endl;
    std::cout <<"Gkiri:lineArcIntersection =AntiClockwise " << !areClockwise(sectorStart, relPoint) << std::endl;
    std::cout <<"Gkiri:lineArcIntersection =isWithinRadius " << isWithinRadius(relPoint, r) << "radius= "<<relPoint.x*relPoint.x + relPoint.y*relPoint.y<< std::endl;
    #endif

     return !areClockwise(sectorStart, relPoint) &&
            areClockwise(sectorEnd, relPoint) &&
            isWithinRadius(relPoint, r);//r or r*rcheck????

}

// /https://stackoverflow.com/questions/30006155/calculate-intersect-point-between-arc-and-line?rq=1
//line(start,end)
//arc(r,center)
bool lineArcIntersection(Point Line_Start,Point Line_End,Point Arc_Start,Point Arc_End,double r,Point center,std::vector<Point>& cal_points)
{

    double theta;
    double line_dist;
    double A,B,C;
    double p,q,temp,t1,t2;

    Point arc_point1,arc_point2;

    line_dist=pow((Line_Start.x-Line_End.x),2)+pow((Line_Start.y-Line_End.y),2);
    line_dist=sqrt(line_dist);

    theta=asin(line_dist/(2*r));

    //line equation
    //A=Y2-Y1, B=X1-X2 and C=X2.Y1-X1.Y2
    A=Line_End.y - Line_Start.y;
    B=Line_Start.x - Line_End.x;
    C=Line_End.x * Line_Start.y - Line_Start.x *Line_End.y;
    
    //t = p +/- q = arctan(B/A) +/- arccos(-(A.Xc + B.Yc + C)/R.√A²+B²)

    p=atan(B/A);

    temp=(A*center.x+B*center.y+C)/(r*sqrt(A*A+B*B));
    q=acos(-temp);

    t1=p+q;
    t2=p-q;

    #if ARC_DEBUG
    std::cout <<"Gkiri:lineArcIntersection =t1 " << t1 <<" t2 " << t2 << std::endl;
    #endif
    if(std::isnan(t1) && std::isnan(t2))
    {
        return false;//NO Intersection
    }

    arc_point1.x=r*cos(t1) + center.x;
    arc_point1.y=r*sin(t1) + center.y;

    arc_point2.x=r*cos(t2) + center.x;
    arc_point2.y=r*sin(t2) + center.y;

    double temp_radius1,temp_radius2;

    temp_radius1 =sqrt((center.x-arc_point1.x)*(center.x-arc_point1.x)+(center.y-arc_point1.y)*(center.y-arc_point1.y));
    temp_radius2 =sqrt((center.x-arc_point2.x)*(center.x-arc_point2.x)+(center.y-arc_point2.y)*(center.y-arc_point2.y));

    #if ARC_DEBUG
    std::cout <<"Gkiri:lineArcIntersection =radius=orig " << r <<" temp_radius1 " << temp_radius1 <<" temp_radius2 " << temp_radius2 << std::endl;
    std::cout <<"Gkiri:lineArcIntersection =Arc_Start.x" <<Arc_Start.x<<" Arc_Start.y " << arc_point1.y << std::endl;    
    std::cout <<"Gkiri:lineArcIntersection =Arc_End.x" <<Arc_End.x<<" Arc_End.y " << Arc_End.y << std::endl;    
    std::cout <<"Gkiri:lineArcIntersection =arc_point1.x" <<arc_point1.x<<" arc_point1.y " << arc_point1.y << std::endl;    
    std::cout <<"Gkiri:lineArcIntersection =arc_point2.x" <<arc_point2.x<<" arc_point2.y " << arc_point2.y << std::endl;    
    #endif

    bool inside=point_inside_arc(center,arc_point1, r,Arc_Start,Arc_End);//0 inside 1 outside
    #if ARC_DEBUG
    std::cout <<"Gkiri:lineArcIntersection =point_inside_arc= " << inside << std::endl;
    #endif
    
    bool inside2=point_inside_arc(center,arc_point2, r,Arc_Start,Arc_End);
    #if ARC_DEBUG
    std::cout <<"Gkiri:lineArcIntersection =point_inside_arc2= " << inside2 << std::endl;
    #endif

    cal_points.push_back(center);
    cal_points.push_back(arc_point1);
    cal_points.push_back(arc_point2);

    if(!inside && !inside2)
    {
        //std::cout <<"Gkiri:NO INTERSECTION  " << std::endl;
        return false;
    }
    else{
        //std::cout <<"Gkiri:INTERSECTION " << std::endl;
        return true;
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
void Build_All_Bounding_Box(std::vector<Polygon> obstacle_list,std::vector<Polygon>& Box_list){

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
    bool intersection;
    std::vector<Point> cal_points;
    Point Line_start_point,Line_end_point,X;
    int k=0;
    for (size_t i = 0; i<Box_list.size(); i++){//no of boxes
        
        for (size_t j = 0; j<Box_list[i].size(); j++){ //4corners of each box

            if(j==3) {
                intersection=lineArcIntersection(Box_list[i][j],Box_list[i][0],segment.start_point,segment.end_point,segment.radius,segment.center,cal_points);
                if(intersection)
                    return true;

            }
            else{
                intersection=lineArcIntersection(Box_list[i][j],Box_list[i][j+1],segment.start_point,segment.end_point,segment.radius,segment.center,cal_points);
                if(intersection)
                    return true;
            }
            
        }//Inner forloop
    }//Outer forloop

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

/*High level box vs each arc of dubins check */
bool  Highlevel_Box_dubins_check(std::vector<Polygon> obstacle_list,struct arc_extract *three_seg)
{
    bool Intersection;
    std::vector<Polygon> Box_list;
    Build_All_Bounding_Box(obstacle_list,Box_list);//gets lists of bounding boxes

    for (size_t i = 0; i<3; i++){

        switch (three_seg[i].LSR)
        {
        case 0: // counter-clockwise
            
            Intersection=Process_Box_arc_check(Box_list,three_seg[i]);
            if(Intersection)
                std::cout <<"Gkiri:Highlevel_Box_dubins_check line-arc INTERSECTION  " << std::endl;
            else
                std::cout <<"Gkiri:Highlevel_Box_dubins_check line-arc NO INTERSECTION  " << std::endl;

            break;
        case 1: // Straight line --> Center = 0
            Intersection=Process_Box_line_check(Box_list,three_seg[i]);

            if(Intersection)
                std::cout <<"Gkiri:Highlevel_Box_dubins_check line-line INTERSECTION  " << std::endl;
            else
                std::cout <<"Gkiri:Highlevel_Box_dubins_check line-line NO INTERSECTION  " << std::endl;
            
            break;

        case 2: 
            Intersection=Process_Box_arc_check(Box_list,three_seg[i]);
            if(Intersection)
                std::cout <<"Gkiri:Highlevel_Box_dubins_check line-arc INTERSECTION  " << std::endl;
            else
                std::cout <<"Gkiri:Highlevel_Box_dubins_check line-arc NO INTERSECTION  " << std::endl;

            break;
            
        default:
            printf("unkown LSR");
            break;
        }  
    }//forloop


}
