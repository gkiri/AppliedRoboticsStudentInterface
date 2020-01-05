#include <iostream>
#include <vector>
#include <map>
#include <cmath>

//  Determines the intersection point of the line defined by points A and B with the
//  line defined by points C and D.
//
//  Returns YES if the intersection point was found, and stores that point in X,Y.
//  Returns NO if there is no determinable intersection point, in which case X,Y will
//  be unmodified.
#define EPS 1e-9

bool linelineIntersection(
double Ax, double Ay,
double Bx, double By,
double Cx, double Cy,
double Dx, double Dy,
double *X, double *Y) {

  double  distAB, theCos, theSin, newX, ABpos ;

  //  Fail if either line is undefined.
  if (Ax==Bx && Ay==By || Cx==Dx && Cy==Dy) return false;

  //  (1) Translate the system so that point A is on the origin.
  Bx-=Ax; By-=Ay;
  Cx-=Ax; Cy-=Ay;
  Dx-=Ax; Dy-=Ay;

  //  Discover the length of segment A-B.
  distAB=sqrt(Bx*Bx+By*By);

  //  (2) Rotate the system so that point B is on the positive X axis.
  theCos=Bx/distAB;
  theSin=By/distAB;
  newX=Cx*theCos+Cy*theSin;
  Cy  =Cy*theCos-Cx*theSin; Cx=newX;
  newX=Dx*theCos+Dy*theSin;
  Dy  =Dy*theCos-Dx*theSin; Dx=newX;

  //  Fail if the lines are parallel.
  if (Cy==Dy) return false;

  //  (3) Discover the position of the intersection point along line A-B.
  ABpos=Dx+(Cx-Dx)*Dy/(Dy-Cy);

  //  (4) Apply the discovered position to line A-B in the original coordinate system.
  *X=Ax+ABpos*theCos;
  *Y=Ay+ABpos*theSin;

  //  Success.
  return true; }

bool linecircleIntersection(
double r, // radius of circle
double a, // first parameter of line ax+by+c
double b, // second parameter of line ax+by+c
double c // third parameter of line ax+by+c
) {
	double x0 = -a*c/(a*a+b*b), y0 = -b*c/(a*a+b*b);
	if (c*c > r*r*(a*a+b*b)+EPS)
	    puts ("no points");
	else if (abs (c*c - r*r*(a*a+b*b)) < EPS) {
        puts ("1 point");
        //cout << x0 << ' ' << y0 << '\n';
    }
	else {
    double d = r*r - c*c/(a*a+b*b);
    double mult = sqrt (d / (a*a+b*b));
    double ax, ay, bx, by;
    ax = x0 + b * mult;
    bx = x0 - b * mult;
    ay = y0 - a * mult;
    by = y0 + a * mult;
    puts ("2 points");
    //cout << ax << ' ' << ay << '\n' << bx << ' ' << by << '\n';
	}
}

/*int main()
{
	std::vector<Point> intersections;
	std::vector<Segment> segments;
	segments.push_back(Segment(Point('A',-7.41, -0.58), Point('C',-1.3,-0.79)));
	segments.push_back(Segment(Point('B',-4.0, 1.27),   Point('D',-4.21, -2.99)));
	segments.push_back(Segment(Point('F',-4.92, 0.71),  Point('G',-4.26, -1.40)));
	segments.push_back(Segment(Point('I',-4.55, -1.24), Point('J',-2.54, -0.42)));
	segments.push_back(Segment(Point('K',-3.70, 0.48),  Point('L',-3.70, -2.41))); //vertical
	intersect(segments, intersections,false);
	std::cout << "Intersection points[" << intersections.size() << "]: " << std::endl;
	for (std::vector<Point>::iterator it = intersections.begin(); it != intersections.end(); ++it)
		std::cout << it->letter << "(" << it->x << "," << it->y << ") " << std::endl;
	std::cout << "Segments[" << segments.size() << "]: " << std::endl;
	for (std::vector<Segment>::iterator it = segments.begin(); it != segments.end(); ++it)
		std::cout << "[ " << it->beg.letter << "(" << it->beg.x << "," << it->beg.y << "), " << it->end.letter << "(" << it->end.x << "," << it->end.y << ") ] " << std::endl;
	return 0;
}*/