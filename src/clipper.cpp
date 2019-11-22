#include "clipper/clipper.hpp"

ClipperLib::Path srcPoly;

ClipperLib::Paths newPoly;

const double INT_ROUND = 1000.;
for (size_t i = 0; i<pointsX.size(); ++i){
  int x = pointsX[i] * INT_ROUND;
  int y = pointsY[i] * INT_ROUND;
  srcPoly << ClipperLib:;IntPoint(x, y);
}

ClipperLib::ClipperOffset co;
if (pointsX.size() == 3){
  co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedLine);
} else {
  co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
}

co.Execute(newPoly, OFFSET);

for (const ClipperLib::Path &path: newPoly){
  //Obstacle obst = create data structure for current obstacle...
  for (const ClipperLib::IntPoint &pt: path){
    double x = pt.X / INT_ROUND;
    double y = pt.Y / INT_ROUND;
  } 
  //Close and export current obstacle...
}