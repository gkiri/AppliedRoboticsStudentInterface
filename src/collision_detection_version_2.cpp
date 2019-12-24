
#include<iostream>
#include <vector>
#include <cmath>

//#include "dubins_curve.hpp"

namespace IntersectionFunctions
{
	struct IntersectionObject
	{
		std::vector<Vector2> points;

		void InsertSolution(float x, float y)
		{
			points.push_back(Vector2(x, y));
		}

		void InsertSolution(Vector2 v)
		{ 
			points.push_back(v);
		}

		int NumberOfSolutions()
		{
			return points.size();
		}
	};
    // Used only within this namespace
	int PrivateLineToCircleIntersection(Vector2 vertex1, Vector2 vertex2,
					    Vector2 circlePosition, float radius,
					    Vector2 & solution1, Vector2 & solution2)
	{   
		{
			arc_extract arcStruct;
			arcStruct.start_point = vertex1;
			arcStruct.end_point = vertex2;
			arcStruct.radius = radius;
			arcStruct.centre = circlePosition;
			{if (arcStruct.LSR = 0)  //0 is clockwise i.e., L
			  return 0;
			  else if (arcStruct.LSR = 2)  //2 is counter-clockwise i.e., R
			  return 2;
			  else return 1; //1 is straight i.e., S
			  
			}

		}
		// Vector from point 1 to point 2
		Vector2 vertex1to2 = vertex2 - vertex1;
		// Vector from point 1 to the circle's center
		Vector2 circleToVertex1 = circlePosition - vertex1;

		float dot = vertex1to2.Dot(circleToVertex1);
		Vector2 proj1 = vertex1to2 * (dot / vertex1to2.LengthSq());

		Vector2 midpt = vertex1 + proj1;
		Vector2 circleToMidpt = midpt - circlePosition;
		float distSqToCenter = circleToMidpt.LengthSq();
		if (distSqToCenter > radius * radius) return 0;
		if (distSqToCenter == radius * radius)
		{
			solution1 = midpt;
			return 1;
		}
		float distToIntersection;
		if (distSqToCenter == 0)
		{
			distToIntersection = radius;
		}
		else
		{
			distToIntersection = sqrt(radius * radius - distSqToCenter);
		}
		vertex1to2 = vertex1to2.Normalize();
		vertex1to2 *= distToIntersection;

		solution1 = midpt + vertex1to2;
		solution2 = midpt - vertex1to2;
		return 2;
	}
	// LineSegment to Circle
	IntersectionObject LineSegmentToCircleIntersection(Vector2 vertex1, Vector2 vertex2,
													   Vector2 circlePosition, float radius)
	{
		IntersectionObject result;
		Vector2 solution1, solution2;
		Vector2 vertex1to2 = vertex2 - vertex1;
		Vector2 vertex1ToSolution1, vertex2ToSolution1, vertex1ToSolution2, vertex2ToSolution2;
		switch(PrivateLineToCircleIntersection(vertex1, vertex2, circlePosition, radius, solution1, solution2))
		{
		case 2:
			vertex1ToSolution2 = solution2 - vertex1;
			vertex2ToSolution2 = solution2 - vertex2;
			if (vertex1ToSolution2.Dot(vertex1to2) > 0 &&
				vertex2ToSolution2.Dot(vertex1to2) < 0)
			{
				result.InsertSolution(solution2);
			}
		case 1:
			vertex1ToSolution1 = solution1 - vertex1;
			vertex2ToSolution1 = solution1 - vertex2;
			if (vertex1ToSolution1.Dot(vertex1to2) > 0 &&
				vertex2ToSolution1.Dot(vertex1to2) < 0)
			{
				result.InsertSolution(solution1);
			}
			break;
		}
		return result;
	}

	// Circle to LineSegment
	IntersectionObject CircleToLineSegmentIntersection(Vector2 circlePosition, float radius,
							   Vector2 vertex1, Vector2 vertex2)
	{
		return LineSegmentToCircleIntersection(vertex1, vertex2, circlePosition, radius);
	}
    // Used only within this namespace
	bool PrivateLineToLineIntersection(Vector2 vertex1, Vector2 vertex2,
					   Vector2 vertex3, Vector2 vertex4, float & r, float & s)
	{
		{
			arcStruct lineseg1;
			lineseg1.start_point = vertex1;
			lineseg1.end_point = vertex2;
			{if (arcStruct.LSR = 0)  //0 is clockwise i.e., L
			  return 0;
			  else if (arcStruct.LSR = 2)  //2 is counter-clockwise i.e., R
			  return 2;
			  else return 1; //1 is straight i.e., S
			}


		}{
			arcStruct lineseg2;
			lineseg2.start_point = vertex3;
			lineseg2.end_point = vertex4;
			{if (arcStruct.LSR = 0)  //0 is clockwise i.e., L
			  return 0;
			  else if (arcStruct.LSR = 2)  //2 is counter-clockwise i.e., R
			  return 2;
			  else return 1; //1 is straight i.e., S
			}
		}
		float d;
		//Make sure the lines aren't parallel
		Vector2 vertex1to2 = vertex2 - vertex1;
		Vector2 vertex3to4 = vertex4 - vertex3;
		//if (vertex1to2.x * -vertex3to4.y + vertex1to2.y * vertex3to4.x != 0)
		//{
		if(vertex1to2.y / vertex1to2.x != vertex3to4.y / vertex3to4.x)
		{
			d = vertex1to2.x * vertex3to4.y - vertex1to2.y * vertex3to4.x;
			if (d != 0)
			{
				Vector2 vertex3to1 = vertex1 - vertex3;
				r = (vertex3to1.y * vertex3to4.x - vertex3to1.x * vertex3to4.y) / d;
				s = (vertex3to1.y * vertex1to2.x - vertex3to1.x * vertex1to2.y) / d;
				return true;
			}
		}
		return false;
	}

	// LineSegment to LineSegment
	IntersectionObject LineSegmentToLineSegmentIntersection(Vector2 vertex1, Vector2 vertex2,
								Vector2 vertex3, Vector2 vertex4)
	{
		IntersectionObject result;
		float r, s;
		if(PrivateLineToLineIntersection(vertex1, vertex2, vertex3, vertex4, r, s))
		{
			if (r >= 0 && r <= 1)
			{
				if (s >= 0 && s <= 1)
				{
					result.InsertSolution(vertex1 + (vertex2 - vertex1) * r);
				}
			}
		}
		return result;
	}

}