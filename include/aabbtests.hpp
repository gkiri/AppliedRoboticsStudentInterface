#if ! defined(AABBTESTS_HPP_INCLUDED)
#define       AABBTESTS_HPP_INCLUDED

#include <cmath>
#include "primitives.hpp"

typedef AABB_center AABB;


//std::fabs(double a);


bool test_normal(const AABB &a, const AABB &b)
{for (int i = 0; i <= 1; i++){
	if ( fabs(a.c[i] - b.c[i]) > (a.r[i] + b.r[i]) ) return false;
	//if ( fabs(a.c[1] - b.c[1]) > (a.r[1] + b.r[1]) ) return false;
	
	// We have an overlap
	return true;
}
};

bool test_anon(const AABB &a, const AABB &b)
{for (int i = 0; i <= 1; i++){
	if ( fabs(a.c[i] - b.c[i]) > (a.r[i] + b.r[i]) )
	{
		//if ( fabs(a.c[1] - b.c[1]) > (a.r[1] + b.r[1]) )
		{
			return false;
		}
	}

	return true;}
}

bool test_CJM(const AABB &a, const AABB &b)
{for (int i = 0; i <= 1; i++){
	bool xOverlap = true;
	bool yOverlap = true;
	bool anyOverlap = false;
	
	if (fabs(a.c[i] - b.c[i]) > (a.r[i] + b.r[i])) xOverlap = false;
	//if (fabs(a.c[1] - b.c[1]) > (a.r[1] + b.r[1])) yOverlap = false;

	anyOverlap = xOverlap && yOverlap;

	return anyOverlap;}
}

bool test_SIMD(const AABB &a, const AABB &b)
{for (int i = 0; i <= 1; i++){
	// SIMD optimized AABB-AABB test
	// Optimized by removing conditional branches
	bool x = fabs(a.c[i] - b.c[i]) <= (a.r[i] + b.r[i]);
	bool y = fabs(a.c[i] - b.c[i]) <= (a.r[i] + b.r[i]);

	return x && y;}
}

#endif // AABBTESTS_HPP_INCLUDED

