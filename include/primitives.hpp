#if !defined (BASIC_PRIMITIVES_HPP_INCLUDED)
#define       BASIC_PRIMITIVES_HPP_INCLUDED

#include <assert.h>

struct point
{
	point() {}
	point( double x, double y)
		: x(x)
        , y(y)
	{}
	double x = 0.0;
	double y = 0.0;
	double w = 0.0;

	const double operator[](const int idx) const
	{
		if (idx == 0) return x;
		if (idx == 1) return y;
		if (idx == 2) return w;

		assert(0);
	}
};

struct AABB_minmax
{
	AABB_minmax() : min(), max() {}
	AABB_minmax( const point & min, const point & max ) 
		: min(min)
		, max(max)
	{}

	point min;
	point max;
};

struct AABB_center
{
	AABB_center() : c(), r() {}

	AABB_center(const point & center, const point & halfwidths)
		: c(center)
		, r(halfwidths)
	{}

	point c;		// center point
	point r;		// halfwidths
};

#endif