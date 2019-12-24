#include <iostream>
#include <ostream>
#include "aabbtests.hpp"

std::ostream& operator<< (std::ostream& os, const point & p)
{	
	os << p.x << " " << p.y << " ";
	return os;
}

void print(const AABB & a)
{
	std::cout << "AABB center(" << a.c << ") halfs(" << a.r << ")" << std::endl;
}

void doTest(const AABB & a, const AABB & b)
{
	std::cout << "AABB 1: "; print(a);
	std::cout << "AABB 2: "; print(b);
	bool postResult = test_normal(a,b);
	bool anonResult = test_anon(a,b);
	bool cjmResult = test_CJM(a,b);
	bool simdResult = test_SIMD(a,b);

	std::cout << "Post result " << postResult << std::endl <<
	"Anon result " << anonResult << std::endl <<
	"CJM result " << cjmResult << std::endl <<
	"SIMD result " << simdResult << std::endl;
}

int main()
{
    const point half(0.5, 0.5);
    AABB a(point(), half);
    AABB b(point(1.0, 0), half);
    AABB c(point(1.5, 0), half);
    AABB d(point(15, 0), half);

    doTest(a,a);
    doTest(a,b);
    doTest(a,c);
    doTest(a,d);

    doTest(b,b);
    doTest(b,c);
    doTest(b,d);

    doTest(c,c);
    doTest(c,d);

    AABB a2(point(0, 1.5), half);
    AABB a3(point(0, 2.5), half);
    AABB a4(point(0, 3.5), half);

    doTest(a,a2);
    doTest(a,a3);
    doTest(a,a4);

    AABB d2(point(15,15), half);
    doTest(a,d2);
}
