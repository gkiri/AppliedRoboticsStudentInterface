#include <stdlib.h>
#include "AABB.h"

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif


	AABB::AABB()
	{
	}


	AABB::AABB(Vector location, std::vector<Vector> vertices)
	{
		position = location;
		findMinMax(vertices);
	}

	void AABB::findMinMax(std::vector<Vector> vertices)
	{
		
		Vector localMin = Vector( INT_MAX - 0.01, INT_MAX - 0.01, INT_MAX - 0.01) ;
		Vector localMax = Vector( INT_MIN + 0.01, INT_MIN + 0.01, INT_MIN + 0.01) ;
		

		for ( int i = 0; i < 3; i++)
			for(int j = 0; j < vertices.size(); j++){
				if(localMin[i] > vertices.at(j)[i])
					localMin[i] = vertices.at(j)[i];
				if(localMax[i] < vertices.at(j)[i])
					localMax[i] = vertices.at(j)[i];
			}
			
		min = Vector ( localMin ) ;
		max = Vector ( localMax ) ;

	}
    //debug: see min and max
		/*std::cout << min << " " << max << std::endl;
		std::cout << min.distance(max) << std::endl << std::endl;*/

		/*