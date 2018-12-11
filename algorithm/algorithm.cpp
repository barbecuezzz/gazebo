#include <istream>
#include "math.h"
#include <Eigen/Dense>
#include "algorithm.h"


Vector3d FindRoute(ListNode *from,ListNode *to)
{
	int i;
	Vector3d temp;
	Vector3d idx2;
	i = to->data.mother;
	if (i==0)
	{
		return temp(0,0,0);
	}
	else if(i==from->data.id)
	{
		return temp(to->data.mother,0,0);
	}
	else
	{
		idx2 = FindRoute(from,to);
		if(FindRoute == temp(0,0,0))
		{
			return temp(0,0,0);
		}
		else
		{
			return temp(idx2,to->data.id,0);	
		}
	}
}
