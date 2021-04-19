#ifndef NODE_H_
#define NODE_H_

#include <WString.h>
#include "Vector.h"

struct Node
{
	Node()
	{
		gCost      = hCost = 0;
		isWalkable = true;
	}
	
	Node* parent;			// address of the parent node
	
	Vector coordinates;		// X and Y coordinates of the node
	
	String cardUID;			// card's UID
	
	bool isWalkable;		// true if the node is walkable, otherwise false
		
	float gCost;			// distance from current node to end node
	float hCost;			// distance from current node to start node
	
	float fCost()
	{
		return gCost + hCost;
	}
};

#endif
