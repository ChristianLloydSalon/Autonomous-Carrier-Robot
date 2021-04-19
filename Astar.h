#ifndef ASTAR_H_
#define ASTAR_H_

#include "Node.h"
#include "List.h"

class Astar
{
public:
	List path;									// path from start to end

	Astar(int, int);							// constructor that accepts two arguments: coordinates of the starting and the end node

	bool FindPath(Vector, Vector);				// creates a path from start to finish
	void ResetMap(void);						// resets the entire map, including the start, end and the obstacles

	Node** nodes;								// dynamic node grid

private:
	int row;									// grid row
	int col;									// grid column
	Node* start;								// address of the starting node
	Node* end;									// address of the end node

	List openList;								// list of unvisited nodes
	List closedList;							// list of visited nodes
	List getNeigbours(Node*);					// gets the neighbours of the current node
	
	void RetracePath(Node*, Node*);				// moves the robot from start to end

	float getDistance(Node* A, Node* B)			// distance from node A to node B
	{
		float distX = (float)(A->coordinates.X - B->coordinates.X) * (float)(A->coordinates.X - B->coordinates.X);
		float distY = (float)(A->coordinates.Y - B->coordinates.Y) * (float)(A->coordinates.Y - B->coordinates.Y);

		return (distX + distY) * 0.5f;
	}
};

#endif 
