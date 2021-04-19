#include "Astar.h"

Astar::Astar(int row, int col)
{
	this->row = row;
	this->col = col;

	nodes = new Node*[col];

	for (int i = 0; i < row; i++)
	{
		nodes[i] = new Node[row];
	}

	ResetMap();
}

bool Astar::FindPath(Vector vStart, Vector vEnd)
{
	path.Clear();
	
	start = &nodes[vStart.Y][vStart.X];
	end   = &nodes[vEnd.Y][vEnd.X];
 	
	start->hCost = getDistance(start, end);

	openList.Add(start);

	while (openList.Count > 0)
	{
		Node* node = openList[0];

		for (int i = 1; i < openList.Count; i++)
		{
			if (openList[i]->fCost() < node->fCost() || openList[i]->fCost() == node->fCost())
			{
				if (openList[i]->hCost < node->hCost)
					node = openList[i];
			}
		}
		
		openList.Remove(node);
		closedList.Add(node);

		if (node == end)
		{
			openList.Clear();
			closedList.Clear();

			RetracePath(start, end);
			return true;
		}

		List neighbours = getNeigbours(node);

		for (int i = 0; i < neighbours.Count; i++)
		{
			if (!neighbours[i]->isWalkable || closedList.Contains(neighbours[i]))
			{
				continue;
			}

			double newCostToNeighbour = node->gCost + getDistance(node, neighbours[i]);

			if (newCostToNeighbour < neighbours[i]->gCost || !openList.Contains(neighbours[i]))
			{
				neighbours[i]->gCost  = newCostToNeighbour;
				neighbours[i]->hCost  = getDistance(neighbours[i], end);
				neighbours[i]->parent = node;

				if (!openList.Contains(neighbours[i]))
				{
					openList.Add(neighbours[i]);
				}
			}
		}
		
		neighbours.Clear();
	}
	
	ResetMap();
	
	return false;

}

void Astar::RetracePath(Node* s, Node* e)
{
	Node* current = e;

	while (current != s)
	{
		path.Add(current);
		current = current->parent;
	}

	path.Add(s);

	path.Reverse();
}

void Astar::ResetMap(void)
{
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			nodes[i][j].isWalkable = true;

			nodes[i][j].coordinates.X = j;
			nodes[i][j].coordinates.Y = i;

			nodes[i][j].gCost = 0.0f;
			nodes[i][j].hCost = 0.0f;
		}
	}

	openList.Clear();
	closedList.Clear();
	path.Clear();
}

List Astar::getNeigbours(Node *currentNode)
{
	List neighbours;
	if (currentNode->coordinates.X - 1 >= 0)
	{
		neighbours.Add(&nodes[currentNode->coordinates.Y][currentNode->coordinates.X - 1]);
	}

	if (currentNode->coordinates.X + 1 < col)
	{
		neighbours.Add(&nodes[currentNode->coordinates.Y][currentNode->coordinates.X + 1]);
	}

	if (currentNode->coordinates.Y - 1 >= 0)
	{
		neighbours.Add(&nodes[currentNode->coordinates.Y - 1][currentNode->coordinates.X]);
	}

	if (currentNode->coordinates.Y + 1 < row)
	{
		neighbours.Add(&nodes[currentNode->coordinates.Y + 1][currentNode->coordinates.X]);
	}
	
	return neighbours;
}
