#ifndef LIST_H_
#define LIST_H_

#include "Node.h"

struct Data
{
	Data(Node* );					// constructor

	Node* node;						// node data
	Data* next;						// address of next element
};

class List
{
public:								// constructor
	List();

	int Count;						// size of the list

	Data* front;					// address of the front node

	void Add(Node*);				// adds an element to the list
	void Reverse(void);				// reverse elements
	void Clear(void);				// removes all elements from the list
	void Remove(Node*);				// removes the element from the top
	
	Node* operator[] (int index);	// gets the ith node

	bool Contains(Node*);			// returns true if the node is in the list, otherwise false

private:
	bool isEmpty();					// returns true if list is empty, otherwise false
};

#endif
