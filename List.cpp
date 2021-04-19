#include "List.h"

List::List()
{
	Count = 0;
	front = NULL;
}

void List::Add(Node* node)
{
	Data* newData = new Data(node);

	Count++;

	if (front == NULL)
	{
		front = newData;
		return;
	}

	Data* temp = front;
	while (temp->next != NULL)
	{
		temp = temp->next;
	}

	temp->next = newData;
}

void List::Remove(Node* searchedNode)
{
	if (!isEmpty())
	{
		Count--;

		if (front->node == searchedNode)
		{
			Data* temp = front;
			front = front->next;
			
			delete temp;
			
			return;
		}

		Data* temp = front;
		while (temp->next != NULL)
		{
			if (temp->next->node == searchedNode)
			{
				Data* t = temp->next;
				temp->next = temp->next->next;
				
				delete t;
				
				return;
			}
			temp = temp->next;
		}
	}
}

void List::Clear(void)
{
	Count = 0;

	while (front != NULL)
	{
		Data* temp = front;
		front = front->next;
		delete temp;
	}
}

Node * List::operator[](int index)
{
	if (front == NULL)
	{
		return NULL;
	}

	Data* temp = front;
	int count = 0;

	while (temp != NULL)
	{
		if (count == index)
		{
			return temp->node;
		}

		temp = temp->next;
		count++;
	}
}

bool List::Contains(Node * searchedNode)
{
	Data* temp = front;
	while (temp != NULL)
	{
		if (temp->node == searchedNode)
		{
			return true;
		}

		temp = temp->next;
	}

	return false;
}

bool List::isEmpty()
{
	return (front == NULL) ? true : false;
}

void List::Reverse(void)
{
	Data* current = front;
	Data* prev = NULL;
	Data* next = NULL;
	
	while(current != NULL)
	{
		// store next
		next = current->next;
		
		// reverse current node's pointer
		current->next = prev;
		
		// move pointers one position ahead
		prev = current;
		current = next;
	}
	
	front = prev;
}

Data::Data(Node* node)
{
	this->node = node;
	this->next = NULL;
}
