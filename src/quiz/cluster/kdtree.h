/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
private:
	void insert_reccursive(Node** parent,std::vector<float> pt,int id,int depth){
		if(*parent==NULL)
			*parent = new Node(pt,id);
		else{
			if(pt[depth%2] < (*parent)->point[depth%2])
				insert_reccursive(&((*parent)->left) , pt , id , depth +1 );
			else
				insert_reccursive(&((*parent)->right) , pt , id , depth +1 );
		}
		
	}

public:
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insert_reccursive(&root , point , id , 0);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




