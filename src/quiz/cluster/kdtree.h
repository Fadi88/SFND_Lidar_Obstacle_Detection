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
	void insert_recursive(Node** parent,std::vector<float> pt,int id,int depth){
		if(*parent==NULL)
			*parent = new Node(pt,id);
		else{
			if(pt[depth%2] < (*parent)->point[depth%2])
				insert_recursive(&((*parent)->left) , pt , id , depth +1 );
			else
				insert_recursive(&((*parent)->right) , pt , id , depth +1 );
		}
		
	}

	void search_recursive(const std::vector<float>& target, float distance_tolerance, int depth, Node* parent,std::vector<int> &ids){
		if(parent != NULL){
			if(
				(parent->point[0] >= target[0]-distance_tolerance) &&
				(parent->point[0] <= target[0]+distance_tolerance) &&
				(parent->point[1] >= target[1]-distance_tolerance) &&
				(parent->point[1] <= target[1]+distance_tolerance)
			){
				float distance = std::sqrt(
					((parent->point[0] - target[0]) * (parent->point[0] - target[0]))
					+
					((parent->point[1] - target[1]) * (parent->point[1] - target[1]))
				);

				if(distance <= distance_tolerance){
					ids.push_back(parent->id);
				}
			}

			if((target[depth%2] - distance_tolerance) < parent->point[depth%2])
				search_recursive(target , distance_tolerance , depth + 1 , parent->left , ids );

			if((target[depth%2] + distance_tolerance) > parent->point[depth%2])
				search_recursive(target , distance_tolerance , depth + 1 , parent->right , ids );


		}
	}

public:
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insert_recursive(&root , point , id , 0);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_recursive(target,distanceTol,0,root,ids);
		return ids;
	}
	

};




