/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node *&node, uint depth, std::vector<float> point, int id)
	{
		//tree has been traversed, create new node
		if(node==NULL)
			node = new Node(point,id);
		else
		{
			uint dimm = depth % 3;

			//if x/y/z value is less than x/y/z of current node, traverse left
			if(point[dimm] < node->point[dimm])
				insertHelper(node->left, depth+1, point, id);
			//else traverse right
			else
				insertHelper(node->right, depth+1, point, id);
		}
		
	}	

	void insert(std::vector<float> point, int id)
	{
		//create a new node and place within the root 
		insertHelper(root,0,point,id);


	}


	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node!=NULL)
		{
			float x_point = node->point[0];
			float y_point = node->point[1];
			float z_point = node->point[2];
			float x_target = target[0];
			float y_target = target[1];
			float z_target = target[2];

			if( (x_point >= x_target-distanceTol) && (x_point <= x_target+distanceTol) && (y_point >= y_target-distanceTol) && (y_point <= y_target+distanceTol)
			&& (z_point >= z_target-distanceTol) && (z_point <= z_target+distanceTol)) 
			{
				float x_diff = x_point - x_target;
				float y_diff = y_point - y_target;
				float distance = sqrt(x_diff * x_diff + y_diff * y_diff);
				//add node id to ids if within distance tolerance
				if(distance <= distanceTol)
					ids.push_back(node->id);	
			}
			//check across boundry
			if( (target[depth % 3]-distanceTol) < node->point[depth % 3])
				searchHelper(target,node->left, depth+1, distanceTol, ids);
			if( (target[depth % 3]+distanceTol) > node->point[depth % 3])
				searchHelper(target,node->right, depth+1, distanceTol, ids); 
		}
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




