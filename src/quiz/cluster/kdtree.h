/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>

using namespace std;

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

	void insert(std::vector<float> point, int id)
	{
		insertHelper(root, 0, point, id);
	}

	void insertHelper(Node*& node, uint depth, std::vector<float> point, int id)
	{
		int ndims = point.size();
		// tree is empty
		if(node==NULL) {
			node = new Node(point, id);
		} else {
			// calculate current dim
			uint cd = depth % ndims;

			if ( point[cd] < (node->point[cd]) ) {
				insertHelper(node->left, depth+1, point, id);
			} else {
				insertHelper(node->right, depth+1, point, id);
			}
		}
	}

	bool inBox(std::vector<float> target, std::vector<float> check, float distanceTol)
	{
		bool inside = true;
		for (int i = 0; i < target.size(); i++){
			if ( fabs(target[i] - check[i]) > distanceTol ) {
				inside = false;
			}
		}
		return inside;
	}

	bool inRadius(std::vector<float> target, std::vector<float> check, float distanceTol)
	{
		float distance = 0;
		float squareDistance = 0;
		int ndims = target.size();
		for (int i = 0; i < ndims; i++){
			squareDistance += pow( (target[0] - check[0]), 2 );
		}
		
		distance = sqrt(squareDistance);
		if ( distance < distanceTol ) {
			return true;
		} else {
			return false;
		}
	}
	
	void helpSearch(std::vector<int>& ids, Node* node, float distanceTol, std::vector<float> target, int depth)
	{
		int ndims = target.size();
		int cd = depth % ndims;
		if (node==NULL) {

		} else if ( inBox(target, node->point, distanceTol) ) {
			if ( inRadius(target, node->point, distanceTol) ) {
				ids.push_back(node->id);
				helpSearch(ids, node->left, distanceTol, target, depth+1);
				helpSearch(ids, node->right, distanceTol, target, depth+1);
			} else {
				helpSearch(ids, node->left, distanceTol, target, depth+1);
				helpSearch(ids, node->right, distanceTol, target, depth+1);
			}
		} else if ( fabs(target[cd] - node->point[cd]) < distanceTol ) {
			helpSearch(ids, node->left, distanceTol, target, depth+1);
			helpSearch(ids, node->right, distanceTol, target, depth+1);
		} else if ( target[cd] > node->point[cd] ) {
			helpSearch(ids, node->right, distanceTol, target, depth+1);
		} else { 
			helpSearch(ids, node->left, distanceTol, target, depth+1);
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		helpSearch(ids, root, distanceTol, target, 0);
		return ids;
	}


};




