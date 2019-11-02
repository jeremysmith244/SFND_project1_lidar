// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

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

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    void Ransac(pcl::PointIndices::Ptr& inliers, typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    void Proximity(const std::vector<std::vector<float>> points, KdTree* tree, const float distanceTol, int id, std::set<int>& markedPoints, std::vector<int>& cluster);

    std::vector<std::vector<int>> EuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */