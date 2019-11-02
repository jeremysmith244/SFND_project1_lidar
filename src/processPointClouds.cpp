// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

void Proximity(const std::vector<std::vector<float>> points, KdTree* tree, const float distanceTol, int id, std::set<int>& markedPoints, std::vector<int>& cluster)
{

	if ( (markedPoints.count(id) == 0) ) {
		markedPoints.insert(id);
		cluster.push_back(id);
		
		std::vector<int> nearbyPoints = tree->search(points[id], distanceTol);
		std::vector<int>::iterator it;

		for(it = nearbyPoints.begin(); it != nearbyPoints.end(); ++it) {
			if ( (markedPoints.count(*it) == 0) ) {
				Proximity(points, tree, distanceTol, *it, markedPoints, cluster);
			} else {
				continue;
			}
		}
	}
}

std::vector<std::vector<int>> EuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::set<int> markedPoints;
	for (int i = 0; i < points.size(); i++) {
		if (markedPoints.count(i) == 0){
			std::vector<int> cluster;
			Proximity(points, tree, distanceTol, i, markedPoints, cluster);
			clusters.push_back(cluster);
		} 
	}
	return clusters;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloud_c (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> crop(true);

    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setInputCloud(cloud);

    crop.filter(*cloud_c);

    crop.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    crop.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    crop.setInputCloud(cloud_c);
    crop.setNegative(true);
    crop.filter(*cloud_c);

    pcl::VoxelGrid<PointT> sor;

    sor.setInputCloud (cloud_c);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_f);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_f;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>), cloud_p (new pcl::PointCloud<PointT>);

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    extract.setNegative (true);
    extract.filter (*cloud_f);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Ransac(pcl::PointIndices::Ptr& inliers, typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	int a,b,c;
	double A,B,C,D,distance;
	PointT p1, p2, p3, pTest;

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while(maxIterations--)
	{
		std::unordered_set<int> inliersTest;
        int nPoints = cloud->points.size();

		a = rand() % nPoints;
		b = rand() % nPoints;
		c = rand() % nPoints;

		while(a==b)
		{
			b = rand() % nPoints;
		}

		while( (a==c) || (b==c) )
		{
			c = rand() % nPoints;
		}

		p1 = cloud->points[a];
		p2 = cloud->points[b];
		p3 = cloud->points[c];

		A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
		B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
		C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
		D = -1*(A*p1.x + B*p1.y + C*p1.z);

		for (int j=0; j<cloud->points.size(); j++)
		{
			pTest = cloud->points[j];
			distance = fabs(A*pTest.x + B*pTest.y + C*pTest.z + D)/sqrt(pow(A,2) + pow(B,2) + pow(C,2));
			if(distance < distanceTol){
				inliersTest.insert(j);
			}
		}

		if (inliersTest.size() > inliersResult.size())
		{
			inliersResult = inliersTest;
		}


	}

    for (auto it = inliersResult.begin();it != inliersResult.end(); ++it){
        inliers->indices.push_back(*it);
    }
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create new segment object, and coefficients
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::SACSegmentation<PointT> seg;

    // PCL BUILTIN
    // seg.setOptimizeCoefficients (true);
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (maxIterations);
    // seg.setDistanceThreshold (distanceThreshold);

    // // Run segment on cloud
    // seg.setInputCloud (cloud);
    // seg.segment (*inliers, *coefficients);

    // Custom

    Ransac(inliers, cloud, maxIterations, distanceThreshold);

    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;


    // Creating the KdTree object for the search method of the extraction with builtin pcl

    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud (cloud);
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance (clusterTolerance);
    // ec.setMinClusterSize (minSize);
    // ec.setMaxClusterSize (maxSize);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (cloud);
    // ec.extract (cluster_indices);

    // translate cloud to vector for using lesson euclidean distance calculation, create kdtree using kdtree.h struct        
    std::vector<std::vector<float>> points;
    KdTree* tree = new KdTree;
    for (int i=0; i < cloud->points.size(); i++)
    {
        points.push_back( {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z} );
        tree->insert(points[i],i); 
    }

    std::vector<std::vector<int>> cluster_indices;
    cluster_indices = EuclideanCluster(points, tree, clusterTolerance);

    for (std::vector<std::vector<int>>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->begin(); pit != it->end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
    

    // loop for builtin pcl

    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    // {
    //     typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>);
    //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    //     {
    //         cloud_cluster->points.push_back (cloud->points[*pit]);
    //     }
    //     cloud_cluster->width = cloud_cluster->points.size ();
    //     cloud_cluster->height = 1;
    //     cloud_cluster->is_dense = true;
    //     clusters.push_back(cloud_cluster);
    // }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}