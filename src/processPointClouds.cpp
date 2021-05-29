// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include "kdtree.h"



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


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);

    //voxel filter
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (filterRes, filterRes, filterRes);
    vox.filter (*cloud_filtered);

    //region filtering
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    // remove roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    for (int i: indices)
        inliers->indices.push_back(i);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_region);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_region);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}
/*
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>()); 
    typename pcl::PointCloud<PointT>::Ptr road (new pcl::PointCloud<PointT>());

    for(int index : inliers->indices)
    {
        road->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}
*/

//plane segmentation using Ransac from scratch
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while(maxIterations--)
	{	
        std::unordered_set<int> inliers;
		//pick 3 random points to fit plane to and add to unsorted set 'inliers'
		while (inliers.size()<3)
		 	inliers.insert(rand() % cloud->points.size());
		
		//const float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		//start at first point in inlier and dereferance to get value
		auto itr = inliers.begin();
		const float x1 = cloud -> points[*itr].x;
		const float y1 = cloud -> points[*itr].y;
		const float z1 = cloud -> points[*itr].z;
		//do same for second and third point
		itr++;
		const float x2 = cloud -> points[*itr].x;
		const float y2 = cloud -> points[*itr].y;
		const float z2 = cloud -> points[*itr].z;
		itr++;
		const float x3 = cloud -> points[*itr].x;
		const float y3 = cloud -> points[*itr].y;
		const float z3 = cloud -> points[*itr].z;

		//vector v1 travels from point 1 to point 2
		//const float v1 [3] = {x2-x1, y2-y1 ,z2-z1};
		//vector v2 travels from point 1 to point 3
		//const float v2 [3] = {x3-x1, y3-y1 ,z3-z1};

		//taking cross product of v1 and v2 to find normal vector to the plane
		const float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		const float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		const float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

		const float A = i;
		const float B = j;
		const float C = k;
		const float D = -(i*x1 + j*y1 + k*z1);
		
		//itterate through all points in cloud
		for(int i = 0; i < cloud->points.size(); i++)
		{
			//if current index is of 2 points in index, ignore them
			if(inliers.count(i)>0)
				continue;

			//get x,y,z of point
			PointT point_tmp = cloud->points[i];
			float x4 = point_tmp.x;
			float y4 = point_tmp.y;
			float z4 = point_tmp.z;

			//calculate distance from plane
			const float d = fabs(A*x4 + B*y4 + C*z4 + D) / sqrt(A*A + B*B + C*C);
			if(d <= distanceThreshold)
				inliers.insert(i);

		}

		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}
    typename pcl::PointCloud<PointT>::Ptr roadCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr carCloud(new pcl::PointCloud<PointT>());

    //seperate inliers and outliers into respective point clouds
    if(!inliersResult.empty())
    {
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointT point = cloud->points[i];

            if(inliersResult.count(i))
                roadCloud->points.push_back(point);
            else
                carCloud->points.push_back(point); 
        }
    }
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(roadCloud, carCloud);

    return segResult;


}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>()); 
    typename pcl::PointCloud<PointT>::Ptr road (new pcl::PointCloud<PointT>());

    for(int index : inliers->indices)
    {
        road->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}

//plane segmentation using pcl functions
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coeffecients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    //segment road from input point cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeffecients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate planar model for this dataset" << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


template <typename PointT>
void ProcessPointClouds<PointT>::proximity(int i, const std::vector<std::vector<float>> &points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol)
{
	processed[i] = true;
	cluster.push_back(i);

	std::vector<int> nearbyPoints = tree->search(points[i], distanceTol);

	for(int j : nearbyPoints)
	{
		if(!processed[j])
			proximity(j, points, cluster, processed, tree, distanceTol);
	}

}
template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
{

	//return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(),false);

	for(int i=0; i < points.size(); i++)
	{
		if(processed[i])
			continue;
		
		std::vector<int> cluster;
		proximity(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
	}
  
	return clusters;

}

//clustering from scratch
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, const float clusterTolerance, const int minSize, const int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    KdTree* tree = new KdTree;
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<float>> points_vector;

    for (int i=0; i<cloud->points.size(); i++) 
    {
        const std::vector<float> p = {cloud->points[i].x, cloud->points[i].y,cloud->points[i].z};
        tree->insert(p, i);
        points_vector.push_back(p);
    }


    std::vector<std::vector<int>> indicies = euclideanCluster(points_vector, tree, clusterTolerance);
    
    for(const auto getIndices : indicies)
    {
        if(getIndices.size() > minSize && getIndices.size() < maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster{new pcl::PointCloud<PointT>};


            for(const auto index : getIndices)
            {
              cloud_cluster->points.push_back(cloud->points[index]);
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

//clustering using pcl functions
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree -> setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    
    for(pcl::PointIndices getIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster -> points.push_back(cloud -> points[index]);
        
        clusters.push_back(cloudCluster);
    }

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