/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	std::unordered_set<int> inliers;
	while(maxIterations--)
	{	
		//pick two random points to fit line to and add to unsorted set 'inliers'
		while (inliers.size()<2)
		 	inliers.insert(rand() % cloud->points.size());
		
		float x1,x2,y1,y2;
		//start at first point in inlier and dereferance to get value
		auto itr = inliers.begin();
		x1 = cloud -> points[*itr].x;
		y1 = cloud -> points[*itr].y;
		//do same for second point
		itr++;
		x2 = cloud -> points[*itr].x;
		y2 = cloud -> points[*itr].y;

		float a = y1 - y2;
		float b = x2 - x1;
		float c = (x1 * y2) - (x2 * y1);

		//itterate through all points in cloud
		for(int i = 0; i < cloud->points.size(); i++)
		{
			//if current index is of 2 points in index, ignore them
			if(inliers.count(i)>0)
				continue;

			//get x,y of point
			pcl::PointXYZ point = cloud->points[i];
			float x3 = point.x;
			float y3 = point.y;

			//calculate distance from line
			float d = fabs(a*x3 + b*y3 + c) / sqrt(a*a + b*b);
			if(d <= distanceTol)
				inliers.insert(i);

		}

		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}

	// return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	std::unordered_set<int> inliers;
	while(maxIterations--)
	{	
		//pick 3 random points to fit plane to and add to unsorted set 'inliers'
		while (inliers.size()<3)
		 	inliers.insert(rand() % cloud->points.size());
		
		float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		//start at first point in inlier and dereferance to get value
		auto itr = inliers.begin();
		x1 = cloud -> points[*itr].x;
		y1 = cloud -> points[*itr].y;
		z1 = cloud -> points[*itr].z;
		//do same for second and third point
		itr++;
		x2 = cloud -> points[*itr].x;
		y2 = cloud -> points[*itr].y;
		z2 = cloud -> points[*itr].z;
		itr++;
		x3 = cloud -> points[*itr].x;
		y3 = cloud -> points[*itr].y;
		z3 = cloud -> points[*itr].z;

		//vector v1 travels from point 1 to point 2
		float v1 [3] = {x2-x1, y2-y1 ,z2-z1};
		//vector v2 travels from point 1 to point 3
		float v2 [3] = {x3-x1, y3-y1 ,z3-z1};

		//taking cross product of v1 and v2 to find normall vector to the plane
		float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

		float A = i;
		float B = j;
		float C = k;
		float D = -(i*x1 + j*y1 + k*z1);
		
		//itterate through all points in cloud
		for(int i = 0; i < cloud->points.size(); i++)
		{
			//if current index is of 2 points in index, ignore them
			if(inliers.count(i)>0)
				continue;

			//get x,y,z of point
			pcl::PointXYZ point = cloud->points[i];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			//calculate distance from plane
			float d = fabs(A*x4 + B*y4 + C*z4 + D) / sqrt(A*A + B*B + C*C);
			if(d <= distanceTol)
				inliers.insert(i);

		}

		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}

	// return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 500, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
