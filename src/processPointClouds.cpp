// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
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


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
  typename pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
  sor.setInputCloud (cloud);
  sor.setLeafSize (filterRes, filterRes, filterRes);
  sor.filter (*filtered_cloud);
     typename pcl::PointCloud<PointT>::Ptr crop_cloud (new pcl::PointCloud<PointT>);
  pcl::CropBox< PointT> region(true);
  region.setInputCloud(cloud);
   region.setMax(maxPoint);
   region.setMin(minPoint);
   region.filter(*crop_cloud);




   std::vector<int> indices;
   pcl::CropBox<PointT> roof(true);
   roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
  roof.setMax(Eigen::Vector4f(2.6,1.7,-4,1));
  roof.setInputCloud(crop_cloud);
  roof.filter(indices);



pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
for (int point:indices)
inliers->indices.push_back(point);

pcl::ExtractIndices<PointT> extract;
extract.setInputCloud (crop_cloud);
extract.setIndices(inliers);
extract.setNegative (true);
extract.filter (*crop_cloud);




    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return crop_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());

  for(auto c : inliers->indices)
  {
      planeCloud->points.push_back(cloud->points[c]);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
  return segResult;
  

      
    
}


template<typename PointT> 
inline std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	int num_pts = cloud->points.size();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
  
	for(size_t i = 0; i < maxIterations; i++)
	{
		PointT pt1;
		PointT pt2;
		PointT pt3;

		std::unordered_set<int> inliersTemp;

		while(inliersTemp.size() < 3)
		{
			inliersTemp.insert(rand() % num_pts);
		}

		auto it = inliersTemp.begin();
		pt1 = cloud->points[*it];
		it++;
		pt2 = cloud->points[*it];
		it++;
		pt3 = cloud->points[*it];
      
		float x1 = pt1.x, x2 = pt2.x, x3 = pt3.x;
		float y1 = pt1.y, y2 = pt2.y, y3 = pt3.y;
		float z1 = pt1.z, z2 = pt2.z, z3 = pt3.z;
	
		// A is equivalent to i, the x component of the normal vector of the plane formed by pt1, pt2, and pt3
		// B is equivalent to j, the y component
		// C is equivalent to k, the z component
		double A = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		double B = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
		double C = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
		double D = -(A * x1 + B * y1 + C * z1);
		
		for(size_t j = 0; j < num_pts; j++)
		{
			if(inliersTemp.count(j) > 0) continue;

			auto other = cloud->points[j];
			auto x4 = other.x;
			auto y4 = other.y;
			auto z4 = other.z;
          
			double distance = fabs(A * x4 + B * y4 + C * z4 + D) / sqrt(A*A + B*B + C*C);

			if(distance <= distanceTol)
			{
				inliersTemp.insert(j);
			}
		}

		if(inliersTemp.size() > inliersResult.size())
		{
			inliersResult = inliersTemp;
		}
	}

	return inliersResult;
}
    

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  // TODO:: Fill in this function to find inliers for the cloud.

  std::unordered_set<int> ransacResult = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
  inliers->indices.insert(inliers->indices.end(), ransacResult.begin(), ransacResult.end());

  if(inliers->indices.size() == 0) std::cout << "Could not estimate a planar model for the dataset." << std::endl;

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
    
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (clusterTolerance); // 2cm
  ec.setMinClusterSize (minSize);
  ec.setMaxClusterSize (maxSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

 for(pcl::PointIndices getIndices: cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster ( new pcl::PointCloud<PointT>);
        for(int index: getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

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