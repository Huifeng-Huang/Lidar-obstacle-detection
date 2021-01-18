/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
   
  //renderPointCloud (viewer, inputcloud, "inputcloud");
  // Experiment with the ? values and find what works best
pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor->FilterCloud(inputCloud, 0.25, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f (25, 7, 2, 1));
  
  
std::pair <pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =pointProcessor->SegmentPlane(filterCloud, 20, 0.2)  ;
 std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =pointProcessor->Clustering(segmentCloud.first, 0.6,10,2000);
  
  int clusterID = 0;
  std::vector<Color> colors{Color(1,0,0),Color(0,1,0),Color(0,0,1)};
  
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster:cloudClusters)
   {  Box box =pointProcessor->BoundingBox(cluster); 
  renderBox (viewer,box,clusterID);
   
   renderPointCloud(viewer, cluster, "obstacle_cloud" +std::to_string(clusterID), colors[clusterID]);
  ++clusterID;  }
             
//renderPointCloud(viewer, segmentCloud.first, "obstacle_cloud", Color(1,0,0));

renderPointCloud(viewer, segmentCloud.second, "plane_cloud", Color(0,1,0));
  
  }
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
   Lidar* lidar= new Lidar(cars,0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = lidar->scan();
  //renderRays(viewer, lidar-> position,input_cloud);
// renderPointCloud (viewer, input_cloud, "input_cloud");
    // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  std::pair <pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =pointProcessor.SegmentPlane (input_cloud, 100,0.2)  ;
 // renderPointCloud(viewer, segmentCloud.first, "obstacle_cloud", Color(1,0,0));
                  //   renderPointCloud(viewer, segmentCloud.second, "plane_cloud", Color(0,1,0));
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =pointProcessor.Clustering(segmentCloud.first, 1,3,30);
  
  int clusterID = 0;
  std::vector<Color> colors{Color(1,0,0),Color(0,1,0),Color(0,0,1)};
  
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster:cloudClusters)
     // renderPointCloud(viewer, cluster, "obstacle_cloud" +std::to_string(clusterID), colors[clusterID%colors.size()]);
  ++clusterID;
                   
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    while (!viewer->wasStopped ())
  {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
    {
      streamIterator = stream.begin();
    }

    viewer->spinOnce();
}
}