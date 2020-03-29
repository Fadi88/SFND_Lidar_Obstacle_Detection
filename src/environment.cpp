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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar * lidar_obj = new Lidar{cars, 0.0d};
    auto cloud = lidar_obj->scan();
    //renderRays(viewer , lidar_obj->position,lidar_obj->scan());
    //renderPointCloud(viewer , cloud , "Point Cloud" , {0.0f , 0.8f , 0.0f});
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processor_obj;
    auto seg_result = processor_obj.SegmentPlane(cloud , 100 , 0.2);

    //renderPointCloud(viewer,seg_result.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,seg_result.second,"planeCloud",Color(0,1,0));

    auto cloudClusters = processor_obj.Clustering(seg_result.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0.5,0,0.5), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor_obj.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = processor_obj.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
  
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>& processor_obj, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){

    //ProcessPointClouds<pcl::PointXYZI> processor_obj{};
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = processor_obj.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    auto seg_result = processor_obj.SegmentPlane(inputCloud , 100 , 0.2);
    
    auto filterCloud = processor_obj.FilterCloud(seg_result.first, 0.25 , Eigen::Vector4f (-20, -4, -3, 1), Eigen::Vector4f ( 25, 6, 2, 1));
    
    renderPointCloud(viewer,filterCloud,"filterCloud");

    
    auto cloudClusters = processor_obj.Clustering(filterCloud, .7, 30, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0.5,0,0.5), Color(0,0,1)};

    for(auto cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor_obj.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        Box box = processor_obj.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    

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

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI> pointProcessor{};

    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    while (!viewer->wasStopped ()){

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessor.loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessor, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
        streamIterator = stream.begin();

    viewer->spinOnce ();
    } 
}