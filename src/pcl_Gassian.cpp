#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/search/kdtree.h>
class cloudHandler
{
public:
    cloudHandler()
    : viewer("Cloud Viewer")
    {
     pcl::PointCloud<pcl::PointXYZ> cloud;
 pcl::PointCloud<pcl::PointXYZ> cloud_gaussian;
     pcl::io::loadPCDFile ("/home/ge/roswork/ros_vision/src/chapter10_tutorials/src/test_pcd.pcd", cloud);
     pcl::filters::GaussianKernel<pcl::PointXYZ,pcl::PointXYZ> kernel;
     kernel.setSigma(4);
     kernel.setThresholdRelativeToSigma(4);
     kernel.setThreshold(0.05);
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
     tree->setInputCloud(cloud.makeShared());
     pcl::filters::Convolution3D<pcl::PointXYZ,pcl::PointXYZ,pcl::filters::GaussianKernel<pcl::PointXYZ,pcl::PointXYZ>>convolution;
     convolution.setKernel(kernel);
     convolution.setInputCloud(cloud.makeShared());
     convolution.setNumberOfThreads(8);
     convolution.setSearchMethod(tree);
     convolution.setRadiusSearch(0.01);
     convolution.convolve(cloud_gaussian);
 
     
     viewer.showCloud(cloud_gaussian.makeShared());
     viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);
    }
 
    void timerCB(const ros::TimerEvent&)
    {
        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }
 
protected:
    ros::NodeHandle nh;
    
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
};
 
main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_radius");
    cloudHandler handler;
    ros::spin();
    return 0;
}






