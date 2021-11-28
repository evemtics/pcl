#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
 
class cloudHandler
{
public:
    cloudHandler()
    : viewer("Cloud Viewer")
    {
     pcl::PointCloud<pcl::PointXYZ> cloud;
 pcl::PointCloud<pcl::PointXYZ> cloud_radius;
     pcl::io::loadPCDFile ("/home/ge/roswork/ros_vision/src/chapter10_tutorials/src/test_pcd.pcd", cloud);
 
     pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusfilter;
     radiusfilter.setInputCloud(cloud.makeShared());
     radiusfilter.setRadiusSearch(5);
     radiusfilter.setMinNeighborsInRadius(20);
    radiusfilter.filter(cloud_radius);
 
     
     viewer.showCloud(cloud_radius.makeShared());
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






