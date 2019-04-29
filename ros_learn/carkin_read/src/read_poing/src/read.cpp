#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>

main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_read");

    ROS_INFO("Started lidar node");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 2);

    sensor_msgs::PointCloud2 output;

    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl::io::loadPCDFile ("1539766001.251637000.pcd", cloud);

    pcl::toROSMsg(cloud,output);
    
    output.header.frame_id = "point_cloud";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
