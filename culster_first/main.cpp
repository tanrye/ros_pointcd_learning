
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

ros::Subscriber subPointCloud;
ros::Publisher  pubCluster;


void clusterHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromROSMsg (*laserCloudMsg,*cloudIn);

pcl::SACSegmentation<pcl::PointXYZ> seg; // ????????
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
// pcl::ModelCoefficients coefficients; //???????
// pcl::PointIndices inliers; //????????????
seg.setOptimizeCoefficients (true); // ???????????????
seg.setModelType (pcl::SACMODEL_PLANE); //????
seg.setMethodType (pcl::SAC_RANSAC); //??????????????
seg.setDistanceThreshold (0.2); //?????????
seg.setInputCloud (cloudIn); //???????
seg.segment (*inliers,*coefficients);


// ???????? -> ros????
pcl::ExtractIndices<pcl::PointXYZ> extract; //ExtractIndices??????????????????????
extract.setInputCloud (cloudIn);
extract.setIndices (inliers); //????????????????
extract.setNegative (true); //?????????? ????
extract.filter (*cloud_filtered);
//?rviz?????????PointCloud2
sensor_msgs::PointCloud2 cloud_filteredMsg;
pcl::toROSMsg(*cloud_filtered,cloud_filteredMsg);
cloud_filteredMsg.header = laserCloudMsg->header;
//cloud_filteredMsg.header.frame_id=laserCloudMsg.frame_id;

pubCluster.publish (cloud_filteredMsg);

}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "cluster");
    ROS_INFO("Started cluster node");
    ros::NodeHandle nh;

    subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points",2,clusterHandler);

    pubCluster = nh.advertise<sensor_msgs::PointCloud2> ("point_cluster", 12);

    ros::spin();


    return 0;
}

