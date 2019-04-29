
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
ros::Subscriber subPointCloud;
ros::Publisher  pubCluster;


void clusterHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromROSMsg (*laserCloudMsg,*cloudIn);

pcl::SACSegmentation<pcl::PointXYZ> seg; // 创建一个分割方法
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
// pcl::ModelCoefficients coefficients; //申明模型的参数
// pcl::PointIndices inliers; //申明存储模型的内点的索引
seg.setOptimizeCoefficients (true); // 这一句可以选择最优化参数的因子
seg.setModelType (pcl::SACMODEL_PLANE); //平面模型
seg.setMethodType (pcl::SAC_RANSAC); //分割平面模型所使用的分割方法
seg.setDistanceThreshold (0.2); //设置最小的阀值距离
seg.setInputCloud (cloudIn); //设置输入的点云
seg.segment (*inliers,*coefficients);


// 把提取出来的外点 -> ros发布出去
pcl::ExtractIndices<pcl::PointXYZ> extract; //ExtractIndices滤波器，基于某一分割算法提取点云中的一个子集
extract.setInputCloud (cloudIn);
extract.setIndices (inliers); //设置分割后的内点为需要提取的点集
extract.setNegative (true); //设置提取内点而非外点 或者相反
extract.filter (*cloud_filtered);
//再rviz上显示所以要转换回PointCloud2
sensor_msgs::PointCloud2 cloud_filteredMsg;
pcl::toROSMsg(*cloud_filtered,cloud_filteredMsg);
cloud_filteredMsg.header=laserCloudMsg->header;


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


