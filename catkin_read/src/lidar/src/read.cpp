#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace boost;
using namespace pcl;
using namespace pcl::console;
ros::Publisher pointCloud_pub;

//void sysUsecTime(SystemTime *OutputTime);

double getMin(double x, double y)
{
    if( x <= y )
        return x;
    else
        return y;
}

double getMax(double x, double y)
{
    if( x >= y )
        return x;
    else
        return y;
}

/*typedef struct tarCoorBucket
{
    int x;
    int y;
} CoorBucket;
*/
typedef struct tarSystemTime
{
    int hour;
    int minute;
    int second;
    long int u_second;
} SystemTime;


void laserCloudHandle(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
{
	ROS_INFO("Continuous acceptance and processing");
	
	//sensor_msgs::PointCloud2 output;
    //output = *pointCloudMsg;
    pcl::PointCloud<pcl::PointXYZI> cloud_in;
     
    pcl::fromROSMsg(*pointCloudMsg,cloud_in);
    
    //std::cout<<'size:  '<<cloud_in.points.size()<<std::endl; 
        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr conver_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr converXY_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr julei_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    /*pcl::PointCloud<pcl::PointXYZI>::ConstPtr CloudProcess( pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud )
    {*/

        //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
       

        for(unsigned long i=0; i<cloud_in.points.size(); i++)
        {
            if( cloud_in.points[i]._PointXYZI::intensity > 20 && cloud_in.points[i]._PointXYZI::y>0 )
            {
                filter_cloud->points.push_back(cloud_in.points[i]);
            }
        }

        pcl::PointXYZI point_XYZ;
        pcl::PointXYZI  Point_XY;
        for( unsigned long i=0; i<filter_cloud->points.size(); i++ )
        {
            point_XYZ._PointXYZI::x = filter_cloud->points[i]._PointXYZI::x;
            point_XYZ._PointXYZI::y = filter_cloud->points[i]._PointXYZI::y;
            point_XYZ._PointXYZI::z = filter_cloud->points[i]._PointXYZI::z;
            conver_cloud->points.push_back(point_XYZ);

            Point_XY.PointXYZI::x = filter_cloud->points[i]._PointXYZI::x;
            Point_XY.PointXYZI::y = filter_cloud->points[i]._PointXYZI::y;
            Point_XY.PointXYZI::z = 0;
            converXY_cloud->points.push_back(Point_XY);

        }
        /*------基于欧式距离的聚类------------------------------------------------------------------------------------------------------*/
        pcl::search::KdTree<pcl::PointXYZI>::Ptr Tree (new pcl::search::KdTree<pcl::PointXYZI>);
        Tree->setInputCloud (converXY_cloud);//cloud_plane

        std::vector<pcl::PointIndices> cluster_indices;
        std::vector<pcl::PointIndices> deal_cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        pcl::PointCloud<pcl::PointXYZ> Tem;
        ec.setClusterTolerance (0.5); // 设置近邻搜索的搜索半径为50cm
        ec.setMinClusterSize (1);//设置一个聚类需要的最少的点数目为1
        ec.setMaxClusterSize (10000);//设置一个聚类需要的最大点数目
        ec.setSearchMethod (Tree);//设置点云的搜索机制
        ec.setInputCloud (converXY_cloud);
        ec.extract (cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中

        for(unsigned long i=0; i<cluster_indices.size(); i++)
        {
            int flag = 0;
            for( unsigned long j=0; j<cluster_indices[i].indices.size(); j++ )
            {
                if( conver_cloud->points[cluster_indices[i].indices[j]]._PointXYZI::z > 1.1 )
                {
                    flag = 1;
                    break;
                }
            }
            if( flag == 0 )
            {
                deal_cluster_indices.push_back( cluster_indices[i] );
            }
        }


        for( unsigned long i=0; i<deal_cluster_indices.size(); i++ )
        {
            double MinDistence = 10000;
            double MaxDistence = 0;
            double currentDistence = 0;
            double BucketMinHigh = 10000;
            double BucketMaxHigh = -10000;
            pcl::PointXYZI current_point;
            for(unsigned long j=0; j<deal_cluster_indices[i].indices.size(); j++)
            {
                current_point = conver_cloud->points[deal_cluster_indices[i].indices[j]];
                currentDistence = sqrt(current_point.x*current_point.x+current_point.y*current_point.y);
                MinDistence = GetMin( MinDistence, currentDistence );
                MaxDistence = GetMax( MaxDistence, currentDistence );
                BucketMinHigh = GetMin( BucketMinHigh, current_point.z );
                BucketMaxHigh = GetMax( BucketMaxHigh, current_point.z );
            }
            if( (MaxDistence - MinDistence < 0.4) && MaxDistence < 30 && (BucketMaxHigh - BucketMinHigh < 0.75) )
            {
                for( unsigned long j=0; j<deal_cluster_indices[i].indices.size(); j++ )
                    julei_cloud->points.push_back(conver_cloud->points[deal_cluster_indices[i].indices[j]]);
            }
        }
        //return julei_cloud;
		std::cout<<"size: "<<julei_cloud->points.size()<<std::endl;

/*----------------------------------------------------------------------------------------------------------*/
	//ros::NodeHandle node;

	
	sensor_msgs::PointCloud2 output_again;
	
	pcl::toROSMsg(*filter_cloud,output_again);
	output_again.header.frame_id = "outpointAgain";
	pointCloud_pub.publish(output_again);

    ROS_INFO("NO ERROR");    
}

main(int argc, char **argv)
{
    ros::init (argc, argv, "read");

    ROS_INFO("Started lidar node");
	ros::NodeHandle nh;
    
       
	ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points",2,laserCloudHandle);
	
	 pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("point_outputAgain", 12);
	
	//ros::Publisher pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("point_output", 1);
   // pcl::toROSMsg(cloud,output);
    //pcl::fromROSMsg(output,cloud_in);
    
    //output.header.frame_id = "point_cloud";

    //ros::Rate loop_rate(1);
    ros::spin();
    /*while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }*/

    return 0;
}
