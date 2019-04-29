#include<ros/ros.h>
#include<std_msgs/String.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I have listen: [%s]",msg->data.c_str());
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"topic_sub");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("bibi",1000,chatterCallback);

	ros::spin();
	return 0;
}
