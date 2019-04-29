#include<ros/ros.h>
#include<std_msgs/String.h>
#include<iostream>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"topic_pub");
	ros::NodeHandle nh;
	ros::Publisher topic_pub = nh.advertise<std_msgs::String>("bibi",100);//发布消息的缓冲区大小
	ros::Rate loop_rate(1);//目的是控制消息发布的频率，如果频率过快，发布者发布(publish)的消息数据就会很快填满消息队列(也就是缓冲区)，而将消息发送给订阅者节点这个通信过程是需要时间的. 
//ros::rate 是单位时间1s发布消息的次数
	while(ros::ok())
	{
		std_msgs::String msg;
		msg.data = "hello comunicate_ros2";
		topic_pub.publish(msg);
		loop_rate.sleep();
	}
return 0;
}
