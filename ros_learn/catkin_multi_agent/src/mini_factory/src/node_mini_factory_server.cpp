/*-----------------------------------Client-----------------------------------------*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "mini_factory/Add.h"
#include "mini_factory/Greeting.h"


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I have listen: [%s]",msg->data.c_str());
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"agent_feedback_sub");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/agent_feedback",2,chatterCallback);

	ros::ServiceClient client1 = nh.serviceClient<mini_factory::Add>("/agent_task_1");
	ros::ServiceClient client2 = nh.serviceClient<mini_factory::Greeting>("/agent_task_2");
/*-----------------------------------------------------------------------------------------*/
// 创建learning_communication::AddTwoInts类型的service消息
  mini_factory::Add srv1;
  
  srv1.request.a = atoll(argv[1]);
  srv1.request.b = atoll(argv[2]);
  
  // 发布service请求，等待加法运算的应答结果
  if (client1.call(srv1))
  {
    ROS_INFO("Sum: %ld", (long int)srv1.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service mini_factory");
    return 1;
  }
/*-----------------------------------------------------------------------------------------*/
// 实例化srv，设置其request消息的内容，这里request包含两个变量，name和age，见Greeting.srv
	mini_factory::Greeting srv2;
	srv2.request.name = "world";
	srv2.request.age = 20;

	if (client2.call(srv2))
	{
		// 注意我们的response部分中的内容只包含一个变量response，另，注意将其转变成字符串
		ROS_INFO("Response from server: %s", srv2.response.feedback.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service mini_factory");
		return 1;
	}
/*-----------------------------------------------------------------------------------------*/
	ros::spin();
	return 0;
}
