#include <ros/ros.h>
#include "mini_factory/Greeting.h"


// service回调函数，输入参数req，输出参数res
bool handle(mini_factory::Greeting::Request  &req,
            mini_factory::Greeting::Response &res)
{
 
  	ROS_INFO("Request from %s with age %d ", req.name.c_str(), req.age);	
	// 返回一个反馈，将response设置为"..."
	res.feedback = "Hi " + req.name + ". I'm server!";
	
  return true;
}

int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "agent_task_2_server");
  
  // 创建节点句柄
  ros::NodeHandle n;

  // 创建一个名为add_two_ints的server，注册回调函数add()
  ros::ServiceServer service = n.advertiseService("/agent_task_2", handle);
  
  ros::spin();

  return 0;
}

