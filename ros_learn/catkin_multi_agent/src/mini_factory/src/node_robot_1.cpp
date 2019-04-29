#include <ros/ros.h>
#include "mini_factory/Add.h"
#include "string"

// service回调函数，输入参数req，输出参数res
bool add(mini_factory::Add::Request  &req,
         mini_factory::Add::Response &res)
{
  // 将输入参数中的请求数据相加，结果放到应答变量中
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  
  return true;
}

int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "agent_task_1_server");
  
  // 创建节点句柄
  ros::NodeHandle n;


  ros::ServiceServer service = n.advertiseService("/agent_task_1", add);

  ROS_INFO("node_robot_1_server is ready state");
  ros::spin();

  return 0;
}

