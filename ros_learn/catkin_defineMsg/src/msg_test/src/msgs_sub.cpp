#include "ros/ros.h"
#include "msg_test/Path.h"
#include "msg_test/test.h"
#include <iostream>

//--------subscribe std_msgs/String messages----------
void customMsgCallback(const msg_test::test msg)
{
    /*printf("Header/\n");
    printf("  seq %d\n",msg.header.seq);
    printf("  frame_id %s\n",msg.header.frame_id.c_str());
    printf("id %d\n",msg.id);
    printf("len %d\n",msg.len);
    printf("data [%d,%d,%d,%d]\n",msg.data[0],msg.data[1],msg.data[2],msg.data[3]);
    printf("x %f\n",msg.x);
    printf("y %f\n",msg.y);*/
    //cout<<msg.temp<<endl;
    printf("len %d\n",msg.temp);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "custom_msgs_listener");
  
  ros::NodeHandle nh;
  
  ros::Subscriber sub_string = nh.subscribe("can_data", 10, customMsgCallback);

  
  ros::spin();

  return 0;
}
