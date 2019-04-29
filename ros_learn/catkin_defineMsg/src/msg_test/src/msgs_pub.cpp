#include "ros/ros.h"
#include "msg_test/Path.h"
#include "msg_test/test.h"
#include <iostream>
using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "custom_msgs_talker");

  ros::NodeHandle nh;

//-----------define custom messages Publisher--------------
 // ros::Publisher pub_can_data = nh.advertise<msg_test::Path>("can_data", 10);
  ros::Publisher pub_can_data = nh.advertise<msg_test::test>("can_data", 1);
  
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
//----------initialize custom messages and publish---------
    msg_test::test output;
    
    /*output.header.seq = count;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "can_data";
    output.id = 0x101;
    output.len = 2;
    output.data[0] = 10;
    output.data[1] = 20;
    output.data[2] = 30;
    output.data[3] = 40;
    output.x = 5;
    output.y = 6;
    */
    output.temp = 100;
    pub_can_data.publish(output);
    //printf("len %d\n",output.temp);
cout<<unsigned(output.temp)<<endl;
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }


  return 0;
}
