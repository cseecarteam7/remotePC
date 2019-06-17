#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cat_feeder"); // Initiate new ROS node named "talker"
	ros::NodeHandle node;
	ros::Publisher pubFeather = node.advertise<std_msgs::String>("/servo_motion", 10);
	//ros::Publisher pubTeleop = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	ros::Rate loop_rate(10);
	//geometry_msgs::Twist baseCmd;
	std_msgs::String msg;
	std::stringstream ss;
	ss << "1";


	int flag = 0;
	printf("***** PRESS '1' TO FEED CATS!!*****\n\n");
	while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
	{
		msg.data = ss.str();
		//ROS_INFO("%s", msg.data.c_str());
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
		
		scanf("%d",&flag);
		printf("f : %d\n",flag);
		if(flag == 1) {
			/**/
			printf("\n\n[cat_feeder/main] FEED CATS!\n");
			pubFeather.publish(msg);
			ros::Duration d3(3);
			d3.sleep();
		
			/**/
			/*baseCmd.linear.x = -0.1;
			baseCmd.linear.y = 0;
			baseCmd.angular.z = 0;
			pubTeleop.publish(baseCmd);
			ros::Duration d1(1);
			baseCmd.linear.x = 0;
			pubTeleop.publish(baseCmd);
			*/

			flag = 0;
		}
		else if(flag != 0 && flag != 1) {
			printf("***** PRESS '1' TO FEED CATS!!*****\n\n");
			flag = 0;
		}
		flag = 0;

	}

	return 0;
}
