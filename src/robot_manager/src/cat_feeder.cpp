#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cat_feeder");
	ros::NodeHandle node;
	ros::Publisher pubFeed = node.advertise<std_msgs::String>("/servo_motion", 10);
	ros::Publisher pubTeleop = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	ros::Rate loop_rate(10);
	
	geometry_msgs::Twist baseCmd;
	std_msgs::String msg;
	std::stringstream ss;
	ss << "1";
	msg.data = ss.str();
	
	printf("************************************\n");
	printf("***** INPUT '1' TO FEED CATS!! *****\n");
	printf("************************************\n");
	
	int flag = 0;
	while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
	{
		//ROS_INFO("%s", msg.data.c_str());
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
		
		printf("[input] : ");
		scanf("%d",&flag);
		printf("--- f : %d\n",flag);
		
		if(flag == 1) {
			printf("[cat_feeder/main] FEED CATS!!!\n\n");
			
			//stop
			baseCmd.linear.x = 0;
			baseCmd.linear.y = 0;
			baseCmd.angular.z = 0;
			pubTeleop.publish(baseCmd);

			//feed and wait
			pubFeed.publish(msg);
			ros::Duration d3(1);
			d3.sleep();
		}
		/* the others */
		else
			printf("\n***** PRESS NUMBER '1' TO FEED CATS!! *****\n\n");

		// init flag
		flag = 0;
	}

	return 0;
}
