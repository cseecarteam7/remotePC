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
int flag = 0;

void reqFeedMsgCallBack(const std_msgs::String::ConstPtr& msg)
{
	if(!strcmp(msg->data.c_str(),"1"))
		flag = 2;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cat_feeder");
	ros::NodeHandle pubNode, subNode;
	ros::Publisher pubFeed = pubNode.advertise<std_msgs::String>("/servo_motion", 10);
	ros::Publisher pubTeleop = pubNode.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	ros::Subscriber subReqFeed = subNode.subscribe("/req_feed", 10, &reqFeedMsgCallBack);
	ros::Rate loop_rate(10);
	
	geometry_msgs::Twist baseCmd;
	std_msgs::String msg;
	
	std::stringstream case0, case1, case2;
	case0 << "0"; //send to robot_manager
	case1 << "1"; //send to robot_manager
	case2 << "2"; //send to turtlebot_servermotor

	/* turtlebot_servo_motion ) 
			sub 2 : feed  
			sub 3 : feather
	*/

	printf("************************************************\n");
	printf("***** INPUT NUMBER TO CONTROL FEEDING MODE *****\n");
	printf("************************************************\n");
	printf("*           0 : TURN OFF FEEDING MODE          *\n");
	printf("*           1 : TURN ON FEEDING MODE           *\n");
	printf("*           2 : FEED CATS JUST NOW             *\n");
	printf("************************************************\n\n");
	
	
	while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
	{
		//ROS_INFO("%s", msg.data.c_str());
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
		
		printf("[input] : ");
		scanf("%d",&flag);
		printf("f : %d\n",flag);
		
		// init value
		if(flag == -1)
			continue;

		/* 0 : turn off mode */
		if(flag == 0) {
			printf("[cat_feeder/case_0] TURN OFF FEEDING MODE!!\n\n");
			msg.data = case0.str();
			pubFeed.publish(msg);
		}
		/* 1 : turn on mode */
		else if(flag == 1) {
			printf("[cat_feeder/case_1] TURN ON FEEDING MODE!!\n\n");
			msg.data = case1.str();
			pubFeed.publish(msg);
		}
		/* 2 : feed cats now */
		else if(flag == 2) {
			printf("[cat_feeder/case_2] FEED CATS NOW!!\n\n");
			
			baseCmd.linear.x = 0;
			baseCmd.linear.y = 0;
			baseCmd.angular.z = 0;
			pubTeleop.publish(baseCmd);

			//feed and wait
			msg.data = case2.str();
			pubFeed.publish(msg);
			ros::Duration d3(1);
			d3.sleep();
		}
		/* the others */
		else
			printf("\n***** PRESS NUMBER {0 1 2} TO CONTROL MODE!! *****\n\n");

		// init flag
		flag = -1;
	}

	return 0;
}
