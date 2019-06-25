/* 
 * HW02
 * 2016115113 김선희
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <stdio.h>
#include <stdlib.h>

#include<opencv2/core.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/core/matx.hpp>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/LaserScan.h>
#include<iomanip>
#include<vector>

#define toRadian(degree) ((degree) * (M_PI / 180.))
#define toDegree(radian) ((radian) * (180. / M_PI))
#define MAP_SIZE 500

using namespace std;
using namespace cv;


/** global variables **/
double x[20], y[20]; // stop points of txt file
boost::mutex mutex[2];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;
vector<Vec3d> wayPoints;
float pre_dAngleTurned;



// Functions
void odomMsgCallback(const nav_msgs::Odometry &msg);
void scanMsgCallback(const sensor_msgs::LaserScan& msg);
///
template<typename T> inline bool isnan(T value);
void convertOdom2XYZ(nav_msgs::Odometry &odom, Vec3d &curPos, Vec3d &curRot);
void convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs);
void transformScanXY(vector<Vec3d> &laserScanXY, double x, double y, double theta);
///
bool doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed, Vec3d &curPos, Vec3d &initPos, Vec3d &curRot, bool &flag, vector<Vec3d> &route, Mat &display);

bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed);

tf::Transform getCurrentTransformation(void);

tf::Transform getInitialTransformation(void);
///
void initGrid(Mat &display, int nImageSize);
void displayRoute(Mat &display, vector<Vec3d> &route, Vec3d &curRot);
void drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist);
void drawTrajectory(Mat &display, vector<Vec3d> &trajectory, Vec3d &curPos, double dMaxDist);


///////////////***** callback function *****///////////////

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
	mutex[0].lock(); {
		g_odom = msg;
	} mutex[0].unlock();
}


	void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	mutex[1].lock(); {
		g_scan = msg;
	} mutex[1].unlock();
}



///////////////***** convert into coordinates *****///////////////

template<typename T>
inline bool isnan(T value){
	return value != value;
}


//convert odom into coord
void convertOdom2XYZ(nav_msgs::Odometry &odom, Vec3d &curPos, Vec3d &curRot)
{
	// 이동 저장
	curPos[0] = odom.pose.pose.position.x;
	curPos[1] = odom.pose.pose.position.y;
	curPos[2] = odom.pose.pose.position.z;

	// 회전 저장
	tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	tf::Matrix3x3(rotationQuat).getEulerYPR(curRot[2], curRot[1], curRot[0]);

}


//convery scan into coord
	void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
{
	int nRangeSize = (int)lrfScan.ranges.size();
	XYZs.clear();
	XYZs.resize(nRangeSize);
	for(int i=0; i<nRangeSize; i++) {
		double dRange = lrfScan.ranges[i];
		if(isnan(dRange)) {
			XYZs[i] = Vec3d(0., 0., 0.);
		} else {
			double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
			XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
		}
	}
}

//3차원 point인 laserScanXY를 world coordinate로 변환하는 function
void transformScanXY(vector<Vec3d> &laserScanXY, double x, double y, double theta)
{
	Vec3d newPoint; 
	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	int nRangeSize = (int)laserScanXY.size();

	for(int i = 0 ; i < nRangeSize ; i++) {
		newPoint[0] = cosTheta*laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
		newPoint[1] = sinTheta*laserScanXY[i][0] + cosTheta*laserScanXY[i][1] + y;
		newPoint[2];
		laserScanXY[i] = newPoint;
	}
}



////////////////***** move and rotate *****////////////////
	bool
doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed, Vec3d &curPos, Vec3d &initPos, Vec3d &curRot, bool &flag, vector<Vec3d> &route, Mat &display)
{
	geometry_msgs::Twist baseCmd;
	nav_msgs::Odometry odom;
	bool bDone = false;
	ros::Rate loopRate(1000.0);

	//set command ... go forward / backward
	if(dTranslation < 0)
		baseCmd.linear.x = -dTranslationSpeed;
	else
		baseCmd.linear.x = dTranslationSpeed;
	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;


	while(ros::ok() && !bDone) {
		// Get callback messages
		ros::spinOnce();

		// get current transformation
		tf::Transform currentTransformation = getCurrentTransformation();

		/* odom */
		mutex[0].lock(); {
			odom = g_odom;
		} mutex[0].unlock();

		// get coord from odom
		convertOdom2XYZ(odom, curPos, curRot);
		// save current coord on route
		double x = curPos[0];
		double y = curPos[1];
		if(x!=0 && y!=0) {
			route.push_back(Vec3d(x,y));
			if(flag == false) { //when it's first move
				initPos[0] = x; initPos[1] = y;
				flag = true;				
			}
		}
		printf("curPos : %.2lf , %.2lf\n", curPos[0], curPos[1]);


		/* display route on opencv window  */
		displayRoute(display, route, curRot);


		/* Check termination condition */
		//see how far we've traveled
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
		double dDistMoved = relativeTransformation.getOrigin().length();

		if(fabs(dDistMoved) >= fabs(dTranslation)) {
			// save real destination point of robot when reached
			bDone = true;
			wayPoints.push_back(Vec3d(x,y,0));
			printf("------- HERE IS STOP POINT -------\n\n");
			break;
		} else {
			// keep go forward
			pubTeleop.publish(baseCmd);
			loopRate.sleep();
		}

		

	}

	// Initialization
	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);
	// sleep for 0.5 sec
	ros::Duration(0.5, 0).sleep();
	

	waitKey(30);

	return bDone;
}


bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
	//turn at 'rotationSpeed' rad/s
	geometry_msgs::Twist baseCmd;
	bool bDone = false;
	ros::Rate loopRate(1000.0);


	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;
	if(dRotation < 0.)
		baseCmd.angular.z = -dRotationSpeed;
	else
		baseCmd.angular.z = dRotationSpeed;	


	while(ros::ok() && !bDone) {
		ros::spinOnce();
		tf::Transform currentTransformation = getCurrentTransformation();

		//see how far we've traveled
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
		tf::Quaternion rotationQuat = relativeTransformation.getRotation();
		double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));


		// Check termination condition
		if( fabs(dAngleTurned) > fabs(dRotation) || (dRotation == 0)) 
		{
			bDone = true;
			break;
		} else {
			pre_dAngleTurned = dAngleTurned;
			pubTeleop.publish(baseCmd);
			loopRate.sleep();
		}
	}

	// Initialization
	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}




///////////////***** get odom info *****///////////////

/** Return current transformation matrix from odom **/
tf::Transform getCurrentTransformation(void)
{
	tf::Transform transformation;
	nav_msgs::Odometry odom;

	mutex[0].lock(); {
		odom = g_odom;
	} mutex[0].unlock();

	// Save position from odom message
	transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

	// Save rotation angle from odom message
	transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

	return transformation;
}

/** Save position when robot stopped(initial state) **/
tf::Transform getInitialTransformation(void)
{
	tf::Transform transformation;
	ros::Rate loopRate(1000.0);
	double x,y,z;

	while(ros::ok()) {
		ros::spinOnce();
		transformation = getCurrentTransformation();

		x = transformation.getOrigin().getX();
		y = transformation.getOrigin().getY();
		z = transformation.getOrigin().getZ();

		//success to get location, break
		if(x != 0. || y != 0. && z != 0.)
			break;
		else
			loopRate.sleep();
		
	}

	return transformation;
}


////////////***** draw on display *****////////////

	void
initGrid(Mat &display, int nImageSize)
{
	const int nImageHalfSize = nImageSize/2;
	const int nAxisSize = nImageSize/16;
	const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
	display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
}

/** show route of robot **/
void
displayRoute(Mat &display, vector<Vec3d> &route, Vec3d &curRot) {
	sensor_msgs::LaserScan scan;
	vector<Vec3d> laserScanXY;
	const double dGridMaxDist = 4.5;
	int routeSize = route.size();
	ros::spinOnce();

	mutex[1].lock(); {
		scan = g_scan;
	} mutex[1].unlock();
	// scan으로부터 Cartesian X-Y scan 획득
	convertScan2XYZs(scan, laserScanXY);
	// convert laserScan into 2D coordinate
	Vec3d curPos = route[routeSize-1];
	transformScanXY(laserScanXY, curPos[0], curPos[1], curRot[2]);


	// 현재 상황을 draw할 display 이미지를 생성
	initGrid(display, MAP_SIZE);
	drawLRFScan(display, laserScanXY, dGridMaxDist);
	drawTrajectory(display, route, curPos, dGridMaxDist);

	
	// 2D 영상좌표계에서 top-view 방식의 3차원 월드좌표계로 변환
	transpose(display, display); // X-Y축 교환
	flip(display, display, 0); // 수평방향 반전
	flip(display, display, 1); // 수직방향 반전

	// image 출력
	imshow("KNU ROS Lecture >> turtle_kinect_lrf_view", display);

	// 사용자의 키보드 입력을 받음!
	waitKey(30);

}


//draw scan info
void drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
	int nRangeSize = (int)laserScanXY.size();
	for(int i=0; i<nRangeSize; i++) {
		int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
		int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);
		if(x >= 0 && x < display.cols && y >= 0 && y < display.rows)
		{
			display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
		}
	}
}


//draw route of robot
void drawTrajectory(Mat &display, vector<Vec3d> &trajectory, Vec3d &curPos, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
	int nSize = (int) trajectory.size();

	for(int i=0; i<nSize; i++) {
		double x = imageHalfSize[0] + cvRound((trajectory[i][0]/dMaxDist)*imageHalfSize[0]);
		double y = imageHalfSize[1] + cvRound((trajectory[i][1]/dMaxDist)*imageHalfSize[1]);

		if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
			circle(display, Point(x, y), 1, CV_RGB(255, 255, 0), 1, CV_AA);
		}
	}

	for(int i=0; i<wayPoints.size(); i++) {
		double x = imageHalfSize[0] + cvRound((wayPoints[i][0]/dMaxDist)*imageHalfSize[0]);
		double y = imageHalfSize[1] + cvRound((wayPoints[i][1]/dMaxDist)*imageHalfSize[1]);
		circle(display,Point(x,y),5,CV_RGB(255,0,0),-1,CV_AA);
	}
}


/////////////////***** main function *****/////////////////

int main(int argc, char **argv)
{
	//
	ros::init(argc, argv, "turtle_auto_move");
	ros::NodeHandle pubNh,subNh;

	ros::Subscriber subOdom = subNh.subscribe("/odom", 100, &odomMsgCallback);
	ros::Subscriber subScan = subNh.subscribe("/scan", 10, &scanMsgCallback);
	ros::Publisher pubVel = pubNh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	//
	Mat display;
	initGrid(display, MAP_SIZE);

	Vec3d curPos, prePos, curRot;
	vector<Vec3d> route;
	bool flag = false;


	/* 1. get robot path */
	char pathFile[100] = "/home/sunny/catkin_ws/src/knu_ros_lecture/src/input_path.txt";
	FILE *fp = fopen(pathFile, "r");
	if(fp == NULL) {
		printf("E : cannot open the file.\n");
		exit(1);
	}

	int num;
	fscanf(fp, "%d", &num); //number of stop point
	for(int i = 0; i < num; i++)
		fscanf(fp, "%lf %lf", &x[i], &y[i]); //coordination of stop point
	fclose(fp);

	// velocity of robot
	double angle_vel = 0.2;
	double linear_vel = 0.25;
	double prev_angle = 0;



	/* 2. move through stop points */
	for(int i = 0; i < num; i++)
	{
		// 2-1. get current position
		tf::Transform nextTransformation = getInitialTransformation();

		// 2-2. rotation
		double gety = nextTransformation.getOrigin().getY();
		double getx = nextTransformation.getOrigin().getX();
		double angle = atan2(y[i]-gety, x[i]-getx);
		doRotation(pubVel, nextTransformation, angle-prev_angle, angle_vel);

		// 2-3. go forward
		double distance = sqrt((pow(x[i]-getx, 2.0)) + (pow(y[i]-gety, 2.0)));
		doTranslation(pubVel, nextTransformation, distance, linear_vel, curPos, prePos, curRot, flag, route, display);
		
		// save pre angle for next rotation
		prev_angle = angle;
	}


	return 0;
}
