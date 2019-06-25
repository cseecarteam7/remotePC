/*
 * Title: turtlebot3_pose_lidar_avg.cpp (for hw03)
 * author: knuros05
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <float.h>
#include <stdlib.h>
#include <time.h>
#include <std_msgs/String.h>
#include <sstream>

using namespace cv;
using namespace std;

#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))
#define ROTDEGREE toRadian(80)
#define OBSTACLEDISTANCE 0.7
#define OBDEGREE 30
#define DISPLAYSIZE 501

boost::mutex mutex[2];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;


//////////***** callback functions *****//////////
template<typename T>
inline bool isnan(T value)
{
	return value != value;
}

void
odomMsgCallback(const nav_msgs::Odometry &msg)
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


//////////***** get info from odom *****//////////

tf::Transform getCurrentTransformation(void)
{
	tf::Transform transformation;
	nav_msgs::Odometry odom;

	mutex[0].lock(); {
		odom = g_odom;
	} mutex[0].unlock();

	// Save position
	transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

	// Save rotation angle
	transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

	return transformation;
}


void
saveCurrentPosition(Vec3d &xyz, vector<Vec3d> &trajectory, double dMinDist)
{
	// nSize : size of saved coordinates of route
	int nSize = (int) trajectory.size();

	if(nSize <= 0)
		trajectory.push_back(xyz); //at first, just save
	else {
		Vec3d diff = trajectory[nSize-1] - xyz;
		double len = sqrt(diff.dot(diff));
		
		//if robot moved, save cur coord
		if(len > dMinDist)
			trajectory.push_back(xyz);
	}
}


//////////***** execute rotation *****//////////

bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
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
		if( fabs(dAngleTurned) > fabs(dRotation) || (dRotation == 0)) {
			bDone = true;
			break;
		} else {
			pubTeleop.publish(baseCmd);
			loopRate.sleep();
		}

	}

	// Initialization
	baseCmd.linear.x = 0.3;
	baseCmd.linear.y = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}


//////////***** convert into coord *****//////////
	void
convertOdom2XYZRPY(nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy)
{
	// 이동 저장
	xyz[0] = odom.pose.pose.position.x;
	xyz[1] = odom.pose.pose.position.y;
	xyz[2] = odom.pose.pose.position.z;

	// 회전 저장
	tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, 
			odom.pose.pose.orientation.y, 
			odom.pose.pose.orientation.z, 
			odom.pose.pose.orientation.w);

	tf::Matrix3x3(rotationQuat).getEulerYPR(rpy[2], rpy[1], rpy[0]);
}


	void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs, double &avg)
{
	int nRangeSize = (int)lrfScan.ranges.size();
	XYZs.clear();
	XYZs.resize(nRangeSize);

	double obstacle_cnt = 0;

	for(int i=0; i < nRangeSize; i++) {
		double dRange = lrfScan.ranges[i];

		if(isnan(dRange))
			XYZs[i] = Vec3d(0., 0., 0.);
		else {
			double dAngle = lrfScan.angle_min + i * lrfScan.angle_increment;
			XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
		}


		//sum obstacle distance which are in range
		if(i <= OBDEGREE || i >= 360-OBDEGREE){
			if(!isinf(lrfScan.ranges[i])){
				if(i < 20 || i > 340)
					avg += lrfScan.ranges[i] *1.7;
				else
					avg += lrfScan.ranges[i];
				obstacle_cnt++;
			}
		}

	}

	//calculate avg of obstacle distance
	if(obstacle_cnt >0)
		avg /= obstacle_cnt;
}


	void
transform(vector<Vec3d> &laserScanXY, double x, double y, double theta)
{
	Vec3d newPt;
	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	int nRangeSize = (int)laserScanXY.size();

	for(int i=0; i<nRangeSize; i++) {
		newPt[0] = cosTheta*laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
		newPt[1] = sinTheta*laserScanXY[i][0] + cosTheta*laserScanXY[i][1] + y;
		newPt[2];
		laserScanXY[i] = newPt;
	}
}


//////////***** draw on display *****//////////
	void
initGrid(Mat &display, int nImageSize)
{
	const int nImageHalfSize = nImageSize / 2;
	const int nAxisSize = nImageSize / 16;
	const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);

	display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
}

	void
drawTrajectory(Mat &display, vector<Vec3d> &trajectory, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);

	int nSize = (int) trajectory.size();

	for(int i=0; i<nSize; i++) {
		int x = imageHalfSize[0] + cvRound((trajectory[i][0]/dMaxDist)*imageHalfSize[0]);
		int y = imageHalfSize[1] + cvRound((trajectory[i][1]/dMaxDist)*imageHalfSize[1]);

		if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
			display.at<Vec3b>(y, x) = Vec3b(0, 255, 255);
		}
	}
}

	void
drawCurrentPositionWithRotation(Mat &display, Vec3d &xyz, Vec3d &rpy, double dMaxDist)
{
	//printf("_r = %.3lf, _p = %.3lf, _y = %.3lf\n", toDegree(rpy[0]), toDegree(rpy[1]), toDegree(rpy[2]));

	const int nHeadingSize = 30;
	Vec2i headingDir = Vec2i(nHeadingSize*cos(rpy[2]), nHeadingSize*sin(rpy[2]));
	Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);

	int x = imageHalfSize[0] + cvRound((xyz[0]/dMaxDist)*imageHalfSize[0]);
	int y = imageHalfSize[1] + cvRound((xyz[1]/dMaxDist)*imageHalfSize[1]);

	if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
		circle(display, Point(x, y), nHeadingSize, CV_RGB(255, 0, 255), 1, CV_AA);
		line(display, Point(x, y), Point(x+headingDir[0], y+headingDir[1]), CV_RGB(255, 0, 255), 1, CV_AA);
	}
}

	void
drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
	int nRangeSize = (int)laserScanXY.size();

	for(int i=0; i<nRangeSize; i++) {
		int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
		int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);

		if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
			display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
		}
	}
}

	void
drawLRFScanMulti(Mat &display, vector< vector<Vec3d> > &laserScanXYMulti, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
	int nNumOfScan = (int)laserScanXYMulti.size();

	for(int i=0; i<nNumOfScan; i++) {
		int nRangeSize = (int)laserScanXYMulti[i].size();

		for(int j=0; j<nRangeSize; j++) {
			int x = imageHalfSize[0] + cvRound((laserScanXYMulti[i][j][0]/dMaxDist)*imageHalfSize[0]);
			int y = imageHalfSize[1] + cvRound((laserScanXYMulti[i][j][1]/dMaxDist)*imageHalfSize[1]);

			if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
				display.at<Vec3b>(y, x) = Vec3b(128, 128, 128);
			}
		}
	}
}


//////////***** 1 *****//////////
void getInfoFromOdomScan(nav_msgs::Odometry &odom, sensor_msgs::LaserScan &scan, Vec3d &xyz, Vec3d &rpy, vector<Vec3d> &trajectory, vector<Vec3d> &laserScanXY, double &avg) {

	/* Get current odom-info of turtlebot */
	mutex[0].lock(); {
		odom = g_odom;
	} mutex[0].unlock();
	//printf("로봇 현위치 = (%.3lf, %.3lf)\n", odom.pose.pose.position.x, odom.pose.pose.position.y);	

	// get info and save
	convertOdom2XYZRPY(odom, xyz, rpy);
	saveCurrentPosition(xyz, trajectory, 0.02);


	/* Get current scan-info of turtlebot */
	mutex[1].lock(); {
		scan = g_scan;
	} mutex[1].unlock();
	convertScan2XYZs(scan, laserScanXY, avg);
}


//////////***** 2 *****//////////
void automove(double avg, ros::Publisher &pubTeleop) 
{
	bool obstacle_flag = false;
	geometry_msgs::Twist baseCmd;

	//go forward first
	baseCmd.linear.x = 0.5;
	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;
	pubTeleop.publish(baseCmd);

	/* 4. Calculate average distance from obstacles */
	printf("장애물 (평균거리) = (%.3lf)\n", avg);
	if(avg > 0 && avg < OBSTACLEDISTANCE){
		baseCmd.linear.x = 0;
		baseCmd.linear.y = 0;
		baseCmd.angular.z = 0;
		obstacle_flag = true;			
	}
	pubTeleop.publish(baseCmd);

	double deg = 0;
	if(obstacle_flag){
		printf("--- OBSTACLE DETECTED! ---\n");
		srand(time(NULL));
		if((rand()%2) == 1)
			deg = ROTDEGREE;
		else
			deg = -ROTDEGREE;
		tf::Transform currentTransformation = getCurrentTransformation();
		doRotation(pubTeleop, currentTransformation, deg, 1.5);		
		obstacle_flag = false;

		//go forward
		baseCmd.linear.x = 0.5;
		baseCmd.linear.y = 0;
		baseCmd.angular.z = 0;
		pubTeleop.publish(baseCmd);
	}
}


//////////***** 3 *****//////////
void drawImage(Mat &display, nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy, vector<Vec3d> &trajectory, vector<Vec3d> &laserScanXY, vector< vector<Vec3d> > &laserScanXYMulti)
{
	const double dGridMaxDist = 4.0;

	// convert laserScan into coord
	transform(laserScanXY, xyz[0], xyz[1], rpy[2]);

	// draw image
	initGrid(display, DISPLAYSIZE);
	drawTrajectory(display, trajectory, dGridMaxDist);
	drawCurrentPositionWithRotation(display, xyz, rpy, dGridMaxDist);
	drawLRFScanMulti(display, laserScanXYMulti, dGridMaxDist);
	drawLRFScan(display, laserScanXY, dGridMaxDist);

	// convert 2D coord into top-viewed 3D coord
	transpose(display, display);  // X-Y축 교환
	flip(display, display, 0);  // 수평방향 반전
	flip(display, display, 1);  // 수직방향 반전

	// display
	imshow("KNU ROS Lecture >> turtle_pose_lidar_avg", display);
}



//////////***** main function *****//////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_lidar_avg");

	ros::NodeHandle nhp;
	ros::Publisher pubTeleop = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	
	ros::NodeHandle nhs;
	ros::Subscriber subOdom = nhs.subscribe("/odom", 100, &odomMsgCallback);
	ros::Subscriber subScan = nhs.subscribe("/scan", 10, &scanMsgCallback);

	nav_msgs::Odometry odom;
	sensor_msgs::LaserScan scan;
	geometry_msgs::Twist baseCmd;

	Mat display;
	initGrid(display, DISPLAYSIZE);

	Vec3d xyz, rpy;	
	vector<Vec3d> trajectory;

	vector<Vec3d> laserScanXY;
	vector< vector<Vec3d> > laserScanXYMulti;


	while(ros::ok()) {
		ros::spinOnce();
		double avg = 0;

		/* 1. get info from odom and scan */
		getInfoFromOdomScan(odom, scan, xyz, rpy, trajectory, laserScanXY, avg);
	
		/* 2. move avoiding obstacles */
		automove(avg, pubTeleop);

		/* 3. draw image to OpenCV */
		drawImage(display, odom, xyz, rpy, trajectory, laserScanXY, laserScanXYMulti);


		// wait for termination
		int nKey = waitKey(30) % 255;
		if(nKey == 27) {
			break;
		} 
		else if(nKey == ' ') {
			laserScanXYMulti.push_back(laserScanXY);
		}
		else if(nKey == 'c' || nKey == 'C') {
			initGrid(display, DISPLAYSIZE);
			laserScanXYMulti.clear();
			trajectory.clear();
		}

	} // end-of-while


	return 0;
}


