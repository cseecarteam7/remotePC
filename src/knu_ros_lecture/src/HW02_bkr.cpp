/*
2016114418 bae keun ryeong
HW02
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include <opencv2/opencv.hpp>
#include <iomanip>

using namespace cv;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))

template<typename T>
inline bool isnan(T value)
{
	return value != value;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variable
boost::mutex mutex[2];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;
float pre_dAngleTurned;


Mat display;
double robot[10][2];
FILE *fp;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
	// receive a '/odom' message with the mutex
	mutex[0].lock(); {
		g_odom = msg;
	} mutex[0].unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	// receive a '/scan' message with the mutex
	mutex[1].lock(); {
		g_scan = msg;
	} mutex[1].unlock();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
saveCurrentPosition(Vec3d &xyz, vector<Vec3d> &trajectory, double dMinDist)
{
	int nSize = (int)trajectory.size();

	if (nSize <= 0) {
		trajectory.push_back(xyz);
	}
	else {
		Vec3d diff = trajectory[nSize - 1] - xyz;
		double len = sqrt(diff.dot(diff));

		if (len > dMinDist) {
			trajectory.push_back(xyz);
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
transform(vector<Vec3d> &laserScanXY, double x, double y, double theta)
{
	Vec3d newPt;
	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	int nRangeSize = (int)laserScanXY.size();

	for (int i = 0; i<nRangeSize; i++) {
		newPt[0] = cosTheta * laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
		newPt[1] = sinTheta * laserScanXY[i][0] + cosTheta * laserScanXY[i][1] + y;
		newPt[2];
		laserScanXY[i] = newPt;
	}
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
initGrid(Mat &display, int nImageSize)
{
	const int nImageHalfSize = nImageSize / 2;
	const int nAxisSize = nImageSize / 16;
	const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
	display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
drawTrajectory(Mat &display, vector<Vec3d> &trajectory, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols / 2, display.rows / 2);

	int nSize = (int)trajectory.size();

	for (int i = 0; i<nSize; i++) {
		int x = imageHalfSize[0] + cvRound((trajectory[i][0] / dMaxDist)*imageHalfSize[0]);
		int y = imageHalfSize[1] + cvRound((trajectory[i][1] / dMaxDist)*imageHalfSize[1]);

		if (x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
			display.at<Vec3b>(y, x) = Vec3b(0, 255, 255);

		}
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
drawCurrentPositionWithRotation(Mat &display, Vec3d &xyz, Vec3d &rpy, double dMaxDist)
{
	//printf("_r = %.3lf, _p = %.3lf, _y = %.3lf\n", toDegree(rpy[0]), toDegree(rpy[1]), toDegree(rpy[2]));

	const int nHeadingSize = 30;
	Vec2i headingDir = Vec2i(nHeadingSize*cos(rpy[2]), nHeadingSize*sin(rpy[2]));
	Vec2i imageHalfSize = Vec2i(display.cols / 2, display.rows / 2);

	int x = imageHalfSize[0] + cvRound((xyz[0] / dMaxDist)*imageHalfSize[0]);
	int y = imageHalfSize[1] + cvRound((xyz[1] / dMaxDist)*imageHalfSize[1]);

	if (x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
		circle(display, Point(x, y), nHeadingSize, CV_RGB(255, 0, 255), 1, CV_AA);
		line(display, Point(x, y), Point(x + headingDir[0], y + headingDir[1]), CV_RGB(255, 0, 255), 1, CV_AA);
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A callback function. Executed eack time a new pose message arrives.
void
drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols / 2, display.rows / 2);
	int nRangeSize = (int)laserScanXY.size();

	for (int i = 0; i<nRangeSize; i++) {
		int x = imageHalfSize[0] + cvRound((laserScanXY[i][0] / dMaxDist)*imageHalfSize[0]);
		int y = imageHalfSize[1] + cvRound((laserScanXY[i][1] / dMaxDist)*imageHalfSize[1]);

		if (x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
			display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
		}
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A callback function. Executed eack time a new pose message arrives.
void
drawLRFScanMulti(Mat &display, vector< vector<Vec3d> > &laserScanXYMulti, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols / 2, display.rows / 2);
	int nNumOfScan = (int)laserScanXYMulti.size();

	for (int i = 0; i<nNumOfScan; i++) {
		int nRangeSize = (int)laserScanXYMulti[i].size();

		for (int j = 0; j<nRangeSize; j++) {
			int x = imageHalfSize[0] + cvRound((laserScanXYMulti[i][j][0] / dMaxDist)*imageHalfSize[0]);
			int y = imageHalfSize[1] + cvRound((laserScanXYMulti[i][j][1] / dMaxDist)*imageHalfSize[1]);

			if (x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
				display.at<Vec3b>(y, x) = Vec3b(128, 128, 128);
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// odom으로부터 현재의 변환행렬 정보를 리턴!
tf::Transform
getCurrentTransformation(void)
{
	// transformation 버퍼
	tf::Transform transformation;

	// odom 버퍼
	nav_msgs::Odometry odom;

	// copy a global '/odom' message with the mutex
	mutex[0].lock(); {
		odom = g_odom;
	} mutex[0].unlock();

	// 위치 저장
	transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

	// 회전 저장
	transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

	// 리턴
	return transformation;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
convertOdom2XYZRPY(nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy)
{
	// 이동 저장
	xyz[0] = odom.pose.pose.position.x;
	xyz[1] = odom.pose.pose.position.y;
	xyz[2] = odom.pose.pose.position.z;

	// 회전 저장
	tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	tf::Matrix3x3(rotationQuat).getEulerYPR(rpy[2], rpy[1], rpy[0]);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
{
	int nRangeSize = (int)lrfScan.ranges.size();
	XYZs.clear();
	XYZs.resize(nRangeSize);

	for (int i = 0; i<nRangeSize; i++) {
		double dRange = lrfScan.ranges[i];

		if (isnan(dRange)) {
			XYZs[i] = Vec3d(0., 0., 0.);
		}
		else {
			double dAngle = lrfScan.angle_min + i * lrfScan.angle_increment;
			XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 이동
bool
doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed)
{
	//Vec3d curRot, curPos;
	//the command will be to go forward at 'translationSpeed' m/s
	geometry_msgs::Twist baseCmd;
	sensor_msgs::LaserScan scan;
	nav_msgs::Odometry odom;
	if (dTranslation < 0) {
		baseCmd.linear.x = -dTranslationSpeed;
	}
	else {
		baseCmd.linear.x = dTranslationSpeed;
	}

	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;

	// 이동하면서 현재위치에 대한 odometry 메시지 받기
	bool bDone = false;
	ros::Rate loopRate(1000.0);
	//sensor_msgs::LaserScan scan;
	while (ros::ok() && !bDone) {
		// 일단 callback 메시지를 받고!
		ros::spinOnce();

		// get current transformation
		tf::Transform currentTransformation = getCurrentTransformation();
		
		mutex[0].lock(); {
			odom = g_odom;
		} mutex[0].unlock();
		printf("######################################\n");
		printf("Robot :: %.3lf, %.3lf \n", odom.pose.pose.position.x, odom.pose.pose.position.y);
		/*
		convertOdom2XYZ(odom, curPos, curRot);
		*/
		//see how far we've traveled
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation;
		double dDistMoved = relativeTransformation.getOrigin().length();
//		printf("dDistMoved :: %lf\n", dDistMoved);
//		printf("dTranslation :: %lf\n",dTranslation);
		// 종료조건 체크

		if (fabs(dDistMoved) >= fabs(dTranslation)) {
			bDone = true;
			break;
		}
		else {
			//send the drive command
			pubTeleop.publish(baseCmd);

			// sleep!
			loopRate.sleep();
		}

		Vec3d xyz, rpy;
		vector<Vec3d> trajectory;


		vector<Vec3d> laserScanXY;
		vector< vector<Vec3d> > laserScanXYMulti;
		const double dGridMaxDist = 4.0;
		
		mutex[1].lock();{
			scan = g_scan;
		}mutex[1].unlock();
		convertScan2XYZs(scan, laserScanXY);
		transform(laserScanXY, xyz[0], xyz[1], rpy[2]);

		initGrid(display, 801);
		drawTrajectory(display, trajectory, dGridMaxDist);
		drawCurrentPositionWithRotation(display, xyz, rpy, dGridMaxDist);
		drawLRFScanMulti(display, laserScanXYMulti, dGridMaxDist);
		drawLRFScan(display, laserScanXY, dGridMaxDist);

		transpose(display, display);
		flip(display, display, 0);
		flip(display, display, 1);
		imshow("KNU ROS Lecture >> turtle_pose_lidar_avg", display);

		// wait for termination
		int nKey = waitKey(30) % 255;
		if (nKey == 27) {
			break;
		}
		else if (nKey == ' ') {
			laserScanXYMulti.push_back(laserScanXY);
		}
		else if (nKey == 'c' || nKey == 'C') {
			initGrid(display, 801);
			laserScanXYMulti.clear();
			trajectory.clear();
		}
	}

	//  초기화

	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 로봇이 멈춰있는 상태(처음 상태)의 위치를 저장!
tf::Transform
getInitialTransformation(void)
{
	// tf 변환행렬
	tf::Transform transformation;

	// 처음위치에 대한 odometry 메시지 받기
	ros::Rate loopRate(1000.0);

	while (ros::ok()) {
		// 일단 callback 메시지를 받고!
		ros::spinOnce();

		// get current transformationreturn;
		transformation = getCurrentTransformation();

		// 메시지를 받았으면 break!
		if (transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
			break;
		}
		else {
			loopRate.sleep();
		}
	}

	// 리턴
	return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 회전실행
bool
doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
	//the command will be to turn at 'rotationSpeed' rad/s
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;

	if (dRotation < 0.) {
		baseCmd.angular.z = -dRotationSpeed;
	}
	else {
		baseCmd.angular.z = dRotationSpeed;
	}

	// 이동하면서 현재위치에 대한 odometry 메시지 받기
	bool bDone = false;
	ros::Rate loopRate(1000.0);



	while (ros::ok() && !bDone) {
		// 일단 callback 메시지를 받고!

		ros::spinOnce();

		// get current transformation
		tf::Transform currentTransformation = getCurrentTransformation();

		//see how far we've traveled
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation;
		tf::Quaternion rotationQuat = relativeTransformation.getRotation();



		double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]), (1 - (2 * (rotationQuat[2] * rotationQuat[2]))));

		// 종료조건 체크

		if (fabs(dAngleTurned) > fabs(dRotation) ||  (dRotation == 0))
		{
			bDone = true;
			break;
		}
		else {
			pre_dAngleTurned = dAngleTurned;
			//send the drive command
			pubTeleop.publish(baseCmd);

			// sleep!
			loopRate.sleep();
		}
	}


	baseCmd.linear.x = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}






////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main(int argc, char **argv)
{
	// ROS 초기화
	ros::init(argc, argv, "turtle_position_move");

	// Ros initialization
	ros::NodeHandle nhp, nhs;

	// Decleation of subscriber
	ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);
	ros::Subscriber subScan = nhp.subscribe("/scan", 10, &scanMsgCallback);
	// Create a publisher object
	ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	//GETTING INFO FROM FILE
	if (!(fp = fopen("/home/sunny/input_path.txt", "r"))) {
		printf("wrong file\n");
		return 1;
	}
	int file_cnt, i;
	fscanf(fp, "%d", &file_cnt);
	for (i = 0; i < file_cnt; i++) {
		fscanf(fp, "%lf %lf", &robot[i][0], &robot[i][1]);
	}

	initGrid(display, 801);


	//initGrid(display, 801);
	// 파라미터 받아오기
	/*
	double dRotation = atof(argv[1]);
	float _dRatation = (float)((int)dRotation % 360);
	double dTranslation = atof(argv[2]);


	if (abs(_dRatation) > 180) {
		if (dRotation > 0) dRotation = -(360 - _dRatation);
		else dRotation = (360 + _dRatation);
	}
	else
		dRotation = _dRatation;

	*/
	double point_x, point_y;
	double prev_deg=0, cur_deg;
	double dRotation, dTranslation, d_Rotation;
	for (i = 0; i < file_cnt; i++) {
		
		printf("Going... :: %.3lf, %.3lf \n", robot[i][0], robot[i][1]);
		// 로봇이 멈춰있는 상태(처음 상태)의 변환행렬 가져오기
		tf::Transform initialTransformation = getInitialTransformation();
		tf::Quaternion initialQuaternion = initialTransformation.getRotation();
		point_x = robot[i][0] - initialTransformation.getOrigin().x();
		point_y = robot[i][1] - initialTransformation.getOrigin().y();
		dTranslation = sqrt(pow(point_x, 2.0) + pow(point_y, 2.0));

//		cur_deg = atan2((2 * initialQuaternion[2] * initialQuaternion[3]), (1 - (2 * (initialQuaternion[2] * initialQuaternion[2])))) * 180 / M_PI;
		cur_deg = atan2(robot[i][1]-initialTransformation.getOrigin().getY(), robot[i][0]-initialTransformation.getOrigin().getX());
		
		printf("Rotation :: %lf\n", cur_deg);
ros::Duration(0.5,0).sleep();
		doRotation(pub, initialTransformation, cur_deg - prev_deg, 0.75);
		//doRotation(pub, initialTransformation, 3.14*2, 0.75);
		prev_deg = cur_deg;
		// 이동
		doTranslation(pub, initialTransformation, dTranslation, 0.25);
		ros::Duration(0.5,0).sleep();
		printf("Finish!! \n");
	}
	return 0;
}
