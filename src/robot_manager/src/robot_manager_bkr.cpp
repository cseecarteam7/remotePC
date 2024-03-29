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
#define ROTDEGREE toRadian(60)
#define OBSTACLEDISTANCE 0.3
#define OBDEGREE 30
//#define MIN(x,y) ((x)<(y)?(x):(y))
//#define MAX(x,y) ((x)>(y)?(x):(y))

boost::mutex mutex[2];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;

bool catDetectFlag = false;
bool feedFlag = false;


	template<typename T>
inline bool isnan(T value)
{
	return value != value;
}


	void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
	// receive a '/odom' message with the mutex
	mutex[0].lock(); {
		g_odom = msg;
	} mutex[0].unlock();
}


	void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	// receive a '/odom' message with the mutex
	mutex[1].lock(); {
		g_scan = msg;
	} mutex[1].unlock();
}

void detectMsgCallback(const std_msgs::String::ConstPtr& msg)
{
	//printf("[detecMsgCallback]\n");
	//if(strcmp(msg->data.c_str(), "cat detected")==0)
	//ROS_INFO("****** %s!!! ******\n\n", msg->data.c_str());
	printf("\n[robot_manager/cat_detection] CAT DETECTED!\n");
	catDetectFlag = true;
}

void feedMsgCallback(const std_msgs::String::ConstPtr& msg)
{
	if(!strcmp(msg->data.c_str(),"1")) {
		printf("\n[robot_manager/cat_feeder] FEED CATS!\n");
		ros::Duration feeddur(3);
		feeddur.sleep();

		feedFlag = true;
	}
}

////////////////////////////////////////////////////////////////////

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


bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
	//the command will be to turn at 'rotationSpeed' rad/s
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;

	if(dRotation < 0.) {
		baseCmd.angular.z = -dRotationSpeed;
	} else {
		baseCmd.angular.z = dRotationSpeed;
	}

	// Get odometry messages of current position while moving
	bool bDone = false;
	ros::Rate loopRate(1000.0);


	while(ros::ok() && !bDone) {
		// Get callback messages
		ros::spinOnce();

		// get current transformation
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
			//send the drive command
			pubTeleop.publish(baseCmd);
			// sleep!
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

//avg -> avg A, B, C
	void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs, double &avgA, double &avgB, double &avgC)
{
	int nRangeSize = (int)lrfScan.ranges.size();
	XYZs.clear();
	XYZs.resize(nRangeSize);

	double obstacle_cntA = 0;
	double obstacle_cntB = 0;
	double obstacle_cntC = 0;

	for(int i=0; i < nRangeSize; i++) {
		double dRange = lrfScan.ranges[i];

		if(isnan(dRange)) {
			XYZs[i] = Vec3d(0., 0., 0.);
		} else {
			double dAngle = lrfScan.angle_min + i * lrfScan.angle_increment;
			XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
		}
		//rangeA(middle)
		if(i <= OBDEGREE || i >= 360-OBDEGREE){
			if(!isinf(lrfScan.ranges[i])){
				avgA += lrfScan.ranges[i];
				obstacle_cntA++;
			}
		}
		//rangeB(right)
		if( ((360-(OBDEGREE*3)) <= i) && (i<= 360-OBDEGREE )){
			if(!isinf(lrfScan.ranges[i])){
				avgB += lrfScan.ranges[i];
				obstacle_cntB++;
			}
		}
		//rangeC(left)
		if(  OBDEGREE < i && i <= OBDEGREE*3){
			if(!isinf(lrfScan.ranges[i])){
				avgC += lrfScan.ranges[i];
				obstacle_cntC++;
			}
		}
	}
	if(obstacle_cntA >0)
		avgA /= obstacle_cntA;
	if(obstacle_cntB >0)
		avgB /=obstacle_cntB;
	if(obstacle_cntC >0)
		avgC /= obstacle_cntC;
}


	void
saveCurrentPosition(Vec3d &xyz, vector<Vec3d> &trajectory, double dMinDist)
{
	int nSize = (int) trajectory.size();

	if(nSize <= 0) {
		trajectory.push_back(xyz);
	} else {
		Vec3d diff = trajectory[nSize-1] - xyz;
		double len = sqrt(diff.dot(diff));

		if(len > dMinDist) {
			trajectory.push_back(xyz);
		}
	}
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


// A callback function. Executed eack time a new pose message arrives.
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


// A callback function. Executed eack time a new pose message arrives.
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

/*	void
printOdometryInfo(nav_msgs::Odometry &odom)
{
	// Display /odom part!
	const ros::Time timestamp = odom.header.stamp;
	const string frame_id = odom.header.frame_id;
	const string child_frame_id = odom.child_frame_id;
	const geometry_msgs::Point translation = odom.pose.pose.position;
	const geometry_msgs::Quaternion rotation = odom.pose.pose.orientation;

	//printf("frame_id = %s, child_frame_id = %s\n", frame_id.c_str(), child_frame_id.c_str());
	//printf("secs: %d / nsecs: %d\n", timestamp.sec, timestamp.nsec);
	//printf("translation = %lf %lf %lf\n", translation.x, translation.y, translation.z);
	//printf("rotation = %lf %lf %lf %lf\n\n\n", rotation.x, rotation.y, rotation.z, rotation.w);
}
*/
//avg -> avg A, B, C
void getInfoFromOdomScan(nav_msgs::Odometry &odom, sensor_msgs::LaserScan &scan, Vec3d &xyz, Vec3d &rpy, vector<Vec3d> &trajectory, vector<Vec3d> &laserScanXY, double &avgA, double &avgB, double &avgC) {

	/* 2. Get current odom-info of turtlebot */
	// 2-1. Get current global odom-info of turtlebot.
	mutex[0].lock(); {
		odom = g_odom;
	} mutex[0].unlock();
	//printf("로봇 현위치 = (%.3lf, %.3lf)\n", odom.pose.pose.position.x, odom.pose.pose.position.y);	

	// 2-2. Get translation & rotation info from odom.
	convertOdom2XYZRPY(odom, xyz, rpy);

	// 2-3. Save current position of turtlebot.
	saveCurrentPosition(xyz, trajectory, 0.02);


	/* 3. Get current scan-info of turtlebot */
	// 3-1. Get current global scan-info of turtlebot.
	mutex[1].lock(); {
		scan = g_scan;
	} mutex[1].unlock();

	// 3-2. Get Cartesian X-Y scan from originl scan between 360 degree.
	convertScan2XYZs(scan, laserScanXY, avgA, avgB, avgC);
}


void shakeBody(ros::Publisher &pubTeleop, double dRotation, double dRotationSpeed){

	geometry_msgs::Twist baseCmd;
	tf::Transform currentTransformation;

	for(int i=0; i<5; i++){
		//get current location
		currentTransformation = getCurrentTransformation();
		//tf::Quaternion t = currentTransformation.getRotation();
		doRotation(pubTeleop, currentTransformation, dRotation, dRotationSpeed);

		currentTransformation = getCurrentTransformation();
		doRotation(pubTeleop, currentTransformation, -1.7 * dRotation, dRotationSpeed);

		currentTransformation = getCurrentTransformation();
		doRotation(pubTeleop, currentTransformation, dRotation, dRotationSpeed);
	}


	baseCmd.linear.x = 0;
	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;
	pubTeleop.publish(baseCmd);

}

//working on it!
void automove(double avgA, double avgB, double avgC,  ros::Publisher &pubTeleop) 
{
	bool obstacle_flag = false;
	geometry_msgs::Twist baseCmd;

	//go forward first
	baseCmd.linear.x = 0.05;
	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;
	pubTeleop.publish(baseCmd);

	/* 4. Calculate average distance from obstacles */
	//printf("장애물 (평균거리) = (%.3lf) (%.3lf) (%.3lf)\n", avgA, avgB, avgC);
	double avg_temp = MIN(avgA, avgB);
	double avg_min = MIN(avg_temp, avgC);
	double avg_max = MAX(avgA, avgB);
	avg_max = MAX(avg_max, avgC);
	if(avg_min > 0 && avg_min < OBSTACLEDISTANCE){
		baseCmd.linear.x = 0;
		baseCmd.linear.y = 0;
		baseCmd.angular.z = 0;
		obstacle_flag = true;			
	}
	pubTeleop.publish(baseCmd);

	double deg = 0;
	if(obstacle_flag){
		printf("[robot_manager/automove] OBSTACLE DETECTED!\n");
	/*	srand(time(NULL));
		if((rand()%2) == 1)
			deg = ROTDEGREE;
		else
			deg = -ROTDEGREE;
	*/
		//Turning 120
		if(avg_max == avgA){
			deg = toRadian(150);
			printf("flag A A A A########\n");
		}

		//range C is left...
		//it means your right side close, so you should turn left. 
		else if(avg_max == avgC){
			deg = ROTDEGREE;
			printf("flag C C C C TURN LEFT #######\n");
		}
		//it means your left side is close, so you should turn right
		else{
			deg = -ROTDEGREE;
			printf("flag BBBBBB TURN RIGHT ######\n");
		}
		
		tf::Transform currentTransformation = getCurrentTransformation();
		doRotation(pubTeleop, currentTransformation, deg, 1.3);		
		obstacle_flag = false;
		printf("TURNING OVER......... sugo yum....\n");
		//go forward
		baseCmd.linear.x = 0.05;
		baseCmd.linear.y = 0;
		baseCmd.angular.z = 0;
		pubTeleop.publish(baseCmd);
	}
}


void drawImage(Mat &display, nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy, vector<Vec3d> &trajectory, vector<Vec3d> &laserScanXY, vector< vector<Vec3d> > &laserScanXYMulti)
{
	// Mat distance for grid
	const double dGridMaxDist = 4.0;

	// 5-1. laserScan을 월드좌표계로 변환
	transform(laserScanXY, xyz[0], xyz[1], rpy[2]);

	// 5-2. 현재 상황을 draw할 display 이미지를 생성
	initGrid(display, 801);
	drawTrajectory(display, trajectory, dGridMaxDist);
	drawCurrentPositionWithRotation(display, xyz, rpy, dGridMaxDist);
	drawLRFScanMulti(display, laserScanXYMulti, dGridMaxDist);
	drawLRFScan(display, laserScanXY, dGridMaxDist);

	// 5-3. 2D 영상좌표계에서 top-view 방식의 3차원 월드좌표계로 변환
	transpose(display, display);  // X-Y축 교환
	flip(display, display, 0);  // 수평방향 반전
	flip(display, display, 1);  // 수직방향 반전

	// 5-4. 영상 출력
	imshow("KNU ROS Lecture >> turtle_pose_lidar_avg", display);
	//printOdometryInfo(odom);

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_manager");

	ros::NodeHandle nhp;
	ros::Publisher pubTeleop = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	ros::Publisher pubFeather = nhp.advertise<std_msgs::String>("/servo_motion", 10);
	
	ros::NodeHandle nhs;
	ros::Subscriber subOdom = nhs.subscribe("/odom", 100, &odomMsgCallback);
	ros::Subscriber subScan = nhs.subscribe("/scan", 10, &scanMsgCallback);
	ros::Subscriber subDetect = nhs.subscribe("/detect", 10, &detectMsgCallback);
	ros::Subscriber subFeed = nhs.subscribe("/servo_motion", 10, &feedMsgCallback);

	nav_msgs::Odometry odom;
	sensor_msgs::LaserScan scan;
	std_msgs::String flagmsg;
	std::stringstream ss;
	ss << "2";
	flagmsg.data = ss.str();

	Mat display;
	initGrid(display, 801);

	// odom 이동 및 회전 정보
	Vec3d xyz, rpy;	
	// 이동궤적
	vector<Vec3d> trajectory;

	// LRF scan 정보
	vector<Vec3d> laserScanXY;
	vector< vector<Vec3d> > laserScanXYMulti;

	// Move first for shakeBody()
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = 0.05;
	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;
	pubTeleop.publish(baseCmd);
	baseCmd.linear.x = 0;
	pubTeleop.publish(baseCmd);


	int cnt = 0;
	while(ros::ok()) {
		/* 1. Call callback methods: odomMsgCallback( ), scanMsgCallback( ), detecMsgCallback() */        
		ros::spinOnce();


		/* 2. get info from odom and scan */
		double avgA = 0;
		double avgB = 0;
		double avgC = 0;
		getInfoFromOdomScan(odom, scan, xyz, rpy, trajectory, laserScanXY, avgA, avgB, avgC);

	
		/*3. feed cat*/
		if(feedFlag == true) {		
	
			printf("-------move back!-------\n\n");

			ros::Time start = ros::Time::now();
			ros::Rate loopRate(1000.0);
			while(ros::Time::now() - start < ros::Duration(2.0)){
				baseCmd.linear.x = -0.05;
				baseCmd.linear.y = 0;
				baseCmd.angular.z = 0;
				pubTeleop.publish(baseCmd);
				ros::spinOnce();
				loopRate.sleep();
			}
			
			printf("-----x = -0.05 publish-----\n\n");

			//ros::Duration d1(1);
			//d1.sleep();
			baseCmd.linear.x = 0;
			baseCmd.linear.y = 0;
			baseCmd.angular.z = 0;
			pubTeleop.publish(baseCmd);
			printf("-----x = 0 publish-----\n\n");
			
			
			tf::Transform currentTransformation;
			currentTransformation = getCurrentTransformation();
			double degree = toRadian(30);
			doRotation(pubTeleop, currentTransformation, degree, 1.3);
			printf("-----rotate publish-----\n\n");

			feedFlag = false;
		}


		/* 4. when cat detected, shake body or feather.*/
		if(catDetectFlag == false){
			//if (feedFlag == false){
				automove(avgA, avgB, avgC, pubTeleop);
				printf("##### feedFlag == false #######\n\n");
			//}
			/*else{
				baseCmd.linear.x = 0;
				baseCmd.linear.y = 0;
				baseCmd.angular.z = 0;
				pubTeleop.publish(baseCmd);
				printf("##### feedFlag == true #######\n\n");
			}*/
		}
		else if(catDetectFlag == true) {

			baseCmd.linear.x = 0;
			baseCmd.linear.y = 0;
			baseCmd.angular.z = 0;
			pubTeleop.publish(baseCmd);

			srand(time(NULL));
			//1) wiggle "flag 1" : call function
			if((rand()%2) == 1) {
				printf("\n\n[robot_manager/main] WIGGLE~WIGGLE~\n");
				double deg = toRadian(10);				
				shakeBody(pubTeleop, deg, 1.5);
			}
			//2) feather "flag 2" : pub message
			else {
				printf("\n\n[robot_manager/main] PLAYING WITH FEATHER~\n");
				pubFeather.publish(flagmsg);
				ros::Duration d(3);
			}
			catDetectFlag = false;
		}


		/* 5. Draw image to OpenCV */
		drawImage(display, odom, xyz, rpy, trajectory, laserScanXY, laserScanXYMulti);


		/* 6. Wait for key to termination */
		int nKey = waitKey(30) % 255;
		if(nKey == 27) {
			break;
		} 
		else if(nKey == ' ') {
			laserScanXYMulti.push_back(laserScanXY);
		}
		else if(nKey == 'c' || nKey == 'C') {
			initGrid(display, 801);
			laserScanXYMulti.clear();
			trajectory.clear();
		}

	} // end-of-while


	return 0;
}

