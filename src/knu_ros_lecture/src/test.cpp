#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

int main () {
	Mat img = imread("indoor.jpg", IMREAD_COLOR);
	imshow("output",img);
	waitKey(0);
	return 0;
}


/*
#include <opencv2/core/mat.hpp>  
#include <opencv2/imgcodecs.hpp>  
#include <opencv2/imgproc.hpp>  
#include <opencv2/highgui.hpp>  
#include <opencv2/opencv.hpp> 
 
using namespace cv;  
using namespace std;  
  
  
int main(int argc, char**argv )  
{  
    Mat img_original = imread("indoor.jpg", IMREAD_COLOR );  
    Mat img_edge, img_gray;  
  
  
    //그레이 스케일 영상으로 변환 한후.  
    cvtColor( img_original, img_gray, COLOR_BGR2GRAY);  
    //캐니에지를 이용하여 에지 성분을 검출합니다.  
    Canny( img_gray, img_edge, 30, 90, 3);  
  
  
    //직선 성분을 검출합니다.  
    vector<Vec4i> lines;  
    HoughLinesP( img_edge, lines, 1, CV_PI/180,  30, 30, 3);  
  
    //검출한 직선을 영상에 그려줍니다.  
    for( int i=0; i<lines.size(); i++ )  
    {  
        Vec4i L = lines[i];  
        line(img_original, Point(L[0],L[1]), Point(L[2],L[3]),  
             Scalar(0,0,255), 1, LINE_AA );  
    }  
  
  
    cvNamedWindow("edge", WINDOW_AUTOSIZE );  
    imshow( "edge", img_edge );  
  
    cvNamedWindow("result", WINDOW_AUTOSIZE );  
    imshow( "result", img_original );  
  
    waitKey(0);  
}
*/
