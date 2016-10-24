#pragma once
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <iterator>  
#include <vector>  
using namespace std;
using namespace cv;

/**************输入基本参数***********************/
const int imageWidth = 480;                             //摄像头的分辨率  
const int imageHeight = 640;
const int boardWidth = 9;                               //横向的角点数目  
const int boardHeight = 7;                              //纵向的角点数据  
const int boardCorner = boardWidth * boardHeight;       //总的角点数据  
const int frameNumber = 22;                             //相机标定时需要采用的图像帧数  
const int squareSize = 50;                              //标定板黑白格子的大小 单位mm 
const int SLOPE_idx = 10;                               //平行于地面的棋盘图id
const Size boardSize = Size(boardWidth, boardHeight);
/****************************************************/

int WriteData(string fileName, cv::Mat& matData);
int getData(string fileName1, cv::Mat& mymatData, int mymatRows, int mymatCols, int mymatChns);
class demarcate
{
public:
	bool demarcateProcess();
	bool distance(int x, int y, double &wordX,double &wordY);
	int cameraX;
	int cameraY;
private:
	                              
	Mat intrinsic;                                          //相机内参数  
	Mat distortion_coeff;                                   //相机畸变参数  
	vector<Mat> rvecs;                                        //旋转向量  
	vector<Mat> tvecs;                                        //平移向量  
	vector<vector<Point2f>> corners;                        //各个图像找到的角点的集合 和objRealPoint 一一对应  
	vector<vector<Point3f>> objRealPoint;                   //各副图像的角点的实际物理坐标集合  
	vector<Point2f> corner;                                   //某一副图像找到的角点  
	Mat rgbImage, grayImage;
	/*计算标定板上模块的实际物理坐标*/
	void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize);
	/*设置相机的初始参数 也可以不估计*/
	void CalibrationEvaluate(void);//标定结束后进行评价
	void guessCameraParam(void);
	void outputCameraParam(void);
	
	
	
};

class Getphoto
{
public:
	bool takephoto();

};
