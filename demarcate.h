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

/**************�����������***********************/
const int imageWidth = 480;                             //����ͷ�ķֱ���  
const int imageHeight = 640;
const int boardWidth = 9;                               //����Ľǵ���Ŀ  
const int boardHeight = 7;                              //����Ľǵ�����  
const int boardCorner = boardWidth * boardHeight;       //�ܵĽǵ�����  
const int frameNumber = 22;                             //����궨ʱ��Ҫ���õ�ͼ��֡��  
const int squareSize = 50;                              //�궨��ڰ׸��ӵĴ�С ��λmm 
const int SLOPE_idx = 10;                               //ƽ���ڵ��������ͼid
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
	                              
	Mat intrinsic;                                          //����ڲ���  
	Mat distortion_coeff;                                   //����������  
	vector<Mat> rvecs;                                        //��ת����  
	vector<Mat> tvecs;                                        //ƽ������  
	vector<vector<Point2f>> corners;                        //����ͼ���ҵ��Ľǵ�ļ��� ��objRealPoint һһ��Ӧ  
	vector<vector<Point3f>> objRealPoint;                   //����ͼ��Ľǵ��ʵ���������꼯��  
	vector<Point2f> corner;                                   //ĳһ��ͼ���ҵ��Ľǵ�  
	Mat rgbImage, grayImage;
	/*����궨����ģ���ʵ����������*/
	void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize);
	/*��������ĳ�ʼ���� Ҳ���Բ�����*/
	void CalibrationEvaluate(void);//�궨�������������
	void guessCameraParam(void);
	void outputCameraParam(void);
	
	
	
};

class Getphoto
{
public:
	bool takephoto();

};
