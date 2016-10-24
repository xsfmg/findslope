#pragma once
#include<stdlib.h>
#include<stdio.h>
#include<math.h>
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;
class findslope
{
public:
	 findslope();
	~findslope();
	bool findSlopeProcess(Mat image);
	const int slope_ratio = 4;
	const int SlopeCenterX = 300;
	const double SLOPEAREA = 4000;
	struct carmove
	{
		int left;
		int right;
		int forward;
		int backward;
		bool stop;
		bool stoprotate;
		double Angle;
	}move;
private:
	IplImage *img ;
	IplImage *imghsv ;
	IplImage *seg_img ;
	IplImage *erode_img ;
	IplImage *dilate_img ;
	Mat hough_result;
	Mat slopeCanny;
	CvPoint P;//重心坐标
	double angle;
	double slopearea;//斜坡面积
	double distance(int h, int s, int v);
	bool Segment(IplImage *src, IplImage *dest);
	bool center_of_gravity(IplImage *src, CvPoint *p);
	bool cal_model(IplImage *src, float *model_H, float *model_S, float *model_V);
	bool findslopbox(IplImage *binaryImage);
	bool hough_transform(Mat& im, Mat& orig, double* skew);
	bool imageprocess(IplImage* src,IplImage* &outImage);
	bool image2hsv(Mat image,IplImage* &outImage);
};