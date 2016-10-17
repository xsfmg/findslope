#pragma once
#include<stdlib.h>
#include<stdio.h>
#include<math.h>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;
class findslope
{
public:
	findslope();
	~findslope();
	bool findSlopeProcess(Mat image);
private:
	IplImage *img ;
	IplImage *imghsv ;
	IplImage *seg_img ;
	IplImage *erode_img ;
	IplImage *dilate_img ;
	double distance(int h, int s, int v);
	bool Segment(IplImage *src, IplImage *dest);
	bool center_of_gravity(IplImage *src, CvPoint *p);
	bool cal_model(IplImage *src, float *model_H, float *model_S, float *model_V);
	bool findslopbox(IplImage *binaryImage);
};