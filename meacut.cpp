#include "findslope.h"
void main()
{
	Mat img ;
	//初始化图像
	img = imread("E:/项目/标定/testimage/IMG_2911.jpg");
	findslope Slope;
	Slope.findSlopeProcess(img);
	Slope.~findslope();
	
}
