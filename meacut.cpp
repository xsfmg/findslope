#include "findslope.h"
void main()
{
	Mat img ;
	//��ʼ��ͼ��
	img = imread("E:/��Ŀ/�궨/testimage/IMG_2911.jpg");
	findslope Slope;
	Slope.findSlopeProcess(img);
	Slope.~findslope();
	
}
