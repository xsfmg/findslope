#include "findslope.h"
#include "demarcate.h"
void main()
{
	/**********��һ����������ͼ*****************************/
	//Getphoto mydemarcate;
	//mydemarcate.takephoto();
	
	/**********�ڶ�������궨********************************/
	demarcate mycamera;
	//mycamera.demarcateProcess();

	/**********������б��ʶ��*******************************/
	Mat img ;
	findslope Slope;
	VideoCapture camera(0);
	if (!camera.isOpened()) 
	{
	     cerr << "ERROR: Could not access the camera or video!" << endl;
		 exit(1);
		
	}
	//��ʼ��ͼ��
	//img = imread("E:/��Ŀ/�궨/testimage/IMG_2911.jpg");
	while (1)
	{
		camera >> img;
		Slope.findSlopeProcess(img);
		imshow("img", img);
		waitKey(200);
		system("cls");
	}
	Slope.~findslope();
}
