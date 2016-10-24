#include "findslope.h"
#include "demarcate.h"
void main()
{
	/**********第一步拍摄棋盘图*****************************/
	//Getphoto mydemarcate;
	//mydemarcate.takephoto();
	
	/**********第二步相机标定********************************/
	demarcate mycamera;
	//mycamera.demarcateProcess();

	/**********第三步斜坡识别*******************************/
	Mat img ;
	findslope Slope;
	VideoCapture camera(0);
	if (!camera.isOpened()) 
	{
	     cerr << "ERROR: Could not access the camera or video!" << endl;
		 exit(1);
		
	}
	//初始化图像
	//img = imread("E:/项目/标定/testimage/IMG_2911.jpg");
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
