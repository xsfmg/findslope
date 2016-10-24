#include "demarcate.h"

/*����궨����ģ���ʵ����������*/
void demarcate:: calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
	//  Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));  
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			//  imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);  
			imgpoint.push_back(Point3f(colIndex * squaresize, rowIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

/*��������ĳ�ʼ���� Ҳ���Բ�����*/
void demarcate:: CalibrationEvaluate(void)//�궨�������������
{
	double err = 0;
	double total_err = 0;
	//calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distortion_coeff, rvecs, tvecs, 0);
	cout << "ÿ��ͼ��Ķ�����" << endl;
	for (int i = 0; i < corners.size(); i++)
	{
		vector<Point2f> image_points2;
		vector<Point3f> tempPointSet = objRealPoint[i];
		projectPoints(tempPointSet, rvecs[i], tvecs[i], intrinsic, distortion_coeff, image_points2);
		vector<Point2f> tempImagePoint = corners[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err = err + total_err;
		cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
	cout << "����ƽ����" << total_err / (corners.size() + 1) << "����" << endl;
}

void demarcate:: guessCameraParam(void)
{
	/*�����ڴ�*/
	intrinsic.create(3, 3, CV_64FC1);
	distortion_coeff.create(5, 1, CV_64FC1);
	/*
	fx 0 cx
	0 fy cy
	0 0  1
	*/
	intrinsic.at<double>(0, 0) = 256.8093262;   //fx         
	intrinsic.at<double>(0, 2) = 160.2826538;   //cx  
	intrinsic.at<double>(1, 1) = 254.7511139;   //fy  
	intrinsic.at<double>(1, 2) = 127.6264572;   //cy  

	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;

	/*
	k1 k2 p1 p2 p3
	*/
	distortion_coeff.at<double>(0, 0) = -0.193740;  //k1  
	distortion_coeff.at<double>(1, 0) = -0.378588;  //k2  
	distortion_coeff.at<double>(2, 0) = 0.028980;   //p1  
	distortion_coeff.at<double>(3, 0) = 0.008136;   //p2  
	distortion_coeff.at<double>(4, 0) = 0;          //p3  
}

void demarcate::outputCameraParam(void)
{
	/*��������*/
	WriteData("cameraMatrix.txt", intrinsic);
	WriteData("cameraDistoration.txt", distortion_coeff);
	WriteData("rotatoVector.txt", rvecs[SLOPE_idx]);
	WriteData("translationVector.txt", tvecs[SLOPE_idx]);
	//cvSave("E:/biaodingresult/rotatoVector.xml", &rvecs);
	//cvSave("E:/biaodingresult/translationVector.xml", &tvecs);
	/*�������*/
	cameraX = intrinsic.at<double>(0, 2);
	cameraY = intrinsic.at<double>(1, 2);
	cout << "fx :" << intrinsic.at<double>(0, 0) << endl << "fy :" << intrinsic.at<double>(1, 1) << endl;
	cout << "cx :" << intrinsic.at<double>(0, 2) << endl << "cy :" << intrinsic.at<double>(1, 2) << endl;
	cout << "k1 :" << distortion_coeff.at<double>(0, 0) << endl;
	cout << "k2 :" << distortion_coeff.at<double>(1, 0) << endl;
	cout << "p1 :" << distortion_coeff.at<double>(2, 0) << endl;
	cout << "p2 :" << distortion_coeff.at<double>(3, 0) << endl;
	cout << "p3 :" << distortion_coeff.at<double>(4, 0) << endl;
}

int WriteData(string fileName, cv::Mat& matData)
{
	int retVal = 0;

	// �������Ƿ�Ϊ��  
	if (matData.empty())
	{
		cout << "����Ϊ��" << endl;
		retVal = 1;
		return (retVal);
	}

	// ���ļ�  
	ofstream outFile(fileName.c_str(), ios_base::out);  //���½��򸲸Ƿ�ʽд��  
	if (!outFile.is_open())
	{
		cout << "���ļ�ʧ��" << endl;
		retVal = -1;
		return (retVal);
	}

	// д������  
	for (int r = 0; r < matData.rows; r++)
	{
		for (int c = 0; c < matData.cols; c++)
		{
			int data = matData.at<double>(r, c);    //��ȡ���ݣ�at<type> - type �Ǿ���Ԫ�صľ������ݸ�ʽ  
			outFile << data << "\t";   //ÿ�������� tab ����  
		}
		outFile << endl;  //����  
	}

	return (retVal);
}

int getData(string fileName1, cv::Mat& mymatData, int mymatRows, int mymatCols , int mymatChns)
{
	int retVal = 0;

	// ���ļ�  
	ifstream inFile(fileName1.c_str(), ios_base::in);
	if (!inFile.is_open())
	{
		cout << "��ȡ�ļ�ʧ��" << endl;
		retVal = -1;
		return (retVal);
	}

	// ��������  
	istream_iterator<float> begin(inFile);    //�� float ��ʽȡ�ļ�����������ʼָ��  
	istream_iterator<float> end;          //ȡ�ļ�������ֹλ��  
	vector<float> inData(begin, end);      //���ļ����ݱ����� std::vector ��  
	cv::Mat tmpMat = cv::Mat(inData);       //�������� std::vector ת��Ϊ cv::Mat  

	// ����������д���  
	//copy(vec.begin(),vec.end(),ostream_iterator<double>(cout,"\t"));   

	// ����趨�ľ���ߴ��ͨ����  
	size_t dataLength = inData.size();
	//1.ͨ����  
	if (mymatChns == 0)
	{
		mymatChns = 1;
	}
	//2.������  
	if (mymatRows != 0 && mymatCols == 0)
	{
		mymatCols = dataLength / mymatChns / mymatRows;
	}
	else if (mymatCols != 0 && mymatRows == 0)
	{
		mymatRows = dataLength / mymatChns / mymatCols;
	}
	else if (mymatCols == 0 && mymatRows == 0)
	{
		mymatRows = dataLength / mymatChns;
		mymatCols = 1;
	}
	//3.�����ܳ���  
	if (dataLength != (mymatRows * mymatCols * mymatChns))
	{
		retVal = 1;
		mymatChns = 1;
		mymatRows = dataLength;
	}

	// ���ļ����ݱ������������  
	mymatData = tmpMat.reshape(mymatChns, mymatRows).clone();

	return (retVal);
}

bool demarcate::distance(int x, int y, double &wordX, double &wordY)
{
	Mat Intrinsic;
	Mat rvec;
	Mat tvec;
	getData("cameraMatrix.txt", Intrinsic, 3, 3, 1);
	getData("rotatoVector.txt", rvec, 3, 1, 1);
	getData("translationVector.txt", tvec, 3, 1, 1);
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	Rodrigues(rvec, rotation_matrix);
	cv::Mat H(3, 3, CV_32FC1, Scalar::all(0));
	cv::Mat translation_ve ;
	tvec.copyTo(translation_ve);
	rotation_matrix.copyTo(H);
	cout << "Intrinsic:" << Intrinsic << endl;
	H.at<float>(0, 2) = translation_ve.at<float>(0, 0);
	H.at<float>(1, 2) = translation_ve.at<float>(1, 0);
	H.at<float>(2, 2) = translation_ve.at<float>(2, 0); //[r|t]����
	cout << "H:" << H << endl;
	Mat hu = Mat(3, 3, CV_32FC1, Scalar::all(0));
	Point2f kk;
	hu = Intrinsic*H;
	Mat hu2 = hu.inv();//��ξ���*�ڲξ���������
	float a1, a2, a3, a4, a5, a6, a7, a8, a9;
	a1 = hu2.at<float>(0, 0);
	a2 = hu2.at<float>(0, 1);
	a3 = hu2.at<float>(0, 2);
	a4 = hu2.at<float>(1, 0);
	a5 = hu2.at<float>(1, 1);
	a6 = hu2.at<float>(1, 2);
	a7 = hu2.at<float>(2, 0);
	a8 = hu2.at<float>(2, 1);
	a9 = hu2.at<float>(2, 2);
	kk.x = (a1*x + a2*y + a3) / (a7*x + a8*y + a9);//����������xֵ
	kk.y = (a4*x + a5*y + a6) / (a7*x + a8*y + a9);//����������Yֵ
	wordX = kk.x;
	wordY = kk.y;
	cout << "kk.x:" << kk.x << endl;
	cout << "kk.y:" << kk.y << endl;
	return 0;
}

bool demarcate::demarcateProcess()
{
	int goodFrameCount = 0;
	namedWindow("chessboard");
	cout << "��Q�˳� ..." << endl;
	while (goodFrameCount < frameNumber)
	{

		char filename[100];
		sprintf_s(filename, "E:/��Ŀ/�궨/slope/slope/�궨ͼƬ2/Picture%d.jpg", goodFrameCount);
		//sprintf_s(filename, "%d.jpg", goodFrameCount);
		goodFrameCount++;

		rgbImage = imread(filename, 1);
		cvtColor(rgbImage, grayImage, CV_BGR2GRAY);
		imshow("Camera", grayImage);

		bool isFind = findChessboardCorners(rgbImage, boardSize, corner, 0);
		if (isFind == true) //���нǵ㶼���ҵ� ˵�����ͼ���ǿ��е�  
		{
			/*
			Size(5,5) �������ڵ�һ���С
			Size(-1,-1) ������һ��ߴ�
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)������ֹ����
			*/
			cornerSubPix(grayImage, corner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(rgbImage, boardSize, corner, isFind);
			imshow("chessboard", rgbImage);
			corners.push_back(corner);
			//string filename = "res\\image\\calibration";  
			//filename += goodFrameCount + ".jpg";  
			//cvSaveImage(filename.c_str(), &IplImage(rgbImage));       //�Ѻϸ��ͼƬ��������  

			cout << "The image is good" << endl;
		}
		else
		{
			cout << "The image is bad please try again" << endl;
		}
		//  cout << "Press any key to continue..." << endl;  
		//  waitKey(0);  

		if (waitKey(10) == 'q')
		{
			break;
		}
		//  imshow("chessboard", rgbImage);  
	}

	/*
	ͼ��ɼ���� ��������ʼ����ͷ��У��
	calibrateCamera()
	������� objectPoints  �ǵ��ʵ����������
	imagePoints   �ǵ��ͼ������
	imageSize     ͼ��Ĵ�С
	�������
	cameraMatrix  ������ڲξ���
	distCoeffs    ����Ļ������
	rvecs         ��תʸ��(�����)
	tvecs         ƽ��ʸ��(�������
	*/

	/*����ʵ�ʳ�ʼ���� ����calibrateCamera�� ���flag = 0 Ҳ���Բ���������*/
	guessCameraParam();
	cout << "guess successful" << endl;
	/*����ʵ�ʵ�У�������ά����*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	cout << "cal real successful" << endl;
	/*�궨����ͷ*/
	calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distortion_coeff, rvecs, tvecs, 0);
	cout << "calibration successful" << endl;
	/*���沢�������*/
	outputCameraParam();
	CalibrationEvaluate();
	cout << "out successful" << endl;
	waitKey(0);
	return 0;
}

bool Getphoto:: takephoto()
{
	Mat img;
	VideoCapture camera(0);
	char fileName[100];
	char key;
	int count = 0;
	IplImage* pImg = NULL;
	while (1)
	{
		camera >> img;
		key = cvWaitKey(50);
		if (key == 27) break; //��ESC���˳�����
		if (key == 'c')       //��c������
		{
			sprintf_s(fileName, "E:/Picture %d.jpg", ++count); //�����ļ���
			pImg = cvCreateImage(Size(img.cols, img.rows), 8, 1);
			imwrite(fileName, img);
			*pImg = IplImage(img);
			cvXorS(pImg, cvScalarAll(255), pImg);         //���ĵ���ͼ��ɫ����һ���γ�����Ч����
			cvShowImage("Camera", pImg);
			cvWaitKey(200); //��ɫͼ����ʾms
		}
		imshow("Camera", img);
	}
	return 0;
}