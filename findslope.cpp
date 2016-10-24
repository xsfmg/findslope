#include "findslope.h"
#include "demarcate.h"
findslope::findslope() :img(0), imghsv(0), seg_img(0), erode_img(0), dilate_img(0), P({ 0, 0 }), angle(NULL)
, slopearea(0)
{
	move.Angle = 0;
	move.left = 0;
	move.right = 0;
	move.forward = 0;
	move.backward = 0;
	move.stop = 0;
	move.stoprotate = 0;
}
findslope::~findslope()
{
	//cvReleaseImage(&img);
	cvReleaseImage(&imghsv);
	cvReleaseImage(&seg_img);
	cvReleaseImage(&erode_img);
	cvReleaseImage(&dilate_img);
}
bool findslope::image2hsv(Mat image,IplImage* &outImage)
{
	int image_w = image.rows;
	int image_h = image.cols;
	img = cvCreateImage(cvSize(image_h, image_w), IPL_DEPTH_8U, 3);
	erode_img = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	dilate_img = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	*img = IplImage(image);
	imghsv = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 3);
	seg_img = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	cvCvtColor(img, outImage, CV_BGR2HSV);
	return 0;
}
bool findslope::imageprocess(IplImage* src,IplImage* &outImage)
{
	
	outImage = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	//腐蚀
	cvErode(src, erode_img, NULL, 2);
	//膨胀
	cvDilate(erode_img, outImage, NULL, 1);
	return 0;
}
double findslope::distance(int h, int s, int v)
{
	double const pi = 3.141595653;
	//目标颜色
	int Object_H = 0;
	int Object_S = 0;
	int Object_V = 80;

	double distance = 0;
	double com1 = 0, com2 = 0, com3 = 0;
	//计算距离
	com1 = s*cos(h*pi / 180) - Object_S*cos(Object_H*pi / 180);
	com1 = com1*com1;

	com2 = s*sin(h*pi / 180) - Object_S*sin(Object_H*pi / 180);
	com2 = com2*com2;

	com3 = v - Object_V;
	com3 = com3*com3;

	distance = sqrt(com1 + com2 + com3);
	return distance;
}

//图像分割
bool findslope::Segment(IplImage *src, IplImage *dest)
{
	//阈值设置
	int threshold = 190;
	int height, width, step, channels;
	uchar *data_src, *data_dest;
	int i, j;
	height = src->height;
	width = src->width;
	step = src->widthStep;
	channels = src->nChannels;
	data_src = (uchar*)src->imageData;
	data_dest = (uchar*)dest->imageData;
	//目标为黑色，背景为白色
	for (i = 0; i<height; i++)
		for (j = 0; j<width; j++)
		{
			if (distance(data_src[i*step + j*channels], data_src[i*step + j*channels + 1], data_src[i*step + j*channels + 2])>threshold)
				data_dest[i*(dest->widthStep) + j] = 255;
			else
				data_dest[i*(dest->widthStep) + j] = 0;
		}
	return 0;
}

//计算重心
bool findslope::center_of_gravity(IplImage *src, CvPoint *p)
{
	double m00, m10, m01;
	int height, width;
	CvMoments moment;
	height = src->height;
	width = src->width;
	cvMoments(src, &moment, 1);//计算矩，获得7hu。
	m00 = cvGetSpatialMoment(&moment, 0, 0);
	if (m00 == 0)
		return 1;
	m10 = cvGetSpatialMoment(&moment, 1, 0);
	m01 = cvGetSpatialMoment(&moment, 0, 1);
	p->x = (int)(m10 / m00);
	p->y = (int)(m01 / m00);
	printf("重心坐标：x = %d,y = %d\n", p->x, p->y);
	return 0;
}

//计算颜色模板
bool findslope:: cal_model(IplImage *src, float *model_H, float *model_S, float *model_V)
{
	//阈值设置
	int height, width, step, channels;
	uchar *data_src;
	int i, j;
	*model_H = 0;
	*model_S = 0;
	*model_V = 0;

	height = src->height;
	width = src->width;
	step = src->widthStep;
	channels = src->nChannels;
	data_src = (uchar*)src->imageData;


	//目标为黑色，背景为白色
	for (i = 0; i<height; i++)
		for (j = 0; j<width; j++)
		{
			*model_H += data_src[i*step + j*channels];
			*model_S += data_src[i*step + j*channels + 1];
			*model_V += data_src[i*step + j*channels + 2];
		}
	*model_H = (*model_H) / (height*width);
	*model_S = (*model_S) / (height*width);
	*model_V = (*model_V) / (height*width);
	return 0;
}

//识别斜坡
bool findslope::findslopbox(IplImage *binaryImage)
{
	Mat srcImage(binaryImage, 0);
	std::vector<std::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i>hierarchy;
	double boxwidth;
	double boxheight;
	//contour_image.setTo(cv::Scalar(255));
	cv::findContours(srcImage, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	Mat imageContours = Mat::zeros(srcImage.size(), CV_8UC1); //最小外接矩形画布   
	const int minarea = 1000;
	int maxarea = 0;
	for (int i = 0; i < contours.size(); i++)
	{

		double tmparea = fabs(contourArea(contours[i]));
		if (tmparea > maxarea)
		{
			maxarea = tmparea;
		}
		//cout << "tmparea:\n" << tmparea << endl;
		if (tmparea <minarea)
		{
			contours.erase(contours.begin()+i);
			hierarchy.erase(hierarchy.begin()+i);
			i = i - 1;
		}
	}
	slopearea = maxarea;
	//std::cout << "contours.size():"<<contours.size() << std::endl;
	for (int i = 0; i < contours.size(); i++)
	{
		double tmparea = fabs(contourArea(contours[i]));
		//cout << "tmparea:\n" << tmparea << endl;
		//绘制轮廓  
		//drawContours(imageContours, contours, i, Scalar(255), 1, 8,hierarchy);
		//绘制轮廓的最小外结矩形  
		RotatedRect rect = minAreaRect(contours[i]);
		Point2f P[4];
		rect.points(P);
		for (int j = 0; j <= 3; j++)
		{
			line(imageContours, P[j], P[(j + 1) % 4], Scalar(255), 2);
		}
		boxwidth = sqrt((pow(P[0].x - P[1].x, 2)) + pow((P[0].y - P[1].y), 2));
		boxheight = sqrt((pow(P[0].x - P[3].x, 2)) + pow((P[0].y - P[3].y), 2));
		//std::cout << "a:" << a << std::endl;
		//std::cout << "b:" << b << std::endl;
		
	}
	//添加斜坡特征信息
	if (boxwidth / boxheight >= slope_ratio || boxheight / boxwidth >= slope_ratio)
	{
		//std::cout << "find slope!" << std::endl;
		return 0;
	}
	else
	{
		//std::cout << "no any slope!" << std::endl;
		return 1;
	}
	//cv::namedWindow("contour_image", CV_WINDOW_NORMAL);
	//cv::imshow("contour_image", imageContours);
	//*binaryImage=IplImage(srcImage);
	//cv::waitKey(0);//等待操作
}

//计算旋转角度
bool findslope::hough_transform(Mat& im, Mat& orig, double* skew)
{
	double max_r = sqrt(pow(.5*im.cols, 2) + pow(.5*im.rows, 2));
	int angleBins = 180;
	Mat acc = Mat::zeros(Size(2 * max_r, angleBins), CV_32SC1);
	int cenx = im.cols / 2;
	int ceny = im.rows / 2;
	for (int x = 1; x<im.cols - 1; x++)
	{
		for (int y = 1; y<(im.rows - 1) / 2; y++)
		{
			if (im.at<uchar>(y, x) == 255)
			{
				for (int t = 0; t<angleBins; t++)
				{
					double r = (x - cenx)*cos((double)t / angleBins*CV_PI) + (y - ceny)*sin((double)t
						/ angleBins*CV_PI);
					r += max_r;
					acc.at<int>(t, int(r))++;
				}
			}
		}
	}
	normalize(acc, acc, 255, 0, NORM_MINMAX);
	convertScaleAbs(acc, acc);
	//namedWindow("acc", CV_WINDOW_NORMAL);
	//imshow("acc", acc);

	

	Point maxLoc;
	minMaxLoc(acc, 0, 0, 0, &maxLoc);
	double theta = (double)maxLoc.y / angleBins*CV_PI;
	double rho = maxLoc.x - max_r;
	if (abs(sin(theta))<0.0000001)//check vertical
	{
		//when vertical, line equation becomes
		//x = rho
		double m = -cos(theta) / sin(theta);
		Point2d p1 = Point2d(rho + im.cols / 2, 0);
		Point2d p2 = Point2d(rho + im.cols / 2, im.rows);
		//line(orig, p1, p2, Scalar(0, 0, 255), 1);
		*skew = 90;
		//std::cout << "skew angle " << " 90" << std::endl;
		//std::cout << "p1:" << p1 << std::endl;
		//std::cout << "p2:" << p2 << std::endl;
	}
	else
	{
		//convert normal form back to slope intercept form
		//y = mx + b
		double m = -cos(theta) / sin(theta);
		double b = rho / sin(theta) + im.rows / 2. - m*im.cols / 2.;
		Point2d p1 = Point2d(0, b);
		Point2d p2 = Point2d(im.cols, im.cols*m + b);
		line(orig, p1, p2, Scalar(255, 0, 255), 1);
		double skewangle;
		skewangle = p1.x - p2.x>0 ? (atan2(p1.y - p2.y, p1.x - p2.x)*180. / CV_PI) : (atan2(p2.y - p1.y, p2.x - p1.x)*180. / CV_PI);
		*skew = skewangle;
		//std::cout << "skew angle " << skewangle << std::endl;
		//std::cout << "p1:" << p1 << std::endl;
		//std::cout << "p2:" << p2 << std::endl;
	}
	//namedWindow("lineslope",WINDOW_NORMAL);
	//imshow("lineslope", orig);
	//waitKey(0);
	return 0;
}



//主程序
bool findslope::findSlopeProcess(Mat image)
{
	//图像转hsv
	if (image2hsv(image,imghsv))
	{
     std::cout << "image2hsv failed" << std::endl;
	}
	
	//分割
	if (Segment(imghsv, seg_img))
	{
		std::cout << "Segment failed" << std::endl;
	}

	//图像去燥，腐蚀膨胀
	if (imageprocess(seg_img,dilate_img))
	{
		std::cout << "imageprocess failed" << std::endl;
	}
	
	//识别斜坡
	if (findslopbox(dilate_img))
	{
		std::cout << "no any slope!" << std::endl;
	}
	else
	{
		std::cout << "find slope!" << std::endl;
		if (slopearea < SLOPEAREA)
		{
			std::cout << "go forward..." << std::endl;
			move.forward = 1;
			move.backward = 0;
		}
		else
		{
			Mat slopeimage(dilate_img, 0);
			Canny(slopeimage, slopeCanny, 1, 3, 3);
			//计算旋转角度
			if (hough_transform(slopeCanny, slopeCanny, &angle))
			{
				std::cout << "hough_transform failed" << std::endl;
			}
			else
			{

				if (0 < angle &&89.8>=angle)
				{
					move.Angle = -(90 - angle);
					move.stoprotate = 0; //没有正对斜坡
					std::cout << "逆时针旋转：" << move.Angle << "度" << std::endl;
				}
				if (-89.8 < angle &&0>=angle)
				{
					std::cout << "顺时针旋转：" << abs(angle) << "度" << std::endl;
					move.Angle = abs(angle);
					move.stoprotate = 1; //没有正对斜坡
				}
				//值为正代表逆时针转
				//值为负代表顺时针转
				if (abs(abs(angle) - 90) < 0.2)
				{
					std::cout << "机器人已正对斜坡" << std::endl;
					move.Angle = 0;
					move.stoprotate = 1; //正对斜坡
					move.left = 0;
					move.right = 0;
					move.forward = 0;
					move.backward = 0;
					move.stop = 0;
					//计算斜坡重心
					if (center_of_gravity(dilate_img, &P))
					{
						std::cout << "center_of_gravity failed" << std::endl;
					}
					else
					{
						demarcate mycamera;
						cvCircle(img, P, 5, cvScalar(0, 0, 0), 3);
						int distanceX = P.x -mycamera.cameraX;
						double wordPx=0;
						double wordPy = 0;
						double camera2wordX = 0;
						double camera2wordY = 0;
						mycamera.distance(P.x,P.y,wordPx,wordPy); //计算小车斜坡距离信息
						mycamera.distance(mycamera.cameraX, mycamera.cameraY, camera2wordX, camera2wordY);
						distanceX = wordPx - mycamera.cameraX;
						if (distanceX < -0.1)
						{
							move.Angle = 0;
							move.stoprotate = 1; //正对斜坡
							move.left = distanceX;
							move.right = 0;
							move.forward = mycamera.cameraY;
							move.backward = 0;
							move.stop = 0;
							
							std::cout << "go left..." << std::endl;
						}
						if (-0.1 <= distanceX && 0.1 >= distanceX)
						{
							move.Angle = 0;
							move.stoprotate = 1; //正对斜坡
							move.left = 0;
							move.right = 0;
							move.forward = mycamera.cameraY;
							move.backward = 0;
							move.stop = 0;
							std::cout << "机器人到达斜坡中轴线上" << std::endl;

						}
						if (distanceX > 0.1)
						{
							move.Angle = 0;
							move.stoprotate = 1; //正对斜坡
							move.left = 0;
							move.right = distanceX;
							move.forward = mycamera.cameraY;
							move.backward = 0;
							move.stop = 0;
							std::cout << "go right..." << std::endl;
						}
					}
				}
			}
		}
	}
	//cvShowImage("Seg_Image", seg_img);
	//cvShowImage("Erode_Image", erode_img);
	//cvShowImage("Dilate_Image", dilate_img);
	//在原始图像上标记重心
	image = img;
	//cvNamedWindow("Mark_Image",CV_WINDOW_NORMAL);
	//cvShowImage("Mark_Image", img);
	//cvWaitKey(0);
	
	return 0;
}

