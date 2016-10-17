#include "findslope.h"
findslope::findslope() :img(0), imghsv(0), seg_img(0), erode_img(0), dilate_img(0)
{
}
findslope::~findslope()
{
	//cvReleaseImage(&img);
	cvReleaseImage(&imghsv);
	cvReleaseImage(&seg_img);
	cvReleaseImage(&erode_img);
	cvReleaseImage(&dilate_img);
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
	printf("x = %d,y = %d\n", p->x, p->y);
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
bool findslope::findslopbox(IplImage *binaryImage)
{
	Mat srcImage(binaryImage, 0);
	std::vector<std::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i>hierarchy;
	//contour_image.setTo(cv::Scalar(255));
	cv::findContours(srcImage, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	Mat imageContours = Mat::zeros(srcImage.size(), CV_8UC1); //最小外接矩形画布   
	const int minarea = 1000;
	for (int i = 0; i < contours.size(); i++)
	{

		double tmparea = fabs(contourArea(contours[i]));
		//cout << "tmparea:\n" << tmparea << endl;
		if (tmparea <minarea)
		{
			contours.erase(contours.begin()+i);
			hierarchy.erase(hierarchy.begin()+i);
			i = i - 1;
		}
	}
	std::cout << "contours.size():"<<contours.size() << std::endl;
	for (int i = 0; i < contours.size(); i++)
	{
		double tmparea = fabs(contourArea(contours[i]));
		cout << "tmparea:\n" << tmparea << endl;
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
		double a = sqrt((pow(P[0].x - P[1].x, 2)) + pow((P[0].y - P[1].y), 2));
		double b = sqrt((pow(P[0].x - P[3].x, 2)) + pow((P[0].y - P[3].y), 2));
		std::cout << "a:" << a << std::endl;
		std::cout << "b:" << b << std::endl;
		//添加斜坡特征信息
		if (a / b >= slope_ratio)
		{
			std::cout << "find slope!" << std::endl;
		}
		else
		{
			std::cout << "no any slope!" << std::endl;
		}

	}
	cv::namedWindow("contour_image", CV_WINDOW_NORMAL);
	cv::imshow("contour_image", imageContours);
	//*binaryImage=IplImage(srcImage);
	cv::waitKey(0);//等待操作
	return 0;
}

void findslope::hough_transform(cv::Mat& im, cv::Mat& orig, double* skew)
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
	Mat thresh;
	normalize(acc, acc, 255, 0, NORM_MINMAX);
	convertScaleAbs(acc, acc);
	//namedWindow("acc", CV_WINDOW_NORMAL);
	//imshow("acc", acc);
	/*debug
	Mat cmap;
	applyColorMap(acc,cmap,COLORMAP_JET);
	imshow("cmap",cmap);
	imshow("acc",acc);*/

	Point maxLoc;
	minMaxLoc(acc, 0, 0, 0, &maxLoc);
	double theta = (double)maxLoc.y / angleBins*CV_PI;
	double rho = maxLoc.x - max_r;
	if (abs(sin(theta))<0.000001)//check vertical
	{
		//when vertical, line equation becomes
		//x = rho
		double m = -cos(theta) / sin(theta);
		Point2d p1 = Point2d(rho + im.cols / 2, 0);
		Point2d p2 = Point2d(rho + im.cols / 2, im.rows);
		line(orig, p1, p2, Scalar(0, 0, 255), 1);
		*skew = 90;
		std::cout << "skew angle " << " 90" << std::endl;
		std::cout << "p1:" << p1 << std::endl;
		std::cout << "p2:" << p2 << std::endl;
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

		std::cout << "skew angle " << skewangle << std::endl;
		std::cout << "p1:" << p1 << std::endl;
		std::cout << "p2:" << p2 << std::endl;
	}
	imshow("orig", orig);


}

bool findslope::findSlopeProcess(Mat image)
{
	//重心坐标
	CvPoint P = { 0, 0 };
	int image_w = image.rows;
	int image_h = image.cols;
	img = cvCreateImage(cvSize(image_h, image_w), IPL_DEPTH_8U, 3);
	*img = IplImage(image);
	imghsv = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 3);
	seg_img = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	erode_img = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	dilate_img = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	cvCvtColor(img, imghsv, CV_BGR2HSV);
	//分割
	Segment(imghsv, seg_img);
	//cvShowImage("Seg_Image", seg_img);
	//腐蚀
	cvErode(seg_img, erode_img, NULL, 2);
	//cvSaveImage("r1_erode.jpg",erode_img); 
	//cvShowImage("Erode_Image", erode_img);
	//膨胀
	cvDilate(erode_img, dilate_img, NULL, 1);
	//cvSaveImage("r1_dilate.jpg",dilate_img); 
	cvShowImage("Dilate_Image", dilate_img);
	//计算图像的重心
	center_of_gravity(dilate_img, &P);
	//在原始图像上标记重心
	//cvCircle(img, P, 5, cvScalar(0,255,0), 1);
	cvLine(img, cvPoint((int)(P.x - 8), (int)(P.y)),
		cvPoint((int)(P.x + 8), (int)(P.y)), CV_RGB(0, 255, 0), 1, 8, 0);
	cvLine(img, cvPoint((int)(P.x), (int)(P.y - 8)),
		cvPoint((int)(P.x), (int)(P.y + 8)), CV_RGB(0, 255, 0), 1, 8, 0);
	//cvSaveImage("center_r.jpg",img); 
	findslopbox(dilate_img);
	//cvShowImage("Mark_Image", img);
	cvWaitKey(0);
	return 0;
}

