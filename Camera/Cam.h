#pragma once

#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <zbar.h>

using namespace cv;
using namespace std;

class Cam
{
public:
	Cam();
	~Cam();

	//	打开相机、关闭相机
	bool OpenDevice();
	bool CloseDevice();

	//	设备打开标志位
	bool isOpen = false;

	//	采集单帧图像
	bool GetImage(Mat &img);

	//	获取图像中的条码，以及对应归一化坐标
	bool GetCodes(const Mat &img, vector<string> &barcodes, map<string, Point> &pos);


private:
	//	图像预处理，输入img，输出二值图、梯度图、梯度方向图
	bool PicPreProcess(const Mat &img, vector<Mat> &res);

	//	输入二值图，获取目标  在图像上的中心点
	//bool GetCenters(const Mat &bin_img, vector<Point> &centers);

	//	输入图像中心点，转换到机器人坐标系
	bool TransCordinate(const vector<Point> &centers, vector<Point2d> &dst);
	bool TransCordinate(const Point &centers, Point2d &dst);


	//	获取二值图像，主要参数：预处理卷积核大小，二值化阈值
	bool GetBinImg(const Mat &img, Mat &bin_img, const int &thresh = DEFAULT_THRESH);

	void GetContours(const Mat &bin_img, vector<vector<Point>> &contours);
	void ShowContours(const Mat &img, vector<vector<Point>> &contours, const string &str = "contours", const Scalar &color = Scalar(0, 0, 255));
	void ShowContours(const Mat &img, vector<Point> &contour, const string &str = "contour", const Scalar &color = Scalar(0, 0, 255));
	//	采用zbar识别图像中的条码
	bool ReadBar(const Mat &img_roi, string &code);

	//	针对已经识别到单一条码的ROI提取出零件中心
	void GetCenter(const Mat &img_roi, Point &pos, vector<Point> &accu_contour, const int &thresh = DEFAULT_THRESH);
	void GetDir(const Mat &img_roi, const vector<Point> &contour, float &dir);

	// 边缘直线拟合，主要参数 HoughLinesP 的几个参数需要调节
	bool LineCrop(const Mat &img, const vector<vector<Point>> &contours);

	//	显示图像函数
	//	time 为 waitKey(time);
	bool ShowPic(string winName, const Mat &img, const int &time = 0, const bool &close = false);

	//	打印行
	void Println(string str);

	template <class T>
	void Println(string str, const T t);


	/*		相机硬件相关参数		*/		

	//	指向打开设备的指针
	CGXDevicePointer objDevicePtr;

	//	设备对应的属性控制器的指针
	CGXFeatureControlPointer objFeatureControlPtr;

	//	设备对应的 流 的指针
	CGXStreamPointer objStreamPtr;

	//	单帧图像指针
	CImageDataPointer objImageDataPtr;

private:
	bool PIC_DISP_ON;	// false时关闭ShowPic()函数显示图像
	bool PRINT_ON;
	//	图像宽高
	const static int CAM_WIDTH = 5496;
	const static int CAM_HEIGHT = 3672;

	//	内参矩阵 K，左乘像素坐标后得到归一化坐标
	/*
	3551.024	0			0
	0			3550.081	0
	2796.942	1807.466	1
	*/

	// 由于相机可以调焦，fx和fy需要现场再次标定
	//const double fx = 3524.2;	// 像素描述的焦距长度
	//const double fy = 3523.8;
	//const double u0 = 2801.7;	// 像素描述的主点位置
	//const double v0 = 1803.3;


	//	20191007 标定结果
	const double fx = 3616.2;	// 像素描述的焦距长度
	const double fy = 3616.4;
	const double u0 = 2795.6;	// 像素描述的主点位置
	const double v0 = 1809.3;

	//	畸变系数：-0.104350685299087	0.0848315771350100


	//	默认阈值分割数值
	const static int DEFAULT_THRESH = 60;

	//	轮廓最小包含点个数
	const static int MIN_POINT = 10;

	// 由于拍摄位置固定，可以预选ROI
	const static int rawx = 0;
	const static int rawy = 0;
	const static int raw_width = CAM_WIDTH - rawx - 00;
	const static int raw_height = CAM_HEIGHT - rawy - 000;
};

