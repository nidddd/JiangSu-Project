#pragma once

#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>

using namespace cv;

class Cam
{
public:
	Cam();
	~Cam();

	//	打开相机、关闭相机
	bool OpenDevice();
	bool CloseDevice();

	//	采集单帧图像
	bool GetImage(Mat &img);


	//	图像处理相关
	bool PicPreProcess(const Mat &img, vector<Mat> &res);

	//	输入二值图，获取目标  在图像上的中心点
	bool GetCenters(const Mat &bin_img, vector<Point> &centers);

	//	输入图像中心点，转换到机器人坐标系
	bool TransCordinate(const vector<Point> centers, vector<Point> &dst);

	bool GetCodes(const Mat &img, vector<string> &barcodes, map<string, Point> &pos);

	//	获取二值图像，主要参数：预处理卷积核大小，二值化阈值
	bool GetBinImg(const Mat &img, Mat &bin_img);
	bool ReadBar(const Mat &img_roi, string &code);

	// 边缘直线拟合，主要参数 HoughLinesP 的几个参数需要调节
	bool LineCrop(const Mat &img, const vector<vector<Point>> &contours);


	bool ShowPic(string winName, const Mat &img, const int &time = 0);

public:
	//	指向打开设备的指针
	CGXDevicePointer objDevicePtr;

	//	设备对应的属性控制器的指针
	CGXFeatureControlPointer objFeatureControlPtr;

	//	设备对应的 流 的指针
	CGXStreamPointer objStreamPtr;

	//	设备打开标志位
	bool isOpen = false;

	//	单帧图像指针
	CImageDataPointer objImageDataPtr;

private:
	bool PIC_DISP;
};

