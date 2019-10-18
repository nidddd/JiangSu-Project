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

	//	��������ر����
	bool OpenDevice();
	bool CloseDevice();

	//	�豸�򿪱�־λ
	bool isOpen = false;

	//	�ɼ���֡ͼ��
	bool GetImage(Mat &img);

	//	��ȡͼ���е����룬�Լ���Ӧ��һ������
	bool GetCodes(const Mat &img, vector<string> &barcodes, map<string, Point> &pos);


private:
	//	ͼ��Ԥ��������img�������ֵͼ���ݶ�ͼ���ݶȷ���ͼ
	bool PicPreProcess(const Mat &img, vector<Mat> &res);

	//	�����ֵͼ����ȡĿ��  ��ͼ���ϵ����ĵ�
	//bool GetCenters(const Mat &bin_img, vector<Point> &centers);

	//	����ͼ�����ĵ㣬ת��������������ϵ
	bool TransCordinate(const vector<Point> &centers, vector<Point2d> &dst);
	bool TransCordinate(const Point &centers, Point2d &dst);


	//	��ȡ��ֵͼ����Ҫ������Ԥ�������˴�С����ֵ����ֵ
	bool GetBinImg(const Mat &img, Mat &bin_img, const int &thresh = DEFAULT_THRESH);

	void GetContours(const Mat &bin_img, vector<vector<Point>> &contours);
	void ShowContours(const Mat &img, vector<vector<Point>> &contours, const string &str = "contours", const Scalar &color = Scalar(0, 0, 255));
	void ShowContours(const Mat &img, vector<Point> &contour, const string &str = "contour", const Scalar &color = Scalar(0, 0, 255));
	//	����zbarʶ��ͼ���е�����
	bool ReadBar(const Mat &img_roi, string &code);

	//	����Ѿ�ʶ�𵽵�һ�����ROI��ȡ���������
	void GetCenter(const Mat &img_roi, Point &pos, vector<Point> &accu_contour, const int &thresh = DEFAULT_THRESH);
	void GetDir(const Mat &img_roi, const vector<Point> &contour, float &dir);

	// ��Եֱ����ϣ���Ҫ���� HoughLinesP �ļ���������Ҫ����
	bool LineCrop(const Mat &img, const vector<vector<Point>> &contours);

	//	��ʾͼ����
	//	time Ϊ waitKey(time);
	bool ShowPic(string winName, const Mat &img, const int &time = 0, const bool &close = false);

	//	��ӡ��
	void Println(string str);

	template <class T>
	void Println(string str, const T t);


	/*		���Ӳ����ز���		*/		

	//	ָ����豸��ָ��
	CGXDevicePointer objDevicePtr;

	//	�豸��Ӧ�����Կ�������ָ��
	CGXFeatureControlPointer objFeatureControlPtr;

	//	�豸��Ӧ�� �� ��ָ��
	CGXStreamPointer objStreamPtr;

	//	��֡ͼ��ָ��
	CImageDataPointer objImageDataPtr;

private:
	bool PIC_DISP_ON;	// falseʱ�ر�ShowPic()������ʾͼ��
	bool PRINT_ON;
	//	ͼ����
	const static int CAM_WIDTH = 5496;
	const static int CAM_HEIGHT = 3672;

	//	�ڲξ��� K��������������õ���һ������
	/*
	3551.024	0			0
	0			3550.081	0
	2796.942	1807.466	1
	*/

	// ����������Ե�����fx��fy��Ҫ�ֳ��ٴα궨
	//const double fx = 3524.2;	// ���������Ľ��೤��
	//const double fy = 3523.8;
	//const double u0 = 2801.7;	// ��������������λ��
	//const double v0 = 1803.3;


	//	20191007 �궨���
	const double fx = 3616.2;	// ���������Ľ��೤��
	const double fy = 3616.4;
	const double u0 = 2795.6;	// ��������������λ��
	const double v0 = 1809.3;

	//	����ϵ����-0.104350685299087	0.0848315771350100


	//	Ĭ����ֵ�ָ���ֵ
	const static int DEFAULT_THRESH = 60;

	//	������С���������
	const static int MIN_POINT = 10;

	// ��������λ�ù̶�������ԤѡROI
	const static int rawx = 0;
	const static int rawy = 0;
	const static int raw_width = CAM_WIDTH - rawx - 00;
	const static int raw_height = CAM_HEIGHT - rawy - 000;
};

