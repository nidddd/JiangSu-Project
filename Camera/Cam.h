#pragma once

#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>

using namespace cv;

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
	bool GetBinImg(const Mat &img, Mat &bin_img);

	//	����zbarʶ��ͼ���е�����
	bool ReadBar(const Mat &img_roi, string &code);

	//	����Ѿ�ʶ�𵽵�һ�����ROI��ȡ���������
	void GetCenter(const Mat &img_roi, Point &pos);

	// ��Եֱ����ϣ���Ҫ���� HoughLinesP �ļ���������Ҫ����
	bool LineCrop(const Mat &img, const vector<vector<Point>> &contours);

	//	��ʾͼ����
	//	time Ϊ waitKey(time);
	bool ShowPic(string winName, const Mat &img, const int &time = 0);

	//	��ӡ��
	void Println(string str);



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
	const int CAM_WIDTH = 5496;
	const int CAM_HEIGHT = 3672;

	//	�ڲξ��� K��������������õ���һ������
	/*
	3524.2	0		2801.7
	0		3523.8	1803.3
	0		0		1
	*/

	// ����������Ե�����fx��fy��Ҫ�ֳ��ٴα궨
	const double fx = 3524.2;	// ���������Ľ��೤��
	const double fy = 3523.8;
	const double u0 = 2801.7;	// ��������������λ��
	const double v0 = 1803.3;
};

