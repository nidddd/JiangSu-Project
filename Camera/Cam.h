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

	//	�ɼ���֡ͼ��
	bool GetImage(Mat &img);


	//	ͼ�������
	bool PicPreProcess(const Mat &img, vector<Mat> &res);

	//	�����ֵͼ����ȡĿ��  ��ͼ���ϵ����ĵ�
	bool GetCenters(const Mat &bin_img, vector<Point> &centers);

	//	����ͼ�����ĵ㣬ת��������������ϵ
	bool TransCordinate(const vector<Point> centers, vector<Point> &dst);

	bool GetCodes(const Mat &img, vector<string> &barcodes, map<string, Point> &pos);

	//	��ȡ��ֵͼ����Ҫ������Ԥ�������˴�С����ֵ����ֵ
	bool GetBinImg(const Mat &img, Mat &bin_img);
	bool ReadBar(const Mat &img_roi, string &code);

	// ��Եֱ����ϣ���Ҫ���� HoughLinesP �ļ���������Ҫ����
	bool LineCrop(const Mat &img, const vector<vector<Point>> &contours);


	bool ShowPic(string winName, const Mat &img, const int &time = 0);

public:
	//	ָ����豸��ָ��
	CGXDevicePointer objDevicePtr;

	//	�豸��Ӧ�����Կ�������ָ��
	CGXFeatureControlPointer objFeatureControlPtr;

	//	�豸��Ӧ�� �� ��ָ��
	CGXStreamPointer objStreamPtr;

	//	�豸�򿪱�־λ
	bool isOpen = false;

	//	��֡ͼ��ָ��
	CImageDataPointer objImageDataPtr;

private:
	bool PIC_DISP;
};

