#include "stdafx.h"
#include "Cam.h"

using namespace zbar;


Cam::Cam()
{
	PIC_DISP = true;
}


Cam::~Cam()
{
}

bool Cam::OpenDevice()
{
	// ��ʼ��
	IGXFactory::GetInstance().Init();


	//	ö���豸
	GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;

	IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);

	cout << "��ȡ�����豸�б�" << endl;
	for (uint32_t i = 0; i < vectorDeviceInfo.size(); i++)

	{

		cout << vectorDeviceInfo[i].GetVendorName() << endl;

		cout << vectorDeviceInfo[i].GetModelName() << endl;

	}


	//	���豸
	//  ��ͨ�����¼��ַ�ʽ��

	/*
	IGXFactory::GetInstance().OpenDeviceBySN

	IGXFactory::GetInstance().OpenDeviceByUserID

	IGXFactory::GetInstance().OpenDeviceByMAC

	IGXFactory::GetInstance().OpenDeviceByIP
	*/


	//�������еĵ�һ���豸
	

	if (vectorDeviceInfo.size()> 0)

	{

		GxIAPICPP::gxstring strSN = vectorDeviceInfo[0].GetSN();

		GxIAPICPP::gxstring strUserID = vectorDeviceInfo[0].GetUserID();

		GxIAPICPP::gxstring strMAC = vectorDeviceInfo[0].GetMAC();

		GxIAPICPP::gxstring strIP = vectorDeviceInfo[0].GetIP();


		objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(strSN, GX_ACCESS_EXCLUSIVE);

		//	�豸�Ѿ���
		isOpen = true;


		//objDevicePtr = IGXFactory::GetInstance().OpenDeviceByUserID(strUserID, GX_ACCESS_EXCLUSIVE);
		//objDevicePtr = IGXFactory::GetInstance().OpenDeviceByMAC(strMAC, GX_ACCESS_EXCLUSIVE);
		//objDevicePtr = IGXFactory::GetInstance().OpenDeviceByIP(strIP, GX_ACCESS_EXCLUSIVE);


		//��ȡԶ���豸���Կ�����
		objFeatureControlPtr = objDevicePtr->GetRemoteFeatureControl();

		//	��ȡ ��
		objStreamPtr = objDevicePtr->OpenStream(0);
	}



	return isOpen == true;
}

bool Cam::CloseDevice()
{


	objStreamPtr->Close();

	objDevicePtr->Close();

	//	����ʼ�����ͷ�GxIAPICPP�����������Դ
	IGXFactory::GetInstance().Uninit();

	//	�豸�Ѿ��ر�
	isOpen = false;
	return isOpen == false;
}

bool Cam::GetImage(Mat &img)
{
	//	�ڿ�����"��ͨ���ɼ�"��"���Ϳ�������"֮��,�Ϳ���ͨ��"��ͨ���ɼ�"��ȡ��֡ͼ����

	//	�ɼ���ͼ���,��ǡ����ʱ�� ,�� "ֹͣ�ɼ�", ��ֹͣ��ͨ���ɼ�

	//	��ͨ���ɼ��ȿ�,���


	//	��Ҫ���ж� isOpen
	if (!isOpen)
	{
		return false;
	}

	//	�� �� �п���ͨ��
	objStreamPtr->StartGrab();

	//	��ʼ�ɼ�
	objFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();



	//�ɵ�֡

	CImageDataPointer objImageDataPtr;


	objImageDataPtr = objStreamPtr->GetImage(500);//��ʱʱ��ʹ��500ms���û����������趨

	if (objImageDataPtr->GetStatus() == GX_FRAME_STATUS_SUCCESS)

	{

		//��ͼ�ɹ�����������֡�����Խ���ͼ����...
		int wid = objImageDataPtr->GetWidth();
		int hei = objImageDataPtr->GetHeight();
		img.create(hei, wid, CV_8UC1);



		BYTE *pBuffer = (BYTE*)objImageDataPtr->ConvertToRaw8(GX_BIT_0_7);
		
		try
		{
			memcpy(img.data, pBuffer, (objImageDataPtr->GetHeight()) * (objImageDataPtr->GetWidth()));

		}
		catch (...)
		{
			cout << "ͼƬ���Ƴ���" << endl;
			return false;
		}


		//Mat img1;
		//cv::pyrDown(img, img1);
		//cv::pyrDown(img1, img1);
		//cv::pyrDown(img1, img1);

		//cv::rotate(img1, img1, cv::ROTATE_90_CLOCKWISE);
		//ShowPic("�ɼ���ͼ��", img1);
		//if (cv::waitKey() == 's');// break;

	}



	//����ͣ������

	objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

	objStreamPtr->StopGrab();

	return true;
}

bool Cam::PicPreProcess(const Mat & img, vector<Mat>& res)
{
	//	����ͼ�񣬾���
	//	�Ҷ�ͼ->��˹�˲�->��ֵ�ָ�->��Ե���
	//	��������Ե���������Ե��ֵͼ����Ե����Ƕ�ͼ



	//	�Ҷ�ͼ
	Mat img_gray;
	try
	{
		cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

	}
	catch (...)
	{
		//cout << "�Ѿ��ǻҶ�ͼ������ת��" << endl;
		img_gray = img;
	}

	//pyrDown(img_gray, img_gray);
	//pyrDown(img_gray, img_gray);

	//rotate(img_gray, img_gray, ROTATE_90_CLOCKWISE);

	//	��ֵ�˲�
	Mat img_mid = img_gray;
	medianBlur(img_gray, img_mid, 5);

	//	��˹ģ��
	Mat img_gauss = img_mid;
	GaussianBlur(img_gray, img_gauss, cv::Size(3, 3), 0, 0);


	//	��ֵ�ָ�
	Mat img_bin;
	threshold(img_gauss, img_bin, 100, 255, cv::THRESH_OTSU | cv::THRESH_BINARY_INV);

	res.push_back(img_bin);


	//	��Ե���
	Mat img_schx, img_schy;

	try
	{
		Scharr(img_bin, img_schx, CV_32F, 1, 0);
		Scharr(img_bin, img_schy, CV_32F, 0, 1);
	}
	catch (...)
	{
		cout << "��Ե������" << endl;
	}


	//	���ݼ��ı�Ե�����������Ե�Ƕ�
	Mat img_gra;
	Mat img_dir;	//	[0��2*pi]

	try
	{
		cartToPolar(img_schx, img_schy, img_gra, img_dir, true);
	}
	catch (...)
	{
		cout << "�ݶȷ���������" << endl;
	}


	Mat img_mag;

	try
	{
		img_gra.convertTo(img_mag, CV_8UC1);
	}
	catch (...)
	{
		cout << "��Եͼ��ת������" << endl;
	}

	//std::vector<Mat> res;
	//res.push_back(img_gra_bin);
	res.push_back(img_mag);

	//	����Ϊ CV_32F�� ��Ӧ float
	res.push_back(img_dir);

	return true;
}

bool Cam::GetCenters(const Mat & bin_img, vector<Point>& centers)
{
	try
	{

		Mat img;
		cvtColor(bin_img, img, cv::COLOR_GRAY2BGR);

		vector<vector<Point>> contours;
		findContours(bin_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		drawContours(img, contours, -1, Scalar(0, 0, 255), 2);

		// ��������
		for (auto it : contours) 
		{
			//�������е�����
			Moments mom = cv::moments(cv::Mat(it));
			//��������
			Point center = Point(mom.m10 / mom.m00, mom.m01 / mom.m00);

			//	������ѹ���������
			centers.push_back(center);

			cout << center << endl;
			circle(img, center, 2, cv::Scalar(0,255,0), 3);
		}
	
		string winName = "contours";
		ShowPic(winName, img);
		moveWindow(winName, 200, 200);

		return true;
	}
	catch (...)
	{
		cout << "��ȡ����ʧ�ܣ�" << endl;
		return false;
	}
}

bool Cam::TransCordinate(const vector<Point> centers, vector<Point>& dst)
{
	dst.clear();
	for (Point p : centers)
	{
		// ������λ��ӳ�䵽��������ϵ
		//	��Ҫ �ڲξ��� �� ��ξ���


		//	��������ϵ������������ϵ����Ҫ����ȷ����������ϵ������
	}

	return false;
}

bool Cam::GetCodes(const Mat & img, vector<string>& barcodes, map<string, Point>& pos)
{
	//	��ͼ���ֵ������ֵ����
	//	ԭ�� ��ɫ �Ĳ��֣������룩������Ϊ ��ɫ
	Mat bin_img;
	GetBinImg(img, bin_img);

	//	Ϊ��ȡ׼ȷλ�ã����ܶ�ԭͼ�������͸�ʴ�Ĳ���
	int k_size = 30;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(k_size, k_size));

	//	Ҫ��ȡ�Ĳ���Ϊ��ɫ������ɫ���֣�����2�Σ� ��ʴ3��
	dilate(bin_img, bin_img, kernel);
	dilate(bin_img, bin_img, kernel);
	erode(bin_img, bin_img, kernel);
	erode(bin_img, bin_img, kernel);
	erode(bin_img, bin_img, kernel);
	ShowPic("����*2��ʴ*3", bin_img);

	

	//	��Ѱ����
	vector<cv::Vec4i> hierar;
	vector<vector<Point>> contours;
	findContours(bin_img, contours, hierar, RETR_TREE, CHAIN_APPROX_SIMPLE);

	cout << "�ҵ�������Ŀ��" << contours.size() << endl;

	//	��������
	Mat img1;
	cvtColor(bin_img, img1, cv::COLOR_GRAY2BGR);


	drawContours(img1, contours, -1, Scalar(0, 0, 255), 2);

	ShowPic("contours", img1);


	//	���� --- ����λ�� ��Ӧ
	map<string, Point> code2pos;

	vector<vector<Point>> code_contours;

	//	���ÿ����������ȡ�������򣬲�ʶ������
	for (auto contour : contours)
	{
		Rect rect = boundingRect(contour);

		Point center = Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
		vector<Point> centers;

		Mat img_roi = img(rect);
		ShowPic("img_roi", img_roi);


		string code;
		if (ReadBar(img_roi, code))
		{
			code2pos[code] = center;
			cout << "����λ�ã�" << center << endl;

			//	���ڵ�����������򣬽�������Ҫ����ֱ����ϱ�Ե
			code_contours.push_back(contour);

			//	���ڼ�����������λ��
			//centers.push_back(center);

			//	�ж� ͬһ�С��� �ı���
			//bias_wid = bias_wid > img_roi.cols ? img_roi.cols : bias_wid;
			//bias_hei = bias_hei > img_roi.rows ? img_roi.rows : bias_hei;
			
		}
	}

	LineCrop(img, code_contours);

	if (barcodes.size() == 0) return false;

	std::sort(barcodes.begin(), barcodes.end());

	//	������������λ��
	//	�����Ұ�� ��� �����Ѿ������� ���̱�Ե
	//	��� bias �������ڣ���Ϊ��ͬһ��/��
	/*int bias = 100;

	vector<int> rows, cols;
	rows.push_back(code2pos[barcodes[0]].x);
	cols.push_back(code2pos[barcodes[0]].y);

	for (int i = 0; i < barcodes.size() - 1; i++)
	{
		for (int j = i + 1; j < barcodes.size(); j++)
		{
			for (int m = 0; m < rows.size(); m++)
			{
				if()
			}
		}
	}
*/


	return true;
}

bool Cam::GetBinImg(const Mat & img, Mat & bin_img)
{
	//	�Ҷ�ͼ
	Mat img_gray;
	try
	{
		cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

	}
	catch (...)
	{
		img_gray = img;
	}

	cout << "ͼ���С��" << img_gray.size() << endl;
	//if (img_gray.size[0] > 3000)
	//{
	//	pyrDown(img_gray, img_gray);
	//	pyrDown(img_gray, img_gray); 
	//}
	//rotate(img_gray, img_gray, ROTATE_90_CLOCKWISE);

	//	��ֵ�˲�
	Mat img_mid = img_gray;
	medianBlur(img_gray, img_mid, 5);

	//	��˹ģ��
	Mat img_gauss = img_mid;
	GaussianBlur(img_gray, img_gauss, cv::Size(3, 3), 0, 0);


	//	�������˲�����
	img_gauss = img_gray;

	ShowPic("�Ҷ�ͼ", img_gray);


	//	��ֵ�ָ�
	Mat img_bin;
	threshold(img_gauss, img_bin, 60, 255, /*cv::THRESH_OTSU |*/ cv::THRESH_BINARY_INV);

	bin_img = img_bin;

	ShowPic("��ֵͼ", img_bin);

	return true;
}

bool Cam::ReadBar(const Mat & img_roi, string & code)
{
	ImageScanner scanner;

	//	ɨ���������
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

	Mat img_gray;
	try
	{
		cvtColor(img_roi, img_gray, COLOR_BGR2GRAY);
	}
	catch (...)
	{
		img_gray = img_roi;
	}

	int width = img_gray.cols;
	int height = img_gray.rows;
	uchar *raw = (uchar *)img_gray.data;

	//	��ʼ��һ�� zbarͼ�� "Y800"��ʾ �Ҷ�ͼ
	Image imageZbar(width, height, "Y800", raw, width * height);

	/*Mat tmp;
	tmp.create(height, width, CV_8UC1);
	tmp.data = img_gray.data;
	ShowPic("tmp", tmp);
	waitKey();*/


	scanner.scan(imageZbar); //ɨ������     

	//	���ҽ���ʶ�� 1 ������ʱ�����ظ������ֵ
	Image::SymbolIterator symbol = imageZbar.symbol_begin();
	if (imageZbar.symbol_begin() == imageZbar.symbol_end())
	{
		cout << "ͼƬ�в����������롣" << endl;
		code = "";
		return false;
	}
	else
	{
		
		code = symbol->get_data();
		
		if (++symbol == imageZbar.symbol_end())
		{
			cout << "�ҵ����룺" << code << endl;
			return true;
		}
		else
		{
			cout << "�����������࣬��������!" << endl;
			code = "";
			return false;
		}
	}

}

bool Cam::LineCrop(const Mat & img, const vector<vector<Point>>& contours)
{
	Mat img1;
	for (int i = 0; i < contours.size(); i++)
	{
		img1.create(img.size(), CV_8UC1);
		drawContours(img1, contours, i, Scalar(255, 255, 255), 2);
		ShowPic("contour", img1);
		waitKey();

		vector<Vec4i> lines;
		
		HoughLinesP(img1, lines, 1, CV_PI/90, 100, 100, 10);

		try
		{
			cvtColor(img1, img1, CV_GRAY2BGR);
		}
		catch (...)
		{
			cout << "ת����ȳ���" << endl;
		}
		for (auto l : lines)
		{
			line(img1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 2);
		}

		ShowPic("lines", img1);
		waitKey();
	}
	return false;
}

bool Cam::ShowPic(string winName, const Mat & img, const int &time)
{
	if (!PIC_DISP) return false;
	try 
	{
		Mat tmp = img;
		while (tmp.rows > 1500 || tmp.cols > 1500)
			pyrDown(tmp, tmp);
		imshow(winName, tmp);
		waitKey(time);
	}
	catch (...)
	{
		return false;
	}
	return true;
}
