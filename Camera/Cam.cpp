#include "stdafx.h"
#include "Cam.h"

using namespace zbar;


Cam::Cam()
{
	PIC_DISP_ON = true;
	PRINT_ON = true;
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


		//	�����������ü�ͼ��
		ShowPic("�ɼ���ͼ��", img);
		
		Rect r = Rect(0, 0, img.cols, img.rows);
		img = img(r);
		ShowPic("�ɼ���ͼ��", img);

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

//bool Cam::GetCenters(const Mat & bin_img, vector<Point>& centers)
//{
//	try
//	{
//
//		Mat img;
//		cvtColor(bin_img, img, cv::COLOR_GRAY2BGR);
//
//		vector<vector<Point>> contours;
//		findContours(bin_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//		drawContours(img, contours, -1, Scalar(0, 0, 255), 2);
//
//		// ��������
//		for (auto it : contours) 
//		{
//			//�������е�����
//			Moments mom = cv::moments(cv::Mat(it));
//			//��������
//			Point center = Point(mom.m10 / mom.m00, mom.m01 / mom.m00);
//
//			//	������ѹ���������
//			centers.push_back(center);
//
//			cout << center << endl;
//			circle(img, center, 2, cv::Scalar(0,255,0), 3);
//		}
//	
//		string winName = "contours";
//		ShowPic(winName, img);
//		moveWindow(winName, 200, 200);
//
//		return true;
//	}
//	catch (...)
//	{
//		cout << "��ȡ����ʧ�ܣ�" << endl;
//		return false;
//	}
//}

bool Cam::TransCordinate(const vector<Point> &centers, vector<Point2d>& dst)
{
	dst.clear();
	for (Point p : centers)
	{

		Point2d after;
		if(TransCordinate(p, after)) dst.push_back(after);
		else
		{
			Println("���ص����곬����Χ��");
		}

		//	�������
		//	[k1, k2] = [-0.1016, 0.0818]
		//	x' = x(1 + k1*r^2 + k2*r^4)

	}

	return false;
}

bool Cam::TransCordinate(const Point &p, Point2d & dst)
{
	if (p.x < 0 || p.x > CAM_WIDTH || p.y < 0 || p.y > CAM_HEIGHT) return false;
	dst.x = (p.x - u0) / fx;
	dst.y = (p.y - v0) / fy;
	return true;
}

bool Cam::GetCodes(const Mat & img, vector<string>& barcodes, map<string, Point>& pos)
{
	//	��ͼ���ֵ������ֵ����
	//	ԭ�� ��ɫ �Ĳ��֣������룩������Ϊ ��ɫ
	Mat bin_img;
	GetBinImg(img, bin_img);

	//	���͸�ʴ�󣬻�ȡROI
	int k_size = 20;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(k_size, k_size));

	//	Ҫ��ȡ�Ĳ���Ϊ��ɫ������ɫ���֣�����2�Σ� ��ʴ3��
	//	�ฯʴһ��ȷ������ȡROI����Ŀ��߽�
	int k = 2;
	int kk = k;
	while(kk--) dilate(bin_img, bin_img, kernel);
	
	kk = k + 3;
	while(kk--) erode(bin_img, bin_img, kernel);



	//	��ȡ����
	vector<vector<Point>> contours;
	GetContours(bin_img, contours);
	ShowContours(bin_img, contours);

	//	���� --- ����λ�� ��Ӧ
	map<string, Point> code2pos;


	//	��ȡ�������������ĵ㣬����ʾ
	Mat cen_img = img;
	
	//	��ͼ��ת�ɲ�ɫ���Ա���Ʋ�ɫͼ��
	cvtColor(cen_img, cen_img, COLOR_BGR2GRAY);
	cvtColor(cen_img, cen_img, COLOR_GRAY2BGR);

	
	//	���ÿ����������ȡ�������򣬲�ʶ������
	for (auto contour : contours)
	{
		//	��װ��������״������������������Ҫ12����
		if (contour.size() < MIN_POINT) continue;


		Rect rect = boundingRect(contour);

		Mat img_roi = img(rect);
		

		//	ɨ��ROI���Ƿ��е�������
		string code;
		if (ReadBar(img_roi, code))
		{
			//	���ڵ�����������򣬽�������ϸ��ȡ��������������
			ShowPic("��ROI�д�������", img_roi, 500);

			Point cen;
			vector<Point> accu_contour;
			GetCenter(img_roi, cen, accu_contour);
			
			for (Point &p : accu_contour)
			{
				p.x += rect.x;
				p.y += rect.y;
			}
			//	cen����ΪROI�е����꣬��Ҫ��������ͼ���ϵľ�������
			cen.x += rect.x;
			cen.y += rect.y;

			//	�����ĵ���ͼ�ϻ���
			circle(cen_img, cen, 24, Scalar(0, 0, 255), -1);
			ShowContours(cen_img, accu_contour, "�ҵ��������ĵ�");

			code2pos[code] = cen;

			Println("\n");
			Println("�ҵ����룺", code);
			Println("��������λ�ã�", cen);
			Point2d std_pos;
			TransCordinate(cen, std_pos);
			Println("�����һ��λ�ã�", std_pos);
			Println("\n");
		}
	}

	if (barcodes.size() == 0) return false;

	std::sort(barcodes.begin(), barcodes.end());

	
	return true;
}

bool Cam::GetBinImg(const Mat & img, Mat & bin_img, const int &thresh)
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

	Println("ͼ���С��", img_gray.size());
	

	//	��ֵ�˲�
	Mat img_mid = img_gray;
	medianBlur(img_gray, img_mid, 5);

	//	��˹ģ��
	Mat img_gauss = img_mid;
	GaussianBlur(img_gray, img_gauss, cv::Size(3, 3), 0, 0);


	//	�������˲�����
	img_gauss = img_gray;

	ShowPic("�Ҷ�ͼ", img_gray, 1000);


	//	��ֵ�ָ�
	Mat img_bin;
	threshold(img_gauss, img_bin, thresh, 255, cv::THRESH_BINARY_INV);

	bin_img = img_bin;

	return true;
}

void Cam::GetContours(const Mat & bin_img, vector<vector<Point>>& contours)
{
	//	��Ѱ����
	vector<cv::Vec4i> hierar;
	contours.clear();
	findContours(bin_img, contours, hierar, RETR_TREE, CHAIN_APPROX_SIMPLE);

}

void Cam::ShowContours(const Mat & img, vector<vector<Point>>& contours, const string &str, const Scalar &color)
{
	//	��������
	Mat img1 = img;
	try
	{
		cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
	}
	catch (...)
	{
		;
	}

	drawContours(img1, contours, -1, color, 2);

	ShowPic(str, img1, 1000);
}

void Cam::ShowContours(const Mat & img, vector<Point>& contour, const string &str, const Scalar &color)
{
	//	��������
	Mat img1;
	img.copyTo(img1);
	img1 = img;
	try 
	{
		cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
	}
	catch (...)
	{
		;
	}
	vector<vector<Point>> contours;
	contours.push_back(contour);
	drawContours(img1, contours, -1, color, 8);

	ShowPic(str, img1, 800);
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
	Mat img_bin = img_gray;

	//imshow("img_bin", img_bin);
	//waitKey();
	

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
		//cout << "ͼƬ�в����������롣" << endl;
		code = "";
		return false;
	}
	else
	{
		
		code = symbol->get_data();
		
		if (++symbol == imageZbar.symbol_end())
		{
			//cout << "�ҵ����룺" << code << endl;
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

void Cam::GetCenter(const Mat & img_roi, Point & pos, vector<Point> &accu_contour, const int &thresh)
{
	Mat bin_img;
	cvtColor(img_roi, bin_img, cv::COLOR_BGR2GRAY);

	threshold(bin_img, bin_img, thresh, 255, THRESH_BINARY_INV);


	//	��Ѱ����
	vector<vector<Point>> contours;
	GetContours(bin_img, contours);
	
	//	���ÿ����������ȡ�������򣬲�ʶ������
	int i = 0;
	for (auto contour : contours)
	{
		i++;
		if (contour.size() < MIN_POINT) continue;

		Rect rect = boundingRect(contour);
		Mat roi = img_roi(rect);


		cvtColor(roi, roi, COLOR_BGR2GRAY);
		cvtColor(roi, roi, COLOR_GRAY2BGR);

		string code;
		if (ReadBar(roi, code))
		{
			//�������е�����
			Moments mom = cv::moments(cv::Mat(contour));
			//��������
			pos = Point(mom.m10 / mom.m00, mom.m01 / mom.m00);

			circle(img_roi, pos, 12, Scalar(0, 255, 255), -1);
			ShowContours(img_roi, contour, "�ҵ�����ľ�ϸ����", Scalar(0, 255, 255));
			
			//	pos����ΪROI�е����꣬��Ҫ��������ͼ���ϵľ�������
			pos.x += rect.x;
			pos.y += rect.y;
		
			accu_contour = contour;
			return;

			//Mat img = img_roi;
			//cvtColor(img, img, COLOR_BGR2GRAY);
			//cvtColor(img, img, COLOR_GRAY2BGR);

			//circle(img, pos, 12, cv::Scalar(0,255,0), -1);

			//ShowPic("�ҵ�������", img);
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

		vector<Vec4i> lines;
		
		HoughLinesP(img1, lines, 1, CV_PI/90, 100, 100, 10);

		try
		{
			cvtColor(img1, img1, CV_GRAY2BGR);
		}
		catch (...)
		{
			Println("���ת������");
		}
		for (auto l : lines)
		{
			line(img1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 2);
		}

		ShowPic("lines", img1);

		//	�߽��ӳ��ཻ

	}
	return false;
}

bool Cam::ShowPic(string winName, const Mat & img, const int &time)
{
	if (!PIC_DISP_ON) return false;
	try 
	{
		Mat tmp = img;
		while (tmp.rows > 1000 || tmp.cols > 1000)
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

void Cam::Println(string str)
{
	if(PRINT_ON) cout << str << endl;
}

template <class T>
void Cam::Println(string str, const T t)
{
	if (PRINT_ON) cout << str << t << endl;
}
