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
	// 初始化
	IGXFactory::GetInstance().Init();


	//	枚举设备
	GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;

	IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);

	cout << "获取到的设备列表：" << endl;
	for (uint32_t i = 0; i < vectorDeviceInfo.size(); i++)

	{

		cout << vectorDeviceInfo[i].GetVendorName() << endl;

		cout << vectorDeviceInfo[i].GetModelName() << endl;

	}


	//	打开设备
	//  可通过以下几种方式打开

	/*
	IGXFactory::GetInstance().OpenDeviceBySN

	IGXFactory::GetInstance().OpenDeviceByUserID

	IGXFactory::GetInstance().OpenDeviceByMAC

	IGXFactory::GetInstance().OpenDeviceByIP
	*/


	//打开链表中的第一个设备
	

	if (vectorDeviceInfo.size()> 0)

	{

		GxIAPICPP::gxstring strSN = vectorDeviceInfo[0].GetSN();

		GxIAPICPP::gxstring strUserID = vectorDeviceInfo[0].GetUserID();

		GxIAPICPP::gxstring strMAC = vectorDeviceInfo[0].GetMAC();

		GxIAPICPP::gxstring strIP = vectorDeviceInfo[0].GetIP();


		objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(strSN, GX_ACCESS_EXCLUSIVE);

		//	设备已经打开
		isOpen = true;


		//objDevicePtr = IGXFactory::GetInstance().OpenDeviceByUserID(strUserID, GX_ACCESS_EXCLUSIVE);
		//objDevicePtr = IGXFactory::GetInstance().OpenDeviceByMAC(strMAC, GX_ACCESS_EXCLUSIVE);
		//objDevicePtr = IGXFactory::GetInstance().OpenDeviceByIP(strIP, GX_ACCESS_EXCLUSIVE);


		//获取远端设备属性控制器
		objFeatureControlPtr = objDevicePtr->GetRemoteFeatureControl();

		//	获取 流
		objStreamPtr = objDevicePtr->OpenStream(0);
	}



	return isOpen == true;
}

bool Cam::CloseDevice()
{


	objStreamPtr->Close();

	objDevicePtr->Close();

	//	反初始化，释放GxIAPICPP申请的所有资源
	IGXFactory::GetInstance().Uninit();

	//	设备已经关闭
	isOpen = false;
	return isOpen == false;
}

bool Cam::GetImage(Mat &img)
{
	//	在开启了"流通道采集"和"发送开采命令"之后,就可以通过"流通道采集"获取单帧图像了

	//	采集完图像后,在恰当的时候 ,先 "停止采集", 再停止流通道采集

	//	流通道采集先开,后关


	//	需要先判断 isOpen
	if (!isOpen)
	{
		return false;
	}

	//	从 流 中开启通道
	objStreamPtr->StartGrab();

	//	开始采集
	objFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();



	//采单帧

	CImageDataPointer objImageDataPtr;


	objImageDataPtr = objStreamPtr->GetImage(500);//超时时间使用500ms，用户可以自行设定

	if (objImageDataPtr->GetStatus() == GX_FRAME_STATUS_SUCCESS)

	{

		//采图成功而且是完整帧，可以进行图像处理...
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
			cout << "图片复制出错！" << endl;
			return false;
		}


		//	针对托盘区域裁剪图像
		ShowPic("采集的图像", img);
		
		Rect r = Rect(0, 0, img.cols, img.rows);
		img = img(r);
		ShowPic("采集的图像", img);

	}



	//发送停采命令

	objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

	objStreamPtr->StopGrab();

	return true;
}

bool Cam::PicPreProcess(const Mat & img, vector<Mat>& res)
{
	//	输入图像，经过
	//	灰度图->高斯滤波->阈值分割->边缘检测
	//	最后输出边缘检测结果：边缘二值图、边缘法向角度图



	//	灰度图
	Mat img_gray;
	try
	{
		cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

	}
	catch (...)
	{
		//cout << "已经是灰度图，无需转换" << endl;
		img_gray = img;
	}

	//pyrDown(img_gray, img_gray);
	//pyrDown(img_gray, img_gray);

	//rotate(img_gray, img_gray, ROTATE_90_CLOCKWISE);

	//	中值滤波
	Mat img_mid = img_gray;
	medianBlur(img_gray, img_mid, 5);

	//	高斯模糊
	Mat img_gauss = img_mid;
	GaussianBlur(img_gray, img_gauss, cv::Size(3, 3), 0, 0);


	//	阈值分割
	Mat img_bin;
	threshold(img_gauss, img_bin, 100, 255, cv::THRESH_OTSU | cv::THRESH_BINARY_INV);

	res.push_back(img_bin);


	//	边缘检测
	Mat img_schx, img_schy;

	try
	{
		Scharr(img_bin, img_schx, CV_32F, 1, 0);
		Scharr(img_bin, img_schy, CV_32F, 0, 1);
	}
	catch (...)
	{
		cout << "边缘检测出错！" << endl;
	}


	//	根据检测的边缘分量，计算边缘角度
	Mat img_gra;
	Mat img_dir;	//	[0，2*pi]

	try
	{
		cartToPolar(img_schx, img_schy, img_gra, img_dir, true);
	}
	catch (...)
	{
		cout << "梯度方向计算出错！" << endl;
	}


	Mat img_mag;

	try
	{
		img_gra.convertTo(img_mag, CV_8UC1);
	}
	catch (...)
	{
		cout << "边缘图像转换出错！" << endl;
	}

	//std::vector<Mat> res;
	//res.push_back(img_gra_bin);
	res.push_back(img_mag);

	//	类型为 CV_32F， 对应 float
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
//		// 绘制形心
//		for (auto it : contours) 
//		{
//			//计算所有的力矩
//			Moments mom = cv::moments(cv::Mat(it));
//			//绘制质心
//			Point center = Point(mom.m10 / mom.m00, mom.m01 / mom.m00);
//
//			//	将形心压入输出数组
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
//		cout << "获取形心失败！" << endl;
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
			Println("像素点坐标超出范围！");
		}

		//	径向畸变
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
	//	将图像二值化，二值化后
	//	原本 深色 的部分（如条码）将被设为 白色
	Mat bin_img;
	GetBinImg(img, bin_img);

	//	膨胀腐蚀后，获取ROI
	int k_size = 20;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(k_size, k_size));

	//	要提取的部分为黑色，（白色部分）膨胀2次， 腐蚀3次
	int k = 2;
	int kk = k;
	while(kk--) dilate(bin_img, bin_img, kernel);
	
	kk = k + 1;
	while(kk--) erode(bin_img, bin_img, kernel);

	//ShowPic("膨胀*" + to_string(k) + "后腐蚀*" + to_string(k + 1), bin_img);

	

	//	找寻轮廓
	vector<cv::Vec4i> hierar;
	vector<vector<Point>> contours;
	findContours(bin_img, contours, hierar, RETR_TREE, CHAIN_APPROX_SIMPLE);

	cout << "找到轮廓数目：" << contours.size() << endl;

	//	绘制轮廓
	Mat img1;
	cvtColor(bin_img, img1, cv::COLOR_GRAY2BGR);

	drawContours(img1, contours, -1, Scalar(0, 0, 255), 2);

	ShowPic("contours", img1, 1000);


	//	条码 --- 中心位置 对应
	map<string, Point> code2pos;

	vector<vector<Point>> code_contours;

	//	针对每个轮廓，提取矩形区域，并识别条码
	for (auto contour : contours)
	{
		Rect rect = boundingRect(contour);

		Point center = Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
		vector<Point> centers;

		Mat img_roi = img(rect);
		ShowPic("img_roi", img_roi, 200);


		string code;
		if (ReadBar(img_roi, code))
		{
			Point cen;
			GetCenter(img_roi, cen);
			code2pos[code] = center;
			cout << "条码位置：" << center << endl;

			//	存在单个条码的区域，接下来精细提取轮廓
			code_contours.push_back(contour);
			 
		}
	}

	if (barcodes.size() == 0) return false;

	std::sort(barcodes.begin(), barcodes.end());

	
	return true;
}

bool Cam::GetBinImg(const Mat & img, Mat & bin_img)
{
	//	灰度图
	Mat img_gray;
	try
	{
		cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

	}
	catch (...)
	{
		img_gray = img;
	}

	cout << "图像大小：" << img_gray.size() << endl;
	//if (img_gray.size[0] > 3000)
	//{
	//	pyrDown(img_gray, img_gray);
	//	pyrDown(img_gray, img_gray); 
	//}
	//rotate(img_gray, img_gray, ROTATE_90_CLOCKWISE);

	//	中值滤波
	Mat img_mid = img_gray;
	medianBlur(img_gray, img_mid, 5);

	//	高斯模糊
	Mat img_gauss = img_mid;
	GaussianBlur(img_gray, img_gauss, cv::Size(3, 3), 0, 0);


	//	先跳过滤波处理
	img_gauss = img_gray;

	ShowPic("灰度图", img_gray, 1000);


	//	阈值分割
	Mat img_bin;
	threshold(img_gauss, img_bin, 60, 255, /*cv::THRESH_OTSU |*/ cv::THRESH_BINARY_INV);

	bin_img = img_bin;

	//ShowPic("二值图", img_bin);

	return true;
}

bool Cam::ReadBar(const Mat & img_roi, string & code)
{
	ImageScanner scanner;

	//	扫描对象设置
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

	int width = img_gray.cols;
	int height = img_gray.rows;
	uchar *raw = (uchar *)img_gray.data;

	//	初始化一副 zbar图像， "Y800"表示 灰度图
	Image imageZbar(width, height, "Y800", raw, width * height);

	/*Mat tmp;
	tmp.create(height, width, CV_8UC1);
	tmp.data = img_gray.data;
	ShowPic("tmp", tmp);
	waitKey();*/


	scanner.scan(imageZbar); //扫描条码     

	//	当且仅当识别到 1 个条码时，返回该条码的值
	Image::SymbolIterator symbol = imageZbar.symbol_begin();
	if (imageZbar.symbol_begin() == imageZbar.symbol_end())
	{
		cout << "图片中不存在条形码。" << endl;
		code = "";
		return false;
	}
	else
	{
		
		code = symbol->get_data();
		
		if (++symbol == imageZbar.symbol_end())
		{
			cout << "找到条码：" << code << endl;
			return true;
		}
		else
		{
			cout << "条码数量过多，轮廓过大!" << endl;
			code = "";
			return false;
		}
	}

}

void Cam::GetCenter(const Mat & img_roi, Point & pos)
{
	Mat img_bin;


	Mat img_hist;
	cvtColor(img_roi, img_hist, cv::COLOR_BGR2GRAY);

	equalizeHist(img_hist, img_hist);

	threshold(img_hist, img_bin, 50, 255, THRESH_BINARY_INV);
	ShowPic("roi_with_single_code", img_bin);


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
			Println("深度转换出错！");
		}
		for (auto l : lines)
		{
			line(img1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 2);
		}

		ShowPic("lines", img1);

		//	边角延长相交

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
