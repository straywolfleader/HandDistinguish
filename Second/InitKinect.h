#pragma once

#include <Kinect.h>		
#include <iostream>
#include <opencv2\highgui\highgui.hpp>	
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>

using namespace cv;
using namespace std;

int i = 0;

//初始化深度
int InitDepthKinect(IKinectSensor* &mySensor, IDepthFrameReader* &myReader, IDepthFrameSource* &mySource , int &width, int &height)
{
	if (mySensor == nullptr)
	{
		GetDefaultKinectSensor(&mySensor);
		mySensor->Open();
	}

	mySensor->get_DepthFrameSource(&mySource);
	IFrameDescription   * myDescription = nullptr;  //取得深度数据的分辨率
	mySource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);
	myDescription->Release();
	mySource->OpenReader(&myReader);    //打开深度数据的Reader
	return 1;
}

//初始化红外图像
int InitInfraredKinect(IKinectSensor* &mySensor, IInfraredFrameReader* &myReader, IInfraredFrameSource* &mySource, int &width, int &height)
{
	if (mySensor == nullptr)
	{
		GetDefaultKinectSensor(&mySensor);
		mySensor->Open();
	}

	mySensor->get_InfraredFrameSource(&mySource);
	IFrameDescription   * myDescription = nullptr;
	mySource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);
	mySource->OpenReader(&myReader);
	return 1;
}

//初始化骨骼
int InitBodyKinect(IKinectSensor* &mySensor, IBodyFrameReader* &myReader, IBodyFrameSource* &mySource, ICoordinateMapper* &coordinatemapper)
{
	if (mySensor == nullptr)
	{
		GetDefaultKinectSensor(&mySensor);
		mySensor->Open();
	}

	mySensor->get_BodyFrameSource(&mySource);
	mySource->OpenReader(&myReader);    
	mySensor->get_CoordinateMapper(&coordinatemapper);
	return 1;
}

//初始化彩色
int InitColorKinect(IKinectSensor* &mySensor, IColorFrameReader * &myReader, IColorFrameSource* &mySource, int &width, int &height)
{
	if (mySensor == nullptr)
	{
		GetDefaultKinectSensor(&mySensor);
		mySensor->Open();
	}

	mySensor->get_ColorFrameSource(&mySource);
	IFrameDescription   * myDescription = nullptr;
	mySource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);
	myDescription->Release();
	mySource->OpenReader(&myReader);
	return 1;
}

Mat RectIma(Mat img, DepthSpacePoint d1)
{
	int x = d1.X - 30;
	int y = d1.Y - 30;
	Mat img1(100, 100, CV_8UC1);
	Rect rect(x, y, 60, 60);
	img1 = img(rect);
	return img1;
}

void Release(IKinectSensor* &mySensor, IDepthFrameReader* &myReader, IDepthFrame* &myFrame, IDepthFrameSource* &mySource)
{
	myReader->Release();        //释放不用的变量并且关闭感应器
	mySource->Release();
	mySensor->Close();
	mySensor->Release();
}

void imshowMany(const string& winname, const vector<Mat>& _imgs)
{
	int nImg = (int)_imgs.size();

	Mat dispImg;
	int size;
	int x, y;
	// w - Maximum number of images in a row
	// h - Maximum number of images in a column
	int w, h;
	// scale - How much we have to resize the image
	float scale;
	int max;
	if (nImg <= 0)
	{
		printf("Number of arguments too small....\n");
		return;
	}
	else if (nImg > 12)
	{
		printf("Number of arguments too large....\n");
		return;
	}

	else if (nImg == 1)
	{
		w = h = 1;
		size = 300;
	}
	else if (nImg == 2)
	{
		w = 2; h = 1;
		size = 300;
	}
	else if (nImg == 3 || nImg == 4)
	{
		w = 2; h = 2;
		size = 300;
	}
	else if (nImg == 5 || nImg == 6)
	{
		w = 3; h = 2;
		size = 200;
	}
	else if (nImg == 7 || nImg == 8)
	{
		w = 4; h = 2;
		size = 200;
	}
	else
	{
		w = 4; h = 3;
		size = 150;
	}
	dispImg.create(Size(100 + size*w, 60 + size*h), CV_8UC1);
	for (int i = 0, m = 20, n = 20; i<nImg; i++, m += (20 + size))
	{
		x = _imgs[i].cols;
		y = _imgs[i].rows;
		max = (x > y) ? x : y;
		scale = (float)((float)max / size);
		if (i%w == 0 && m != 20)
		{
			m = 20;
			n += 20 + size;
		}
		Mat imgROI = dispImg(Rect(m, n, (int)(x / scale), (int)(y / scale)));
		resize(_imgs[i], imgROI, Size((int)(x / scale), (int)(y / scale)));
	}
	namedWindow(winname);
	imshow(winname, dispImg);
	i++;
	string sti;
	stringstream ss;
	ss << i;
	ss >> sti;
	string filename = "F:\\C++Kinect\\Second\\img\\" + sti + "result.jpg";
	imwrite(filename.c_str(), dispImg);
	waitKey(33);
}
