#include <iostream>
#include <Kinect.h>
#include <opencv2\highgui\highgui.hpp>
#include "InitKinect.h"

using   namespace   std;
using   namespace   cv;

int main()
{
	HRESULT hResult = S_OK;
	DepthSpacePoint d1, d2;
	IKinectSensor * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	IDepthFrameSource* pDepthSource = nullptr;
	IDepthFrameReader* pDepthReader = nullptr;
	IColorFrameSource   * pColorSource = nullptr;
	IColorFrameReader   * pColorReader = nullptr;
	IInfraredFrameSource * pInfSource = nullptr;
	IInfraredFrameReader * pInfReader = nullptr;
	IBodyFrameSource* pBodysSource = nullptr;
	IBodyFrameReader* pBodysReader = nullptr;
	ICoordinateMapper* coordinatemapper = nullptr;
	int pDepthWidth = 0, pColorWidth = 0, pInfWidth = 0;
	int pDepthHeight = 0, pColorHeight = 0, pInfHeight = 0;

	InitDepthKinect(mySensor, pDepthReader, pDepthSource, pDepthWidth, pDepthHeight);
	//InitColorKinect(mySensor, pColorReader, pColorSource, pDepthWidth, pDepthHeight);
	InitInfraredKinect(mySensor, pInfReader, pInfSource, pInfWidth, pInfHeight);
	InitBodyKinect(mySensor, pBodysReader, pBodysSource, coordinatemapper);

	while (true)
	{
		
		IBodyFrame* pBodyFrame = nullptr;
		pBodysReader->AcquireLatestFrame(&pBodyFrame); // 获取最近的一帧数据  
		if (!pBodyFrame)
		{
			Sleep(100);
			printf(".");
			continue;
		}
		IBody* ppBodies[BODY_COUNT] = { 0 };
		pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies); // 更新所有人身体数据  
		for (int i = 0; i < BODY_COUNT; ++i)
		{
			IBody* pBody = ppBodies[i]; // 轮询每个人的信息  
			if (pBody)
			{
				BOOLEAN bTracked = false;
				pBody->get_IsTracked(&bTracked); // 检测是否被跟踪，即是否有这个人  
				if (bTracked)
				{
					Joint joints[JointType_Count];
					HandState leftHandState = HandState_Unknown;
					HandState rightHandState = HandState_Unknown;
					pBody->get_HandLeftState(&leftHandState); // 获取左手的状态  
					pBody->get_HandRightState(&rightHandState); // 获取右手的状态  

					pBody->GetJoints(_countof(joints), joints); // 获取身体的骨骼信息，25点  
					
					coordinatemapper->MapCameraPointToDepthSpace(joints[JointType_HandLeft].Position, &d1);
					coordinatemapper->MapCameraPointToDepthSpace(joints[JointType_HandRight].Position, &d2);
					/*cout << d1.X << ' ' << d1.Y << endl;
					cout << d2.X << ' ' << d2.Y << endl;*/
				}
			}
		}
		for (int i = 0; i < BODY_COUNT; ++i)
		{
			ppBodies[i]->Release();
		}
		pBodyFrame->Release();
		
		Mat DepthImg(pDepthHeight, pDepthWidth, CV_8UC1);
		Mat bufferMat(pDepthHeight, pDepthWidth, CV_16UC1);
		IDepthFrame* pDepthFrame = nullptr;
		unsigned int bufferSize = pDepthWidth * pDepthHeight * sizeof(unsigned short);
		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hResult))
		{
			pDepthFrame->CopyFrameDataToArray(pDepthHeight * pDepthWidth, (UINT16 *)bufferMat.data);
			bufferMat.convertTo(DepthImg, CV_8UC1, 255.0 / 4500);
			/*imshow("Depth", DepthImg);
			waitKey(33);*/
			pDepthFrame->Release();
		}

		Mat bufImg(pInfHeight, pInfWidth, CV_16UC1);
		Mat InfImg(pInfHeight, pInfWidth, CV_8UC1);
		IInfraredFrame  * pInfFrame = nullptr;
		if (pInfReader->AcquireLatestFrame(&pInfFrame) == S_OK)
		{
			pInfFrame->CopyFrameDataToArray(pInfHeight * pInfWidth, (UINT16 *)bufImg.data);
			bufImg.convertTo(InfImg, CV_8UC1, 255.0 / 4500);
			/*imshow("InfImg", InfImg);
			waitKey(33);*/
			pInfFrame->Release();
		}
		/*
		Mat ColorImg(pColorHeight, pColorWidth, CV_8UC4);
		IColorFrame* pColorFrame = nullptr;
		if (pColorReader->AcquireLatestFrame(&pColorFrame) == S_OK)
		{
		UINT    size = 0;
		pColorFrame->CopyConvertedFrameDataToArray(pColorWidth * pColorHeight * 4, (BYTE *)ColorImg.data, ColorImageFormat_Bgra);
		imshow("TEST2", ColorImg);
		pColorFrame->Release();
		}*/

		Mat DepthLeftImg, DepthRightImg, InfLeftImg, InfRightImg;
		DepthLeftImg = RectIma(DepthImg, d1);
		DepthRightImg = RectIma(DepthImg, d2);
		InfLeftImg = RectIma(InfImg, d1);
		InfRightImg = RectIma(InfImg, d2);
		/*imshow("DepthLeftHand", DepthLeftImg);
		imshow("DepthRightHand", DepthRightImg);
		imshow("InfLeftHand", InfLeftImg);
		imshow("InfRightHand", InfRightImg);*/
		vector<Mat> Mimg;
		Mimg.push_back(DepthRightImg);
		Mimg.push_back(DepthLeftImg);
		Mimg.push_back(InfLeftImg);
		Mimg.push_back(InfRightImg);
		imshowMany("img", Mimg);
	}


	pDepthReader->Release();
	pDepthSource->Release();
	pColorReader->Release();
	pColorSource->Release();
	pInfReader->Release();
	pInfSource->Release();
	pBodysReader->Release();
	pBodysSource->Release();
	mySensor->Close();
	mySensor->Release();
	return 0;
}