#pragma warning(disable : 4996)
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _USE_MATH_DEFINES

//Standard C headers
#include <stdio.h>
#include <time.h>
#include <conio.h>
#include <math.h>
#include <assert.h>
#include <atomic>
#include <thread>
#include <WinSock2.h>
#include <iostream>
#include <chrono>
#include <queue>
#include <cmath>
#include <algorithm>

//OpenCV Headers
#include <opencv/highgui.h>
#include <opencv2/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
//#include <getopt.h>

//Timer Lib
#include "../3rdPartyLibs/tictoc.h"

//My Personal Headers
#include "Talk2MomentCamera.h"
#include "Talk2WStage.h"
#include "Talk2TLDC4104.h"
#include "NeuronAnalysis.h"
#include "AndysOpenCVLib.h"
#include "AndysComputations.h"
#include "experiment.h"
#include "WriteOutNeuron.h"

#pragma comment(lib, "Ws2_32.lib")

//UDP Sender Func
UDPSender::UDPSender() : run_thread(false), message('0'), 
	volChannel('0'), volDir('0'), volWave('0') {
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		std::cerr << "Winsock initialization failed." << std::endl;
		exit(EXIT_FAILURE);
	}

	if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
		std::cerr << "Socket creation failed with error: " << WSAGetLastError() << std::endl;
		WSACleanup();
		exit(EXIT_FAILURE);
	}

	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(557);
	servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
}

UDPSender::~UDPSender() {
	if (run_thread) {
		stop();
	}
	closesocket(sockfd);
	WSACleanup();
}

void UDPSender::send_udp() {
	while (run_thread) {
		char buffer[7] = { volChannel, ':', volWave, '!', volDir, message, '\0'};
		if (sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr*)&servaddr, sizeof(servaddr)) == SOCKET_ERROR) {
			std::cerr << "Send failed with error: " << WSAGetLastError() << std::endl;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
}

void UDPSender::start() {
	run_thread = true;
	udp_thread = std::thread(&UDPSender::send_udp, this);
}

void UDPSender::stop() {
	run_thread = false;
	if (udp_thread.joinable()) {
		udp_thread.join();
	}
}

void UDPSender::changeMessage(char newChannel, char newDir, char newMessage, char newWave) {
	if (run_square_wave) {
		stopSquareWave();  // 如果正在运行，先停止现有的方波
	}

	message = newMessage;
	volChannel = newChannel;
	volDir = newDir;
	volWave = newWave;
	//if(newMessage == '1')	//原电场刺激模式采用指针形式，指针为1时对应电压为0；指针为其他值时对应其余电压。
	if (newMessage == '0')	//0为默认零输入
		curType = WaveType::None;
	else
		curType = WaveType::DC;
}

void UDPSender::SquareWave(char Amplitude, int Frequency, int DutyCycle) {
	if (run_square_wave) {
		run_square_wave = false;  // 停止方波
		if (square_wave_thread.joinable()) {
			square_wave_thread.join();  // 等待线程完成
		}
	}

	run_square_wave = true;  // 标记方波为运行状态
	curType = WaveType::Square;

	square_wave_thread = std::thread([this, Amplitude, Frequency, DutyCycle]() {
		int period = 1000 / Frequency;//一个周期总时间
		int highTime = period * DutyCycle / 100;//高电平维持时间

		while (run_square_wave) {
			this->message = Amplitude;
			std::this_thread::sleep_for(std::chrono::milliseconds(highTime));
			this->message = '0';//指针0对应负电压-4V
			std::this_thread::sleep_for(std::chrono::milliseconds(period - highTime));
		}
		});
	square_wave_thread.detach();  // 让线程在后台运行
}

void UDPSender::stopSquareWave() {
	if (run_square_wave) {
		run_square_wave = false;  // 停止方波
		if (square_wave_thread.joinable()) {
			square_wave_thread.join();  // 等待线程完成
		}
	}
	//message = '1';  // 设置电压为0
	message = '0';
	curType = WaveType::None;
}

//Experiment

/*
 * Creates a new experiment object and sets values to zero.
 */
Experiment* CreateExperimentStruct() {

	/** Create Experiment Object **/
	Experiment* exp;
	exp = (Experiment*)malloc(sizeof(Experiment));

	/*************************************/
	/**  Set Everything to zero or NULL **/
	/*************************************/

	/** GuiWindowNames **/
	exp->WinDisp = "Display";
	exp->WinCon1 = "Control";

	/** Error information **/
	exp->e = 0;

	/** CommandLine Input **/
	exp->argv = NULL;
	exp->argc = 0;

	/** Camera Input**/
	exp->MyCamera = NULL;
	exp->FramePerSec = 0;  

	/** User-configurable Neuron-related Parameters **/
	exp->Params = NULL;

	/** Information about Our Neuron **/
	exp->Neuron = NULL;

	/** internal IplImage **/
	exp->SubSampled = NULL; // Image used to subsample stuff
	exp->lastFrameSeenOutside = 0;
	exp->CurrentSelectedImg = NULL; //The current image selected for display

	/** Write Data To File **/
	exp->DataWriter = NULL;
	exp->outfname = (char*)malloc(1000 * sizeof(char));
	exp->infname = (char*)malloc(1000 * sizeof(char));
	exp->dirname = (char*)malloc(1000 * sizeof(char));

	strcpy(exp->outfname, "w1");
	strcpy(exp->dirname, "D:/Hasuyo/Code_files/C++_Code/WormTrackingSys/w1/");

	/** Write Video To File **/
	//exp->Vid = NULL; //Video Writer
	exp->vid_writer = NULL;
	exp->standardCorX = 0;
	exp->standardCorY = 0;
	exp->Voltage = 0;
	exp->cur_velocity = 0;

	/** Timing  Information **/
	exp->now = 0;
	exp->last = 0;

	/** Frame Rate Information **/
	exp->nframes = 0;
	exp->prevFrames = 0;
	exp->prevTime = 0;

	/** Stage Control **/
	exp->MyStage = NULL;
	exp->stageIsPresent = 1;
	exp->stage = 0;
	exp->stageVel = cvPoint(0, 0);
	exp->stageCenter = cvPoint(0, 0);
	exp->stageFeedbackTargetOffset = cvPoint(0, 0);
	exp->stageIsTurningOff = 0;
	exp->x_initPos = 0;
	exp->y_initPos = 0;
	exp->NemDisp = nullptr;

	/** TLDC LED input **/
	exp->MyLed = NULL;

	/** Error Handling **/
	exp->e = 0;

	exp->sender = NULL;

	exp->pos_accumulate = cvPoint(0, 0);
	exp->pos_num = 0;

	return exp;
}

/*
 * This function allocates images and frames
 * And a Neuron Object
 *
 * And a Parameter Object
 * For internal manipulation
 *
 *
 */
void InitializeExperiment(Experiment* exp) {

	/*** Create IplImage **/
	IplImage* SubSampled = cvCreateImage(cvSize(NSIZEX / 2, NSIZEY / 2),
		IPL_DEPTH_8U, 1);
	exp->CurrentSelectedImg = cvCreateImage(cvSize(NSIZEX, NSIZEY),IPL_DEPTH_8U, 1);
	exp->SubSampled = SubSampled;

	/*** Create Frames **/
	Frame* fromCCD = CreateFrame(cvSize(NSIZEX, NSIZEY));

	exp->fromCCD = fromCCD;

	/** Create Neuron Data Struct and Neuron Parameter Struct **/
	NeuronAnalysisData* Neuron = CreateNeuronAnalysisDataStruct();
	NeuronAnalysisParam* Params = CreateNeuronAnalysisParam();
	InitializeEmptyNeuronImages(Neuron, cvSize(NSIZEX, NSIZEY));
	InitializeNeuronMemStorage(Neuron);

	/* Create Neuron Features */
	Neuron->Head = (CvPoint*)malloc(sizeof(CvPoint));
	Neuron->Tail = (CvPoint*)malloc(sizeof(CvPoint));
	Neuron->prevHead = (CvPoint*)malloc(sizeof(CvPoint));
	Neuron->prevTail = (CvPoint*)malloc(sizeof(CvPoint));
	Neuron->Centerline = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), Neuron->MemCenterLine); //最后一个参数不知是否正确

	//将维护的线虫队列初始化
	exp->NemDisp = new std::queue<double>();

	exp->Neuron = Neuron;
	exp->Params = Params;

	exp->sender = new UDPSender;
	exp->sender->start();

	exp->vid_writer = new cv::VideoWriter();
}

/*
 * Load the command line arguments into the experiment object
 */
void LoadCommandLineArguments(Experiment* exp, int argc, char** argv) {
	exp->argc = argc;
	exp->argv = argv;
}

/*
 * Initialize camera library
 * Allocate Camera Data
 * Select Camera and Show Properties dialog box
 * Start Grabbing Frames as quickly as possible
 * *
 * OR open up the video file for reading.
 */
void RollCameraInput(Experiment* exp) {

	/** Use Moment Camera **/
	int ret = 0;
	/** Turn on Camera **/
	T2Cam_ShowPvcamInfo(exp->argc, exp->argv);
	
	//给exp里的MyCamera分配内存
	if (!T2Cam_AllocateCamDataStructMemory(&(exp->MyCamera)))
	{  
		exp->e = EXP_ERROR;
		return;
	}

	//让Pvcam自动识别相机并初始化并打开相机
	if (!T2Cam_InitAndOpenSpecificCamera(*(exp->MyCamera)))
	{
		exp->e = EXP_ERROR;
		return;
	}

	//让相机开始工作，并尽可能快的拍照
	if (!T2Cam_GrabFramesAsFastAsYouCan(*(exp->MyCamera)))
	{
		exp->e = EXP_ERROR;
		return;
	}
}

/* Init TLDC LED */
void RollLedInput(Experiment* exp)
{
	if (!AllocateLedDataStructMemory(&(exp->MyLed)))
	{
		exp->e = EXP_ERROR;
		return;
	}

	TLDC_InitLed(*(exp->MyLed));

	if (!TLDC_ComScan(*(exp->MyLed)))
	{
		std::cout << "Scan Error" << std::endl;
		exp->e = EXP_ERROR;
		return;
	}

	if (!TLDC_ComActivateAndSetOff(*(exp->MyLed)))
	{
		std::cout << "Activeta ERROR" << std::endl;
		exp->e = EXP_ERROR;
		return;
	}

	TLDC_SetBrightnessPerc(*(exp->MyLed), LED0, exp->Params->BrightnessPerc);
	TLDC_SetBrightnessPerc(*(exp->MyLed), LED1, exp->Params->BrightnessPerc);

}

/*********************************************
 *
 * Image Acquisition
 *
 */

 /** Grab a Frame from either camera or video source
  *
  */
int GrabFrame(Experiment* exp) {

	// Retrieve an image from the Grab Result of the camera before further step.
	if(!T2Cam_RetrieveAnImage(*(exp->MyCamera)))
		return EXP_ERROR;

	/** Acquire from Camera **/
	if (exp->MyCamera->iFrameNumber > exp->lastFrameSeenOutside)
		exp->lastFrameSeenOutside = exp->MyCamera->iFrameNumber;
	else
		return EXP_ERROR;
	/*** Create a local copy of the image***/
	LoadFrameWithBin(exp->MyCamera->iImageData, exp->fromCCD);

	exp->Neuron->frameNum++;
	return EXP_SUCCESS;
}

/*
 * Free up all of the different allocated memory for the
 * experiment.
 *
 */
void ReleaseExperiment(Experiment* exp) {
	/** Free up Frames **/
	if (exp->fromCCD != NULL)
		DestroyFrame(&(exp->fromCCD));

	/** Free up Neuron Objects **/
	if (exp->Neuron != NULL) {
		DestroyNeuronAnalysisDataStruct((exp->Neuron));
		exp->Neuron = NULL;
	}

	if (exp->Params != NULL) {
		DestroyNeuronAnalysisParam((exp->Params));
		exp->Params = NULL;
	}

	/** Free up internal iplImages **/
	if (exp->CurrentSelectedImg != NULL)
		cvReleaseImage(&(exp->CurrentSelectedImg));
	
	if (exp->fromCCD != NULL)
		DestroyFrame(&(exp->fromCCD));

	/* Free up Writer and Filename Address */
	if (exp->dirname != NULL) free(exp->dirname);
	if (exp->outfname != NULL) free(exp->outfname);
	if (exp->infname != NULL) free(exp->infname);

	if(exp->MyLed != NULL)
		TLDC_UnInitLed(*(exp->MyLed));

	/** Release Stage**/
	if (exp->MyStage != NULL)
		free(exp->MyStage);

	if (exp->MyCamera != NULL)
		free(exp->MyCamera);

	if (exp->MyLed != NULL)
		free(exp->MyLed);

	/** Release Udp Sender 停止发送信号 **/
	exp->sender->stop();
	if (exp->sender != NULL)
		delete exp->sender;

	//std::cout << "exp has been released!" << std::endl;

	if (exp->vid_writer != NULL)
		delete exp->vid_writer;
}

/* Destroy the experiment object.
 * To be run after ReleaseExperiment()
 */
void DestroyExperiment(Experiment** exp) {
	free(*exp);
	*exp = NULL;

	//std::cout << "exp has been destoried!" << std::endl;
}

/*
 * SetupGui
 *
 */
void SetupGUI(Experiment* exp) {

	//printf("Beginning to setup GUI\n");

	auto windisp = exp->WinDisp;
	auto wincon = exp->WinCon1;
	auto param = exp->Params;

	cv::namedWindow(windisp, CV_WINDOW_NORMAL); // <-- This goes into the thread.
	cv::namedWindow(wincon);
	cv::resizeWindow(windisp, 800, 600);
	cv::resizeWindow(wincon, 600, 650);

	/** SelectDisplay **/
	cv::createTrackbar("SelectDisplay", wincon, &(param->Display), 2, NULL);
	//printf("Pong\n");

	/** On Off **/
	cv::createTrackbar("On", wincon, &(param->OnOff), 1, NULL);

	/** Control Led ON **/
	cv::createTrackbar("Led0_On", wincon, &(param->Led0_status), 1, NULL);

	cv::createTrackbar("Brightness", wincon, &(param->BrightnessPerc), 100, NULL);

	/** Segmentation Parameters**/
	cv::createTrackbar("MaskSelect", wincon, &(param->MaskStyle), 1, NULL);   //2024.6.22添加mask选择控件

	cv::createTrackbar("Threshold", wincon, &(param->BinThresh), 255,NULL);

	cv::createTrackbar("Diameter", wincon, &(param->MaskDiameter), NSIZEY, NULL);

	cv::createTrackbar("LowBoundary", wincon, &(param->LowBoundary), NSIZEY, NULL);

	cv::createTrackbar("RectHeigh", wincon, &(param->RectHeigh), NSIZEY, NULL);

	/** Segmentation Parameters**/
	// cv::createTrackbar("YMoveLock", wincon, &(param->YMoveLock), 1, NULL);

	cv::createTrackbar("StageSpeedFactor", wincon, &(param->stageSpeedFactor), 10, NULL);

	//cv::createTrackbar("MaxStageSpeed", wincon, &(param->maxstagespeed), 5, NULL);

	cv::createTrackbar("TargetSeg", wincon, &(param->stageTargetSegment), 100, NULL);

	/** 电场强度参数 **/
	cv::createTrackbar("EF_Mode", wincon, &(param->EF_Mode), 2, NULL);

	//printf("Created trackbars and windows\n");

	return;
}

void saveastiff(IplImage* frame)
{
	cv::Mat matFrame = cv::cvarrToMat(frame);
	static int count = 0;
	std::string filename = "D:/Hasuyo/Code_files/C++_Code/WormTrackingSys/image_" + std::to_string(count) + ".tiff";
	cv::imwrite(filename, matFrame);
	std::cout << "Image saved as " << filename << std::endl;
	count++;
}

void contiMoveAndSave(Experiment* exp)
{
	for (int i = 0; i <= 10; i++)
	{
		std::cout << i << " Left!" << std::endl;
		MoveRelativeWait(exp->MyStage->handle, (float)(0 - 0.05), AXIS_X);
		if (FindStagePosition(*(exp->MyStage)))
			std::cout << "x:" << exp->MyStage->AxisPos_x[0] << ",y:" << exp->MyStage->AxisPos_y[0] << std::endl;
		Sleep(2000);
		saveastiff(exp->fromCCD->iplimg);
	}
}

/*
 *
 * Handle KeyStroke
 *
 * Returns 1 when the user is trying to exit
 *
 */
int HandleKeyStroke(int c, Experiment* exp) {
	//采用Lambda表示函数，内置函数
	auto func = [](Experiment* exp) -> void {
		if (FindStagePosition(*(exp->MyStage)))
			std::cout << "[x] " << exp->MyStage->AxisPos_x[0] << " : [y] " << exp->MyStage->AxisPos_y[0] << std::endl;
		std::cout << std::endl;
		exp->Neuron->stagePosition_x = exp->MyStage->AxisPos_x[0];
		exp->Neuron->stagePosition_y = exp->MyStage->AxisPos_y[0];
		exp->MyCamera->resetMaxAndMin = TRUE;
	};

	switch (c) {
	case 27:
		std::cout << "************* Turn Off *************" << std::endl;
		printf("User has pressed escape!\n");
		printf("Setting Stage Tracking Variables to off");
		exp->Params->stageTrackingOn = 0;
		exp->stageIsTurningOff = 1;
		return 1;
	case 's':
		saveastiff(exp->fromCCD->iplimg);
		break;
	case '4':
		std::cout << "Left!  #StagePosition : ";
		MoveRelative(exp->MyStage->handle, (float)(0.1), AXIS_X);
		func(exp);
		break;
	case '5': //停下位移台
		HaltStage(*(exp->MyStage));
		break;
	case '8':
		std::cout << "Up!  #StagePosition : ";
		MoveRelative(exp->MyStage->handle, (float)(0 - 0.1), AXIS_Y);
		func(exp);
		break;
	case '6':
		std::cout << "Right!  #StagePosition : ";
		MoveRelative(exp->MyStage->handle, (float)(0 - 0.1), AXIS_X);
		func(exp);
		break;
	case '2':
		std::cout << "Down!  #StagePosition : ";
		MoveRelative(exp->MyStage->handle, (float)(0.1), AXIS_Y);
		func(exp);
		break;
	case 'r':  //开始进行视频记录
		std::cout << std::endl;
		std::cout << "************* Video Writer *************" << std::endl;
		std::cout << "    Start Recording..." << std::endl;
		exp->e = SetupRecording(exp);
		exp->Params->Record = 1;
		std::cout << "****************************************" << std::endl;
		break;
	case '\t': //开始跟踪按钮
		Toggle(&(exp->Params->stageTrackingOn));
		if (exp->Params->stageTrackingOn == 0) {
			/** If we are turning the stage off, let the rest of the code know **/
			printf("[StageModule] Turning tracking off!\n");
			exp->stageIsTurningOff = 1;
		}
		else {
			printf("[StageModule] Turning tracking on!\n");
			spinStage(*exp->MyStage, exp->Params->maxstagespeed, exp->Params->maxstagespeed);
		}
		break;
	case 'm':
		exp->MyCamera->resetMaxAndMin = TRUE;
		break;
	default:
		break;
	}
	return 0;
}

/*
 * Scan for the USB device.
 */
void InvokeStage(Experiment* exp) {
	exp->stageCenter = cvPoint(NSIZEX / 2, NSIZEY / 2);

	if (!AllocateStageDataStructMemory(&(exp->MyStage))) {
		std::cout << "Cant Init Stage!" << std::endl;
		exp->e = EXP_ERROR;
		return;
	}

	auto stage = exp->MyStage;

	if (stage == nullptr) {
		printf("ERROR! Invoking the stage failed.\nTurning tracking off.\n");
		exp->Params->stageTrackingOn = 0;
	}
	else {
		printf("Invoking the stage succeed.\nTelling stage to HALT.\n");
		HaltStage(*(stage));
	}

	if (!Initialize_StageCOMAndAxis(*(stage)))
	{
		std::cout << "Cant Init Stage!" << std::endl;
		exp->e = EXP_ERROR;
		return;
	}

	auto x = exp->x_initPos;
	auto y = exp->y_initPos;

	MoveToCenter(*(stage), x, y);   //需要通过手动校准程序找到坐标

	//初始化之后立马根据控制面板设定当前速度
	spinStage(*exp->MyStage, exp->Params->maxstagespeed, exp->Params->maxstagespeed);

	return;
}

void ShutOffStage(Experiment* exp) {
	HaltStage(*(exp->MyStage));
}

/*
 *This is the frame rate timer.
 */
void StartFrameRateTimer(Experiment* exp) {
	exp->prevTime = clock();
	//std::cout << "prevTime: " << exp->prevTime << std::endl;
	exp->prevFrames = 0;
}

/*
 * If more than a second has elapsed
 * Calculate the frame rate and print i tout
 *
 */
void CalculateAndPrintFrameRate(Experiment* exp) {
	/*** Print out Frame Rate ***/
	if ((exp->Neuron->timestamp - exp->prevTime) > CLOCKS_PER_SEC) {
		/*printf("%d fps   #", exp->Neuron->frameNum - exp->prevFrames);*/
		exp->FramePerSec = exp->Neuron->frameNum - exp->prevFrames;

		/*std::cout << "CenterOfWorm >> [x] " << exp->Neuron->Segmented->centerOfWorm->x << ": [y] " << exp->Neuron->Segmented->centerOfWorm->y << std::endl;
		std::cout << std::endl;*/
		exp->prevFrames = exp->Neuron->frameNum;
		exp->prevTime = exp->Neuron->timestamp;
	}
}

/*
 * Given an image in teh Neuron object, segment the Neuron
 *
 */
CvPoint calculateCentroid(CvSeq* Boundary) {
	// 初始化重心坐标
	double sumX = 0.0;
	double sumY = 0.0;

	// 获取轮廓点的个数
	int totalPoints = Boundary->total;

	// 遍历每个点并累加
	for (int i = 0; i < totalPoints; i++) {
		// 获取第 i 个点
		CvPoint* pt = (CvPoint*)cvGetSeqElem(Boundary, i);

		// 累加 x 和 y 坐标
		sumX += pt->x;
		sumY += pt->y;
	}

	// 计算平均值（即重心）
	CvPoint centroid;
	centroid.x = cvRound(sumX / totalPoints);  // 使用四舍五入取整
	centroid.y = cvRound(sumY / totalPoints);  // 使用四舍五入取整

	return centroid;
}

//找到线虫的重心
int findWormCentroid(NeuronAnalysisData* Worm, NeuronAnalysisParam* Params)
{
	if (cvSeqExists(Worm->Boundary) == 0) {
		printf("Error! No boundary found in SegmentWorm()\n");
		return -1;
	}

	ClearSegmentedInfo(Worm->Segmented);

	if (Worm->Segmented->PtOnWorm == NULL)
		Worm->Segmented->PtOnWorm = (CvPoint*)malloc(sizeof(CvPoint));

	CvPoint target = calculateCentroid(Worm->Boundary);
	Worm->Segmented->PtOnWorm->x = target.x;
	Worm->Segmented->PtOnWorm->y = target.y;

	return 0;
}

void DoSegmentation(Experiment* exp) {
	/*** <segmentNeuron> ***/

	/*** Find Worm Boundary ***/
	/*
	 *  There is a lot in this one function, FindWormBoudnary(), including:
	 *  Gaussian Blurring
	 *  Thresholding
	 *  Blob Detection
	 *  etc
	 */

	if (!(exp->e))
		FindWormBoundary(exp->Neuron, exp->Params);

	if (!(exp->e))
		exp->e = findWormCentroid(exp->Neuron, exp->Params);

#if 0

	/*** Find Worm Head and Tail ***/
	if (!(exp->e))
		exp->e = GivenBoundaryFindWormHeadTail(exp->Neuron, exp->Params);

	/** If we are doing temporal analysis, improve the WormHeadTail estimate based on prev frame 这一段改进是关闭的 **/
	if (exp->Params->TemporalOn && !(exp->e)) {
		PrevFrameImproveWormHeadTail(exp->Neuron, exp->Params, exp->PrevWorm);
	}

	/** if the user is manually inducing a head/tail flip **/
	if (exp->Params->InduceHeadTailFlip) {
		ReverseWormHeadTail(exp->Neuron);
		/** Turn the flag off **/
		exp->Params->InduceHeadTailFlip = 0;
		/*
		 * Note, of course, this function only makes sense if the user is also doing temporal intelligence.
		 * (Otherwise the flipped head tail would immediately reverse itself in the next frame.)
		 * But for completeness we allow the use to do the flip here.
		 */
	}

	/*** Segment the Worm ***/
	if (!(exp->e))
		exp->e = SegmentWorm(exp->Neuron, exp->Params);

	/** Update PrevWorm Info **/
	if (!(exp->e))
		LoadWormGeom(exp->PrevWorm, exp->Neuron);

#endif
	/*** </segmentworm> ***/
//_TICTOC_TOC_FUNC
}

//void DoSegmentation(Experiment* exp) {
//	/*** <segmentNeuron> ***/
//	
//	/*** Find Worm Boundary ***/
//	/*
//	 *  There is a lot in this one function, FindWormBoudnary(), including:
//	 *  Gaussian Blurring
//	 *  Thresholding
//	 *  Blob Detection
//	 *  etc
//	 */
//
//	if (!(exp->e))
//		FindWormBoundary(exp->Neuron, exp->Params);
//
//	/*** Find Worm Head and Tail ***/
//	if (!(exp->e))
//		exp->e = GivenBoundaryFindWormHeadTail(exp->Neuron, exp->Params);
//
//	/** If we are doing temporal analysis, improve the WormHeadTail estimate based on prev frame 这一段改进是关闭的 **/
//	if (exp->Params->TemporalOn && !(exp->e)) {
//		PrevFrameImproveWormHeadTail(exp->Neuron, exp->Params, exp->PrevWorm);
//	}
//
//	/** if the user is manually inducing a head/tail flip **/
//	if (exp->Params->InduceHeadTailFlip) {
//		ReverseWormHeadTail(exp->Neuron);
//		/** Turn the flag off **/
//		exp->Params->InduceHeadTailFlip = 0;
//		/*
//		 * Note, of course, this function only makes sense if the user is also doing temporal intelligence.
//		 * (Otherwise the flipped head tail would immediately reverse itself in the next frame.)
//		 * But for completeness we allow the use to do the flip here.
//		 */
//	}
//
//	/*** Segment the Worm ***/
//	if (!(exp->e))
//		exp->e = SegmentWorm(exp->Neuron, exp->Params);
//#if 0
//	/** Update PrevWorm Info **/
//	if (!(exp->e))
//		LoadWormGeom(exp->PrevWorm, exp->Neuron);
//
//#endif
//	/*** </segmentworm> ***/
////_TICTOC_TOC_FUNC
//
//	CvPoint* PtOnWorm = (CvPoint*)cvGetSeqElem(exp->Neuron->Segmented->Centerline, exp->Params->stageTargetSegment);
//	exp->pos_accumulate.x += PtOnWorm->x;
//	exp->pos_accumulate.y += PtOnWorm->y;
//
//	exp->pos_num += 1;
//}

void ShowContoursGrayImg(Experiment* exp)
{
	IplImage* ContoursGray = cvCreateImage(cvGetSize(exp->Neuron->ImgSmooth), exp->Neuron->ImgSmooth->depth, exp->Neuron->ImgSmooth->nChannels);
	cvZero(ContoursGray);

	CvSeq* PointTemp = exp->Neuron->Boundary;

	//显示最大线虫的轮廓
	for (; PointTemp != NULL; PointTemp = PointTemp->h_next)
	{
		for (int i = 0; i < PointTemp->total; i++)
		{
			CvPoint* point = (CvPoint*)cvGetSeqElem(PointTemp, i);
			cvSetReal2D(ContoursGray, point->y, point->x, 255.0);
		}
		CvSeq* vnext = PointTemp->v_next;
		for (; vnext != NULL; vnext = vnext->h_next)
		{
			for (int k = 0; k < vnext->total; k++)
			{
				CvPoint* point = (CvPoint*)cvGetSeqElem(vnext, k);
				cvSetReal2D(ContoursGray, point->y, point->x, 255.0);
			}
		}
	}

	cvShowImage(exp->WinDisp, ContoursGray);
	cvReleaseImage(&ContoursGray);
}

/*********************** RECORDING *******************/

/*
 * Sets up data recording and video recording
 * Will record video if exp->RECORDVID is 1
 * and record data if exp->RECORDDATA is 1
 *
 */
int SetupRecording(Experiment* exp) {
	int codec = CV_FOURCC('M', 'J', 'P', 'G');
	double fps = 25.0;

	printf("About to setup recording\n");
	//char* DataFileName = NULL;

	/** Setup Writing and Write Out Comments **/
	exp->DataWriter = SetUpWriteToDisk(exp->dirname, exp->outfname, exp->Neuron->MemStorage);

	/** We should Quit Now if any of the data Writing is not working **/
	if (exp->DataWriter->error < 0) return -1;

	BeginToWriteOutFrames(exp->DataWriter);

	printf("Initialized data recording\n");
	//DestroyFilename(&DataFileName);

	/** Set Up Video Recording **/
	char* MovieFileName;

	MovieFileName = CreateFileName(exp->dirname, exp->outfname, ".avi");

	//exp->Vid = cvCreateVideoWriter(MovieFileName,
	//	CV_FOURCC('M', 'J', 'P', 'G'), 25, cvSize(NSIZEX / 2, NSIZEY / 2),  //'M', 'J', 'P', 'G'
	//	0);    //修改了记录帧数，将帧数设定为25帧 （24/7/16) 
	exp->vid_writer->open(MovieFileName, CV_FOURCC('M', 'J', 'P', 'G'), 25, cvSize(NSIZEX / 2, NSIZEY / 2), 0);  //2025-12-29 by ysh

	if (!exp->vid_writer->isOpened())
	{
		printf("\tERROR in SetupRecording! exp->Vid is NULL\nYou probably are missing the default codec.\n");
		return -1;
	}

	DestroyFilename(&MovieFileName);
	printf("Initialized video recording\n");

	return 0;  
}

/*
 * Finish writing video and  and data
 * and release
 *
 */
void FinishRecording(Experiment* exp) {
	/** Finish Writing Video to File and Release Writer **/
	//if (exp->Vid != NULL)
	//	cvReleaseVideoWriter(&(exp->Vid));
	/** Finish Writing to Disk **/

	if (exp->Params->Record == 1) {
		FinishWriteToDisk(&(exp->DataWriter));
		std::cout << "Finished Record" << std::endl;
	}
	exp->Params->Record = 0;
}

/*
 * Write video and data to Disk
 *
 */
void DoWriteToDisk(Experiment* exp, int distance) {
	//在后台记录视频中进行图像信息标记
	//AddInfoText_Disk(exp, exp->fromCCD->iplimg, distance);

	/** 记录视频数据 **/
	if (exp->Params->Record) {
		cvResize(exp->fromCCD->iplimg, exp->SubSampled, CV_INTER_LINEAR);

		//cvWriteFrame(exp->Vid, exp->SubSampled);
		cv::Mat mat = cv::cvarrToMat(exp->SubSampled);
		exp->vid_writer->write(mat);
		//if (exp->Vid == NULL) printf("\tERROR in DoWriteToDisk!\n\texp->Vid is NULL\n");
	}

	/** 记录yaml数据 **/
	if (exp->Params->Record) {
		AppendNeuronFrameToDisk(exp->Neuron, exp->Params, exp->DataWriter);
	}
}
/********************************Tracking***************************************/
/***********************
 * PID Controler
 ***********************/
float pidControl(float error, float previous_error, float& integral, float Kp, float Ki, float Kd, float dt) {
	integral += error * dt;
	float derivative = (error - previous_error) / dt;
	float output = Kp * error + Ki * integral + Kd * derivative;
	return output;
}
/**************************************************
 * Stage Tracking and FEedback System
 *
 * This should really probably go in a special library called Stage Tracking
 * that depends on both OpenCV AND Talk2STage.c, but its a huge pain to modify the makefile
 * to create a new library that has only one function in it.
 *
 * Alternatively this could conceivably go in Talk2Stage.c, but then I find it weird
 * that Talk2Stage.c should depend on OpenCV, because ultimatley it should be more general.
 *
 * It doesn't really belong in experiment.c either because it is not a method of experiment.c
 * But for now that is where it will sit.
 *
 */
int AdjustStageToKeepObjectAtTarget(StageData* stage, CvPoint* obj, CvPoint target, int speedfactor, double maxspeed, int AxisLockState)
{

	if (obj == NULL) {
		//printf("Error! obj is NULL in AdjustStageToKeepObjectAtTarget()\n");
		return 1;
	}

#if 0
//查询位移台当前运动状态，如果位移台处于移动状态，则限制下一次移动
//SLS_DLL_API bool GetStation(int m_handle, int m_Axis, bool m_GetStation[1]);返回true表示获取成功，false表示获取失败
//获取运动状态 m_GetStation[0]=true 表示在运动中 m_GetStation[0]=false表示运动完成 

	bool m_GetStation_x[1] = { true };
	bool m_GetStation_y[1] = { true };
	// 只要X轴或Y轴中任意一个在移动，就继续循环
	while (m_GetStation_x[0] || m_GetStation_y[0]) {
		// 获取X轴状态
		if (!GetStation(stage->handle, AXIS_X, m_GetStation_x)) {
			std::cout << "[Warning...] Can't get x Station during wait loop" << std::endl;
			return -1;
		}
		// 获取Y轴状态
		if (!GetStation(stage->handle, AXIS_Y, m_GetStation_y)) {
			std::cout << "[Warning...] Can't get y Station during wait loop" << std::endl;
			return -1;
		}

		// 如果仍在移动，则进行短暂睡眠，避免CPU空转
		if (m_GetStation_x[0] || m_GetStation_y[0]) {
			// 可以打印状态用于调试
			// std::cout << "Waiting for stage to stop... X_moving: " << m_GetStation_x[0] << ", Y_moving: " << m_GetStation_y[0] << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 等待10ms再查，避免过于频繁
		}
	}
	// --- 等待逻辑结束，此时位移台已静止 ---
#endif

	// 位移台静止后，开始计算下一次移动
	CvPoint diff;	//单位：像素
	float vel[2];	//单位mm/s
	float dt = 0.08;

	/** (stage-obj)*speed **/

	diff.y = obj->y - target.y;
	diff.x = target.x - obj->x;

	std::cout << "diff.x:" << diff.x << " , diff.y:" << diff.y << std::endl;

	// Error calculation for PID
	float resolution = static_cast<float>(0.1 / 108);
	float error_x = diff.x * resolution;	//单位mm
	float error_y = diff.y * resolution;

	//前馈速度，作为PID调节前的基础
	float v_ff_x = error_x / (stage->target_time / 1000.0f);	//单位mm/s
	float v_ff_y = error_y / (stage->target_time / 1000.0f);

	// Calculate velocity using PID control for both axes
	float pid_velocity_x = pidControl(error_x, stage->previous_error_x, stage->integral_x, stage->Kp_x, stage->Ki_x, stage->Kd_x, dt);
	float pid_velocity_y = pidControl(error_y, stage->previous_error_y, stage->integral_y, stage->Kp_y, stage->Ki_y, stage->Kd_y, dt);

	// Apply PID velocities to the stage
	// 使用 std::min 和 std::max 限制vel不超过设定值
	vel[0] = std::max(static_cast<float>(-maxspeed), std::min(static_cast<float>(0.01 * v_ff_x + pid_velocity_x), static_cast<float>(maxspeed)));
	vel[1] = std::max(static_cast<float>(-maxspeed), std::min(static_cast<float>(0.01 * v_ff_y + pid_velocity_y), static_cast<float>(maxspeed)));
	// vel[0] = std::max(static_cast<float>(-maxspeed), std::min(static_cast<float>(0.8 * v_ff_x + pid_velocity_x), static_cast<float>(maxspeed)));
	// vel[1] = std::max(static_cast<float>(-maxspeed), std::min(static_cast<float>(0.8 * v_ff_y + pid_velocity_y), static_cast<float>(maxspeed)));

	// alpha是平滑系数，越小越平滑，但响应越慢。可以从0.6-0.8开始尝试
	float alpha = 0.6f;
	stage->smoothed_vel_x = alpha * vel[0] + (1.0f - alpha) * stage->smoothed_vel_x;
	stage->smoothed_vel_y = alpha * vel[1] + (1.0f - alpha) * stage->smoothed_vel_y;

	// Update previous error for PID
	stage->previous_error_x = error_x;
	stage->previous_error_y = error_y;

	//vel[0] = (float)abs(0.1 * maxspeed * tanh(0.001 * diff.x * speedfactor));
	//vel[1] = (float)abs(0.1 * maxspeed * tanh(0.001 * diff.y * speedfactor));

	//现在根据diff设定两个阈值，实现更多样的功能
	// Condition 1: If both diff.x and diff.y are less than 30, do not move the stage
	if (abs(diff.x) < 30 && abs(diff.y) < 30) {
		stage->smoothed_vel_x = 0;
		stage->smoothed_vel_y = 0;
		spinStage(*stage, 0, 0);
		std::cout << "Both diff.x and diff.y are less than 30. No movement." << std::endl;
		return 0;
	}

	// Condition 2: If one axis diff is greater than 30 but less than 90, and the other axis diff is less than 90, move the axis with the larger absolute value
	if ((abs(diff.x) > 30 && abs(diff.x) < 90 && abs(diff.y) < 90) || (abs(diff.y) > 30 && abs(diff.y) < 90 && abs(diff.x) < 90)) {
		if (abs(diff.x) > abs(diff.y)) {
			// Move only the X-axis
			stage->smoothed_vel_y = 0;
			std::cout << "Only moving X-axis. diff.x is larger." << std::endl;
		}
		else {
			// Move only the Y-axis
			stage->smoothed_vel_x = 0;
			std::cout << "Only moving Y-axis. diff.y is larger." << std::endl;
		}
	}
	else {
		// Condition 3: If both diff.x and diff.y are greater than 90, move both axes
		std::cout << "Moving both axes. One of the diff.x and diff.y is greater than 90." << std::endl;
	}

	//设定加速度，目前设定为当前设定速度的2倍
	AcDecSet(stage->handle, 8.0f, AXIS_X);
	AcDecSet(stage->handle, 8.0f, AXIS_Y);

	//std::cout << "vel x:" << vel[0] << " , vel y:" << vel[1] << std::endl;

	//将速度传给位移台
	spinStage(*stage, stage->smoothed_vel_x, stage->smoothed_vel_y);

	// 计算在这一个时间步长(dt)内，位移台应该移动的微小距离(mm)
	float displacement_x = stage->smoothed_vel_x * dt / resolution;
	float displacement_y = stage->smoothed_vel_y * dt / resolution;

	switch (AxisLockState) {
	case 0: // 自由移动
		MoveToTarget(*stage, displacement_x, displacement_y);
		break;

	case 1:
		MoveYToTarget(*stage, displacement_y);
		break;

	case 2: // 锁定 Y 轴, 只在 X 方向移动
		MoveXToTarget(*stage, displacement_x);
		break;

	default: 

		break;
	}

	//printf("SpinStage: vel.x=%lf, vel.y=%lf\n", vel[0], vel[1]);
	std::cout << "MoveToTarget had done." << std::endl;

	return 0;
}

/*
 * Update the Stage Tracker.
 * If the Stage tracker is not initialized, don't do anything.
 * If the stage tracker is initialized then either do the tracking,
 * or if we are in the process of turning off tracking off, then tell
 * the stage to halt and update flags.
 */
int HandleStageTracker(Experiment* exp) {
	if (exp->stageIsPresent == 1) { /** If the Stage is Present **/
		if (exp->MyStage == NULL)
			return 0;

		if (exp->Params->stageTrackingOn == 1) {
			if (exp->Params->OnOff == 0) { /** if the analysis system is off **/
				/** Turn the stage off **/
				exp->stageIsTurningOff = 1;
				exp->Params->stageTrackingOn = 0;
			}
			else {
				/** Move the stage to keep the Neuron centered in the field of view **/
				//printf(".");

				/** Get the Point on the worm some distance along the centerline **/
				//CvPoint* PtOnWorm = (CvPoint*)cvGetSeqElem(exp->Neuron->Segmented->Centerline, exp->Params->stageTargetSegment); 
				CvPoint* PtOnWorm = exp->Neuron->Segmented->PtOnWorm;
				//std::cout << "PtOnWorm:x-" << PtOnWorm->x << ", y-" << PtOnWorm->y << std::endl;
				//printf("stageFeedbackTargetoffset=(%d, %d)\n",exp->stageFeedbackTargetOffset.x,exp->stageFeedbackTargetOffset.y);
				CvPoint target = cvPoint(
					exp->stageCenter.x + exp->stageFeedbackTargetOffset.x,
					exp->stageCenter.y + exp->stageFeedbackTargetOffset.y);
				//std::cout << "target:x-" << target.x << ", y-" << target.y << std::endl;
				int ret = AdjustStageToKeepObjectAtTarget(
					exp->MyStage, PtOnWorm, target,
					exp->Params->stageSpeedFactor, exp->Params->maxstagespeed, exp->Params->AxisLockState);

					//检查返回值
				if (ret != 0) {
					std::cout << "[ERROR] AdjustStage failed. Halting tracking." << std::endl;
					//如果底层函数失败，立即中止自己，并将错误码传递上去
					return ret;
				}

				//保存位移台之前的坐标
				exp->MyStage->prePos_x = exp->MyStage->AxisPos_x[0];
				exp->MyStage->prePos_y = exp->MyStage->AxisPos_y[0];

				if (FindStagePosition(*(exp->MyStage))) {
					exp->Neuron->stagePosition_x = exp->MyStage->AxisPos_x[0];
					exp->Neuron->stagePosition_y = exp->MyStage->AxisPos_y[0];
				}
			}
		}
		if (exp->Params->stageTrackingOn == 0) {/** Tracking Should be off **/
			/** If we are in the process of turning tacking off **/
			if (exp->stageIsTurningOff == 1) {
				/** Tell the stage to Halt **/
				printf("[TrackingThread] Tracking Stopped!");
				printf("[TrackingThread] Telling stage to HALT.\n");
				HaltStage(*(exp->MyStage));
				exp->stageIsTurningOff = 0;
			}
			/** The stage is already halted, so there is nothing to do. **/
		}

	}
	return 0;
}
//int HandleStageTracker(Experiment* exp) {
//	if (exp->stageIsPresent == 1) { /** If the Stage is Present **/
//		if (exp->MyStage == NULL)
//			return 0;
//
//		if (exp->Params->stageTrackingOn == 1) {
//			if (exp->Params->OnOff == 0) { /** if the analysis system is off **/
//				/** Turn the stage off **/
//				exp->stageIsTurningOff = 1;
//				exp->Params->stageTrackingOn = 0;
//			}
//			else {
//				/** Move the stage to keep the Neuron centered in the field of view **/
//				//printf(".");
//
//				/** Get the Point on the worm some distance along the centerline **/
//				//CvPoint* PtOnWorm = (CvPoint*)cvGetSeqElem(exp->Neuron->Segmented->Centerline, exp->Params->stageTargetSegment);
//				CvPoint* PtOnWorm = new CvPoint(exp->pos_accumulate.x/exp->pos_num, exp->pos_accumulate.y/exp->pos_num);
//
//				//printf("stageFeedbackTargetoffset=(%d, %d)\n",exp->stageFeedbackTargetOffset.x,exp->stageFeedbackTargetOffset.y);
//				CvPoint target = cvPoint(
//					exp->stageCenter.x + exp->stageFeedbackTargetOffset.x,
//					exp->stageCenter.y + exp->stageFeedbackTargetOffset.y);
//				//printf("target=(%d, %d)\n",target.x,target.y);
//				int ret = AdjustStageToKeepObjectAtTarget(
//					exp->MyStage, PtOnWorm, target,
//					exp->Params->stageSpeedFactor, exp->Params->maxstagespeed, exp->Params->YMoveLock);
//
//				//保存位移台之前的坐标
//				exp->MyStage->prePos_x = exp->MyStage->AxisPos_x[0];
//				exp->MyStage->prePos_y = exp->MyStage->AxisPos_y[0];
//
//				if (FindStagePosition(*(exp->MyStage))) {
//					exp->Neuron->stagePosition_x = exp->MyStage->AxisPos_x[0];
//					exp->Neuron->stagePosition_y = exp->MyStage->AxisPos_y[0];
//				}
//
//				exp->pos_accumulate = cvPoint(0, 0);
//				exp->pos_num = 0;
//				delete PtOnWorm;
//				PtOnWorm = nullptr;
//			}
//		}
//		if (exp->Params->stageTrackingOn == 0) {/** Tracking Should be off **/
//			/** If we are in the process of turning tacking off **/
//			if (exp->stageIsTurningOff == 1) {
//				/** Tell the stage to Halt **/
//				printf("[TrackingThread] Tracking Stopped!");
//				printf("[TrackingThread] Telling stage to HALT.\n");
//				HaltStage(*(exp->MyStage));
//				exp->stageIsTurningOff = 0;
//			}
//			/** The stage is already halted, so there is nothing to do. **/
//		}
//
//	}
//	return 0;
//}

/*
 * Update the Stage Tracker Position.
 * If the Stage tracker is not initialized, don't do anything.
*/
int RecordStageTracker(Experiment* exp) {

	if (FindStagePosition(*(exp->MyStage))) {
		exp->Neuron->stagePosition_x = exp->MyStage->AxisPos_x[0];
		exp->Neuron->stagePosition_y = exp->MyStage->AxisPos_y[0];
	}

	return 0;
}

//在图片上添加LED以及帧率显示信息
void AddInfoText_Exp(Experiment* exp, IplImage* image)
{
	//Led Info
	CvPoint led_org = cvPoint(10, 30);
	CvFont led_font = cvFont(2.5, 3);
	char led_text[100] = "Led- ";
	if (exp->Params->Led0_status)
	{
		strcat(led_text, "On :");
		std::string str = std::to_string(exp->Params->BrightnessPerc);
		strcat(led_text, str.c_str());
		strcat(led_text, "%");
	}
	else strcat(led_text, "Off");

	//Ef Info
	CvPoint EF_org = cvPoint(10, 70);
	CvFont EF_font = cvFont(2.5, 3);
	char EF_text[100] = "EF- ";
	if(exp->Params->EF_Mode == 0) strcat(EF_text, "0");
	else if (exp->Params->EF_Mode == 1) strcat(EF_text, "<---");
	else if (exp->Params->EF_Mode == 2) strcat(EF_text, "--->");

	//Frame Info
	CvPoint Frame_org = cvPoint(600, 30);
	CvFont Frame_font = cvFont(2.5, 3);
	char f_text[100] = {};
	std::string fstr = std::to_string(exp->Neuron->frameNum);

	CvScalar scalar = cvScalar(255);

	cvPutText(image, led_text, led_org, &led_font, scalar);
	cvPutText(image, EF_text, EF_org, &led_font, scalar);
	cvPutText(image, fstr.c_str(), Frame_org, &Frame_font, scalar);
}

void AddInfoText_Disk(Experiment* exp, IplImage* image, int distance)
{
	//Led Info
	CvPoint led_org = cvPoint(25, 75);
	CvFont led_font = cvFont(5, 7);
	char distance_text[100] = "Dist:";

	//将LED亮度调整为距离
	std::string str = std::to_string(distance);
	strcat(distance_text, str.c_str());

	//Ef Info
	CvPoint EF_org = cvPoint(25, 160);
	CvFont EF_font = cvFont(2.5, 3);
	char EF_text[100] = "EF- ";
	if (exp->Params->EF_Mode == 0) strcat(EF_text, "0");
	else if (exp->Params->EF_Mode == 1) strcat(EF_text, "<---");
	else if (exp->Params->EF_Mode == 2) strcat(EF_text, "--->");

	//Frame Info
	CvPoint Frame_org = cvPoint(1400, 75);
	CvFont Frame_font = cvFont(4, 6);
	char f_text[100] = {};
	std::string fstr = std::to_string(exp->Neuron->frameNum);

	CvScalar scalar = cvScalar(255);

	cvPutText(image, distance_text, led_org, &led_font, scalar);
	cvPutText(image, EF_text, EF_org, &led_font, scalar);
	cvPutText(image, fstr.c_str(), Frame_org, &Frame_font, scalar);
}

//负责显示线虫的图像
void ElegansImShow(Experiment* exp) {
	switch (exp->Params->Display) {
		case 0:
		{
			//显示增强后的原图像
			IplImage* scaledImage = cvCreateImage(cvSize(800, 600), IPL_DEPTH_8U, 1);
			cvResize(exp->fromCCD->iplimg, scaledImage, CV_INTER_LINEAR);
			cvSmooth(scaledImage, scaledImage, CV_GAUSSIAN, 5, 5, 0, 0);

			//在实验图像上进行添加当前实验数据信息
			AddInfoText_Exp(exp, scaledImage);

			//显示图像并释放分配的内存
			cvShowImage(exp->WinDisp, scaledImage);
			cvReleaseImage(&scaledImage);
			break;
		}
		case 1:
		{
			cvShowImage(exp->WinDisp, exp->Neuron->ImgThresh);
			break;
		}
		case 2:
		{
			ShowContoursGrayImg(exp);
			break;
		}
	}
}
