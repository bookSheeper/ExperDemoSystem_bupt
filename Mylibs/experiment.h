/*
 * experiment.h
 *
 *  Created on: Sep , 2023
 *      Author: Shuhao Yang
 */

#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

#include "Talk2WStage.h"
#include "WriteOutNeuron.h"

class UDPSender {
private:
	std::atomic<bool> run_thread;
	std::atomic<bool> run_square_wave;   //控制方波信号的运行
	std::atomic<char> volChannel;        //输出通道号
	std::atomic<char> volDir;            //输出电压方向 0-正向 1-负向 默认0
	std::atomic<char> message;           //输出电压幅值
	std::atomic<char> volWave;             //电压波形
	std::thread udp_thread;
	std::thread square_wave_thread;  // 保存方波线程的引用
	SOCKET sockfd;
	struct sockaddr_in servaddr;

	void send_udp();

public:
	UDPSender();
	~UDPSender();
	void start();
	void stop();
	void changeMessage(char newChannel, char newDir, char newMessage, char newWave);
	void SquareWave(char Amplitude, int Frequency, int DutyCycle);
	void stopSquareWave(); // 新增停止方波信号的函数
	WaveType curType = WaveType::None;
};

typedef struct ExperimentStruct{

	/** GuiWindowNames **/
	const char* WinDisp;
	const char* WinCon1;

	/** CommandLine Input **/
	char** argv;
	int argc;

	/** Camera Input **/
	CamData* MyCamera;
	int FramePerSec;

	/** MostRecently Observed CameraFrameNumber **/
	uint64_t lastFrameSeenOutside;

	/** User-configurable Neuron-related Parameters **/
	NeuronAnalysisParam* Params;

	/** Information about Our Neuron **/
	NeuronAnalysisData* Neuron;

	/** Information about the Previous frame's Worm **/
	WormGeom* PrevWorm;

	/** internal IplImage **/
	IplImage* CurrentSelectedImg;
	IplImage* SubSampled; // Image used to subsample stuff
	IplImage* HUDS;  //Image used to generate the Heads Up Display

	/** Internal Frame data types **/
	Frame* fromCCD;

	/** Write Data To File **/
	char* dirname;
	char* outfname;
	char* infname;
	WriteOut* DataWriter;

	/** Write Video To File **/
	//CvVideoWriter* Vid;  //Video Writer
	cv::VideoWriter* vid_writer;  //采用opencv340的videowriter记录视频
	double standardCorX;   //相对于坐标点的标准坐标
	double standardCorY;
	int Voltage;  //记录电场的电压
	double cur_velocity;   //线虫的速度  单位um/s
	

	/** Timing  Information **/
	clock_t now;
	clock_t last;


	/** Frame Rate Information **/
	int nframes;
	int prevFrames;
	long prevTime;

	/** Stage Control **/
	StageData* MyStage;
	int stageIsPresent;
	int stage; // Handle to USB stage object    我们的Stage handle是一个int整型
	CvPoint stageVel; //Current velocity of stage
	CvPoint stageCenter; // Point indicating center of stage.
	CvPoint stageFeedbackTargetOffset; //Target of the stage feedback loop as a delta distance in pixels from the center of the image
	int stageIsTurningOff; //1 indicates stage is turning off. 0 indicates stage is on or off.
	float x_initPos;
	float y_initPos;
	std::queue<double>* NemDisp;   //代表线虫的位移

	/** TLDC LED input **/
	LedData* MyLed;

	/** Error Handling **/
	int e;

	//UDP 模块
	UDPSender* sender;

	//位移累加
	CvPoint pos_accumulate;
	int pos_num;

} Experiment;

/*
 * Creates a new experiment object and sets values to zero.
 */
Experiment* CreateExperimentStruct();


/*
 * Load the command line arguments into the experiment object
 */
void LoadCommandLineArguments(Experiment* exp, int argc, char** argv);

void displayhelp();

/*
 * Handle CommandLine Arguments
 * Parses commandline arguments.
 * Decides if user wants to record video or recorddata
 */
int HandleCommandLineArguments(Experiment* exp);



/* Assigns Default window names to the experiment object
 *
 */
void AssignWindowNames(Experiment* exp);

/*
 * Release the memopry associated with window names
 * and set their pointers to Null
 */
void ReleaseWindowNames(Experiment* exp);


/*
 * SetupGui
 *
 */
void SetupGUI(Experiment* exp);

/*
 * Update's trackbar positions for variables that can be changed by the software
 *
 */
void UpdateGUI(Experiment* exp);


/*
 * Initialize camera library
 * Allocate Camera Data
 * Select Camera and Show Properties dialog box
 * Start Grabbing Frames as quickly as possible
 *
 * OR open up the video file for reading.
 */

void RollCameraInput(Experiment* exp);

/* Init TLDC LED */
void RollLedInput(Experiment* exp);

/** Grab a Frame from either camera or video source
 *
 */
int GrabFrame(Experiment* exp);



/*
 * This function allocates images and frames
 * And a Neuron Object
 *
 * And a Parameter Object
 * For internal manipulation
 *
 *
 */
void InitializeExperiment(Experiment* exp);


/*
 * Free up all of the different allocated memory for the
 * experiment.
 *
 */
void ReleaseExperiment(Experiment* exp);

void DestroyExperiment(Experiment** exp);

/*
 * Setsup data recording and video recording
 * Will record video if exp->RECORDVID is 1
 * and record data if exp->RECORDDATA is 1
 *
 */
int SetupRecording(Experiment* exp);


/*
 * Finish writing video and  and data
 * and release
 *
 */
void FinishRecording(Experiment* exp);

/*********************************************
 *
 * Image Acquisition
 *
 */

/*
 * Is a frame ready from the camera?
 */
int isFrameReady(Experiment* exp);


/************************************************/
/*   Frame Rate Routines
 *
 */
/************************************************/

/*
 *This is the frame rate timer.
 */
void StartFrameRateTimer(Experiment* exp);

/*
 * If more than a second has elapsed
 * Calculate the frame rate and print it out
 *
 */
void CalculateAndPrintFrameRate(Experiment* exp);

/************************************************/
/*   Action Chunks
 *
 */
/************************************************/

/*
 * Given an image in teh Neuron object, segment the Neuron
 *
 */

void DoSegmentation(Experiment* exp);

void PrepareSelectedDisplay(Experiment* exp);


/*
 *
 * Handle KeyStroke
 *
 * Returns 1 when the user is trying to exit
 *
 */
int HandleKeyStroke(int c, Experiment* exp);

/*
 * Write video and data to Disk
 *
 */
void DoWriteToDisk(Experiment* exp, int distance);

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

/*
 * Scan for the USB device.
 */
void InvokeStage(Experiment* exp);

/*
 * Update the Stage Tracker.
 * If the Stage tracker is not initialized, don't do anything.
 * If the stage tracker is initialized then either do the tracking,
 * or if we are in the process of turning off tracking off, then tell
 * the stage to halt and update flags.
 */
int HandleStageTracker(Experiment* exp);

void ShutOffStage(Experiment* exp);

int RecordStageTracker(Experiment* exp);

void ShowContoursGrayImg(Experiment* exp);

//在图片上添加LED以及帧率显示信息_实验中
void AddInfoText_Exp(Experiment* exp, IplImage* image);

//在图片上添加LED以及帧率显示信息_后台数据记录
void AddInfoText_Disk(Experiment* exp, IplImage* image, int distance);

//负责显示线虫的图像
void ElegansImShow(Experiment* exp);

#endif /* EXPERIMENT_H_ */



