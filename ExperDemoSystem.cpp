#include "ExperDemoSystem.h"
#include "ui_ExperDemoSystem.h"

ExperDemoSystem::ExperDemoSystem(QWidget* parent)
    : QWidget(parent)
    , m_cam_running(true), m_stage_running(true), Gui_Debug(false), ExpCanStart(false), socketStatus(false)
    , //m_socket(new QTcpSocket(this)), 
    m_server(new QTcpServer(this)), 
    m_histogram_running(true), m_record_running(true)
{
    ui.setupUi(this);

    // 确保窗口部件可以接收到键盘事件
    setFocusPolicy(Qt::StrongFocus);

    //用于位移台按键连续触发
    KeyTimerConnect();

    //连接SpinBox完成信号
    m_SpinBoxFinished();

    //连接tcp套接字的信号与槽函数
    TcpSocketSlotConnection();

    //是否打开GUI调试功能 - 单纯显示界面
    Gui_Debug = false;
    
    this->e = 0;       

    //初始化设备等
    this->e = PreCamOperation();

    if (this->e != 0)
        throw std::runtime_error("PreCamOperation Error");

    if (!Gui_Debug) {

        //相机拍摄线程 用于相机不间断拍摄
        m_cameraThread = QThread::create([this]() { captureImage(); });
        connect(m_cameraThread, &QThread::finished, m_cameraThread, &QObject::deleteLater);
        m_cameraThread->start();

        //位移台移动线程
        m_stageThread = QThread::create([this]() { moveStage(); });
        connect(m_stageThread, &QThread::finished, m_stageThread, &QObject::deleteLater);
        m_stageThread->start();
    }
    {
        //记录线程
        m_recordThread = QThread::create([this]() { recordData(); });
        connect(m_recordThread, &QThread::finished, m_recordThread, &QObject::deleteLater);
        m_recordThread->start();
    }

    //设置定时器事件显示图片 - 存在主线程中，用于与触发事件串行
    startTimer(50);

	// 初始化电场计时器
	m_efTimer = new QTimer(this); // 创建QTimer实例
	connect(m_efTimer, &QTimer::timeout, this, &ExperDemoSystem::updateTimerDisplay); // 连接信号和槽
	m_efTimer->start(100); // 设置每100毫秒更新一次界面（即每秒10次）
	m_efElapsedTime.start(); // 立即开始计时
}

//析构函数 程序结束触发
ExperDemoSystem::~ExperDemoSystem()
{
    //将线程标记清空
    m_cam_running = false;
    m_stage_running = false;
    m_record_running = false;

    if (!Gui_Debug) {
        m_cameraThread->quit();
        m_stageThread->quit();

        m_cameraThread->wait();
        m_stageThread->wait();
    }

    m_recordThread->quit();
    m_recordThread->wait();

    //结束记录
    FinishRecording(exp);

    //释放exp结构体
    if (this->exp) {
        ReleaseExperiment(this->exp);
        DestroyExperiment(&this->exp);
    }
}

/******************** 主要业务实现部分 *********************/

//同步控制界面参数
void ExperDemoSystem::SynchroControlPanel() {
    /* 图像显示参数控件 */
    exp->Params->MaskDiameter = ui.Diameter->value();
    exp->Params->MaskStyle = ui.CircleRbt->isChecked() ? 0 : 1;
    exp->Params->LowBoundary = ui.UpBoundary->value();
    exp->Params->RectHeigh = ui.RectHeight->value();
    exp->Params->BinThresh = ui.BinThreshSlider->value();
    //exp->Params->DilatePm = ui.DilateSlider->value();
    exp->Params->ErodePm = ui.ErodesSlider->value();

    /* 位移台参数控件 */
    exp->Params->maxstagespeed = ui.StageSpeed->value();
    //spinStage(*exp->MyStage, exp->Params->maxstagespeed, exp->Params->maxstagespeed);

    /* 光遗传刺激控件参数 */
    exp->Params->BrightnessPerc = ui.LedSlider->value();
}

//初始化函数
int ExperDemoSystem::PreCamOperation()
{
    //创建并初始化结构体
    this->exp = CreateExperimentStruct();
    InitializeExperiment(this->exp);

    //同步控制界面参数
    SynchroControlPanel();

    //初始化几个参数模块
    totalDistance = 0;
    curVelocity = 0;
    moveStep = 0.05;
    nematodePosition_x = 0;
    nematodePosition_y = 0;

    RealVoltage = {
        {0, -2},
        {1, 0},
        {2, 2}, 
        {3, 4},
        {4, 6},
        {5, 8}, 
        {6, 10},
        {7, 12},
        {8, 14},
        {9, 16}
    };

    //创建灰度直方图
    //initHistogram();

    if (!Gui_Debug) { // 如果是调试模式将不执行
        //初始化相机
        RollCameraInput(exp);

        //初始化Led控制模块
        //RollLedInput(exp);
    }

    if (this->exp->e != 0) return -1;
    else return 0;
}

//将mat格式转换为qImage格式
QImage ExperDemoSystem::MatToImage(const cv::Mat& mat) {
    if (mat.empty()) {
        return QImage();
    }

    switch (mat.type()) {
    case CV_8UC1: {
        QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8);
        return image;
    }
    case CV_8UC3: {
        QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    case CV_8UC4: {
        QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_ARGB32);
        return image;
    }
    default:
        break;
    }

    return QImage();
}

//用于绘制直方图
cv::Mat drawHistogram(const cv::Mat& grayImage) {
    // Calculate histogram
    int histSize = 256;
    float range[] = { 0, 256 };
    const float* histRange = { range };
    bool uniform = true;
    bool accumulate = false;

    cv::Mat hist;
    cv::calcHist(&grayImage, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

    // Create an image to display histogram
    int hist_w = 600;
    int hist_h = 400;
    int bin_w = std::round((double)hist_w / histSize);
    cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(255));

    // Normalize the result to [0, histImage.rows]
    cv::normalize(hist, hist, 0, histImage.rows - 10, cv::NORM_MINMAX);

    // Draw lines for each bin
    for (int i = 1; i < histSize; i++) {
        cv::line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
            cv::Point(bin_w * i, hist_h - cvRound(hist.at<float>(i))),
            cv::Scalar(255), 1);
    }
    return histImage;
}

//用于给mat格式图片添加信息
void addTextToImage(cv::Mat& image, const std::string& text1, const std::string& text2) {
    // 设置字体类型和缩放因子
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 2.0;
    int thickness = 2;
    cv::Scalar color(255, 255, 255); // 白色

    // 计算文本的尺寸
    int baseline = 0;
    cv::Size textSize1 = cv::getTextSize(text1, fontFace, fontScale, thickness, &baseline);
    cv::Size textSize2 = cv::getTextSize(text2, fontFace, fontScale, thickness, &baseline);

    // 设置文本位置
    cv::Point textOrg1(10, textSize1.height + 10); // 左上角，离顶部和左侧各10个像素
    cv::Point textOrg2(image.cols - textSize2.width - 10, textSize2.height + 10); // 右上角，离顶部和右侧各10个像素

    // 在图像上添加文本
    cv::putText(image, text1, textOrg1, fontFace, fontScale, color, thickness, 8);
    cv::putText(image, text2, textOrg2, fontFace, fontScale, color, thickness, 8);
}

//用于有关线虫图像的显示操作都放在这里
void ExperDemoSystem::ImageLabel() {
    //将Iplimage格式转换为Mat格式
    CvSize CurrentSize = cvGetSize(exp->fromCCD->iplimg);
    IplImage* image = cvCreateImage(CurrentSize, IPL_DEPTH_8U, 1);
    cvZero(image);
    cvCopy(exp->fromCCD->iplimg, image, NULL);

    IplImage* procImage = exp->Neuron->ImgThresh;

    cv::Mat mat = cv::cvarrToMat(image);
    cv::Mat procmat = cv::cvarrToMat(procImage);

    std::string eleganNum = "Test_" + std::to_string(ui.expNum->value());
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(2) << exp->Params->standardCorX; // 固定格式，两位小数
	std::string eleganDistance = " Distance: " + oss.str();
    std::string result = eleganNum + eleganDistance;
    addTextToImage(mat, result, std::to_string(exp->Neuron->frameNum));

    cv::resize(mat, mat, cv::Size(800, 600));
    cv::resize(procmat, procmat, cv::Size(800, 600));

    //将Mat格式转化为QImage格式
    QImage qImage = MatToImage(mat);           //原始图像
    QImage pImage = MatToImage(procmat);       //处理后图像

    //显示图像
    ui.ElegansImage->setPixmap(QPixmap::fromImage(qImage));
    ui.ProcImage->setPixmap(QPixmap::fromImage(pImage));

    cvReleaseImage(&image);
}

//定时器事件 用于显示图像
void ExperDemoSystem::timerEvent(QTimerEvent* e)
{
    //readFromServer();

    if (exp->fromCCD->iplimg == nullptr) return;

    if (ExpCanStart) {
        //只有当位移台准备好了才能开始进行实验 仅触发一次立马将Flag置反
        ui.ExpBegin->setStyleSheet("background-color: rgb(85, 170, 0);");
        ExpCanStart = false;
    }

    //如果位移台为空或者没有准备好，就不显示图像
    if (exp->MyStage == NULL || !exp->MyStage->stageIsPresent)
    {
        return;
    }

    //从服务器读取数据
    //readFromServer();

    //计算线虫图像相对坐标
    auto x = 0 - nematodePosition_x + exp->MyStage->AxisPos_x[0];
    auto y = nematodePosition_y - exp->MyStage->AxisPos_y[0];

    //将坐标显示在界面上
    exp->Params->standardCorX = x;   //将该坐标传回exp用于yaml记录
    exp->Params->standardCorY = y;
    ui.xPos->setText(QString::number(x * 1000, 'f', 2));
    ui.yPos->setText(QString::number(y * 1000, 'f', 2));
    ui.curVelocity->setText(QString::number(curVelocity * 1000, 'f', 2));  //将当前线虫速度也显示在界面上 单位um/s

    exp->Params->cur_velocity = curVelocity * 1000;  //将当前线虫速度值传回exp用于yaml记录  单位um/s
    exp->Params->Voltage = RealVoltage[Amp];   //将当前电场值传回exp用于yaml记录

	//将图像都显示到QLabel上
	ImageLabel();
}

//线程1：用于相机不间断拍照
void ExperDemoSystem::captureImage() {
    //设置计数器
    StartFrameRateTimer(exp);
    while (m_cam_running) {
        if (exp->e == -1 && (exp->Neuron->timestamp - exp->prevTime) > CLOCKS_PER_SEC)
        {
            std::cout << "[Error:GivenBoundaryFindWormHeadTail] The Boundary has too few points." << std::endl;
        }
        exp->e = 0;

        /** 采集照片 **/
        if (GrabFrame(exp) == -1) break;

        /** 计算帧率并保存至exp以取用 **/
        CalculateAndPrintFrameRate(exp);

        //界面显示当前帧率
        ui.FPS->setText(QString("%1").arg(exp->FramePerSec));

        /** 将相机中拍到的图像导入Neuron等待处理 **/
        if (exp->e == 0) {
            exp->e = LoadNeuronImg(exp->Neuron, exp->Params, exp->fromCCD->iplimg, exp->stageFeedbackTargetOffset);
        }

        if (exp->Params->OnOff) {
            /** 对采集到的图像进行切割 **/
            DoSegmentation(exp);
        }
    }

    //关闭相机
    T2Cam_TurnOff(*(this->exp->MyCamera));
}

//计算当前线虫速度
double NematodeDisplace(Experiment* exp, int delay, double& totalDiatance) {
    auto preX = exp->MyStage->prePos_x, preY = exp->MyStage->prePos_y;
    auto curX = exp->MyStage->AxisPos_x[0], curY = exp->MyStage->AxisPos_y[0];

    double dx = curX - preX, dy = curY - preY;

    double distance = std::sqrt(dx * dx + dy * dy);
    auto mq = exp->NemDisp;
    if (mq->size() < 5) {
        mq->push(distance);
        totalDiatance += distance;
    }
    else {
        totalDiatance -= mq->front();
        mq->pop();
        mq->push(distance);
        totalDiatance += distance;

        double time = (delay * 5) / 1000;//计算mq队列中5次跟踪的总位移，然后平均算出运动速度

        return totalDiatance / time;
    }

    return 0;
}

//线程2：用于位移台自动跟踪
void ExperDemoSystem::moveStage() {
    std::cout << std::endl;
    std::cout << "************ Stage ************" << std::endl;
    std::cout << "invoking stage..." << std::endl;

    //设定位移台初始的位置  线程里的循环，我想让它一秒钟执行2次
    exp->x_initPos = static_cast<float>(82.3);
    exp->y_initPos = static_cast<float>(47.4);
    InvokeStage(exp);

    ExpCanStart = true;

    std::cout << "TrackingThread has been invoked" << std::endl;

	// 在循环外初始化一个连续失败计数器
	int consecutive_failures = 0;

	// --- 新的固定周期频率控制逻辑 ---
	// 目标周期：80毫秒 (10Hz)
	const auto loop_interval = std::chrono::milliseconds(80);

	while (m_stage_running) {
		// 1. 记录循环开始的时刻
		auto loop_start_time = std::chrono::steady_clock::now();

		// 2. 执行核心跟踪任务
		// HandleStageTracker 现在会内部等待上一次移动完成，然后才执行新的移动
        int tracker_status = HandleStageTracker(exp);

// 		// 检查任务是否成功
// 		if (tracker_status == 0) {
// 			// 如果成功了，清零失败计数器
// 			if (consecutive_failures > 0) {
// 				std::cout << "[Info] Stage communication has recovered." << std::endl;
// 			}
// 			consecutive_failures = 0;
// 		}
// 		else {
// 			// 如果任务失败，增加失败计数
// 			consecutive_failures++;
// 			std::cout << "[Warning] Tracking task failed. Attempt "
// 				<< consecutive_failures << "/" << 50
// 				<< "." << std::endl;
// 
// 			// 检查连续失败次数是否超过阈值
// 			if (consecutive_failures >= 50) {
// 				std::cout << "[FATAL ERROR] Exceeded max retry attempts. Stopping Stage Thread." << std::endl;
// 				m_stage_running = false; // 标记为停止
// 				break;                   // 立即跳出循环
// 			}
// 		}

		// 计算线虫运动速度
		// 注意：这里的delay参数现在可以传递80，表示我们期望的周期
		curVelocity = NematodeDisplace(exp, 80, totalDistance);

		// 3. 计算从循环开始到现在，所有任务花费的总时间
		auto loop_end_time = std::chrono::steady_clock::now();
		auto task_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);

		// 4. 计算为了凑够100ms，还需要睡眠多久
		auto sleep_duration = loop_interval - task_duration;

		// 5. 执行睡眠
		if (sleep_duration > std::chrono::milliseconds(0)) {
			// 如果任务执行得快，就睡眠剩下的时间
			std::this_thread::sleep_for(sleep_duration);
		}
		else {
			// 如果任务执行时间已经超过或等于100ms，就不再睡眠，立即开始下一次跟踪
			// 这确保了系统不会因长时间的等待而丢失节拍
			std::cout << "[Warning] Tracking loop overshot target interval. Actual time: "
				<< task_duration.count() << "ms" << std::endl;
		}
		// 原有的 k++, EverySoOften(), cvWaitKey(delay) 都被此逻辑替代
	}

    //关闭位移台
    UnInitialize_Stage(*(exp->MyStage));
    T2Stage_TurnOff(&(exp->MyStage));
}

//线程3：用于记录线虫数据
void ExperDemoSystem::recordData() {
    while (m_record_running) {
        if (!exp->Params->Record) continue;

        auto start = std::chrono::high_resolution_clock::now();

        //每隔40ms记录一次相机数据，记录耗时2~3ms，可能存在线程阻塞问题
        DoWriteToDisk(exp, static_cast<int>(nematodePosition_x));

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        if (duration.count() > 1000)
        {//超过1s会报警，并打印输出
            std::cout << "[WARNING] DoWriteToDisk took " 
                << duration.count() << " ms" << std::endl;
        }

    //    auto remaining_wait = std::chrono::milliseconds(40) - duration;
    //    if (remaining_wait > std::chrono::milliseconds(0))
    //    {//如果在40ms内，可以选择打印信息
    //        std::cout << "Frame #" << exp->Neuron->frameNum << " take " 
    //            << duration.count() << " ms" << std::endl;
    //        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    //    }
    //    else
    //    {//记录时间超过40ms会直接打印信息
    //        std::cout << "[INFO] DoWriteToDisk exceeded 40ms cycle time by "
				//<< -remaining_wait.count() << " ms" << std::endl;
    //    }
    }
}

/********************   控件部分  *********************/
/* 图像显示参数控件 */
void ExperDemoSystem::on_ExpBegin_clicked() {

    if (!Gui_Debug) {
        if (exp->MyStage == NULL || !exp->MyStage->stageIsPresent)
        {
            return;
        }
    }

    //按钮样式改变
    if (ui.ExpBegin->text() == "Begin Experiment") {
        ui.ExpBegin->setText("End Experiment");
        ui.ExpBegin->setStyleSheet("background-color: #dc0e0e;");

        ui.Track->setStyleSheet("background-color: rgb(85, 170, 0);");
        ui.SaveData->setStyleSheet("background-color: rgb(85, 170, 0);");

        exp->Params->OnOff = 1;  //实验开始，开始处理数据
    }
    else {
        ui.ExpBegin->setText("Begin Experiment");
        ui.ExpBegin->setStyleSheet("background-color: rgb(85, 170, 0);");

        ui.Track->setStyleSheet("background-color: #f0f0f0;");
        ui.SaveData->setStyleSheet("background-color: #f0f0f0;");

        if (ui.Track->text() == "End Track")
            ui.Track->setText("Start Track");

        if (ui.SaveData->text() == "Saving ...")
            ui.SaveData->setText("Save");

        //停止实验记录
        FinishRecording(exp);

        if (exp->Params->stageTrackingOn)//如果位移台还在跟踪 先停止跟踪
            Toggle(&(exp->Params->stageTrackingOn));
        exp->Params->OnOff = 0;   //实验结束，停止处理数据
    }
}

void ExperDemoSystem::on_ContrastImp_clicked() {
    if (exp->MyCamera == NULL) return;
    exp->MyCamera->resetMaxAndMin = true;
}

void ExperDemoSystem::m_Diameter_valueChanged() {
    int value = ui.Diameter->value();
    exp->Params->MaskDiameter = value;
}

void ExperDemoSystem::m_UpBoundary_valueChanged() {
    int value = ui.UpBoundary->value();
    ui.UpBoundarySlider->setValue(value);
    //exp->Params->LowBoundary = value;
}

void ExperDemoSystem::on_UpBoundarySlider_valueChanged(int value) {
    ui.UpBoundary->setValue(value);
    exp->Params->LowBoundary = value;
}

void ExperDemoSystem::m_RectHeight_valueChanged() {
    int value = ui.RectHeight->value();
    ui.RectHeightSlider->setValue(value);
    //exp->Params->RectHeigh = value;
}

void ExperDemoSystem::on_RectHeightSlider_valueChanged(int value) {
    ui.RectHeight->setValue(value);
    exp->Params->RectHeigh = value;
}

void ExperDemoSystem::m_BinThreshBox_valueChanged() {
    int value = ui.BinThreshBox->value();
    ui.BinThreshSlider->setValue(value);
    //exp->Params->BinThresh = value;
}

void ExperDemoSystem::m_ErodesThresh_valueChanged() {
    //暂时未添加该参数
    int value = ui.ErodesThresh->value();
    ui.ErodesSlider->setValue(value);
}

void ExperDemoSystem::on_ErodesSlider_valueChanged(int value) {
    ui.ErodesThresh->setValue(value);
    exp->Params->ErodePm = value;
}

void ExperDemoSystem::on_BinThreshSlider_valueChanged(int value) {
    ui.BinThreshBox->setValue(value);
    exp->Params->BinThresh = value;
}

void ExperDemoSystem::on_CircleRbt_clicked() {
    exp->Params->MaskStyle = ui.CircleRbt->isChecked() ? 0 : 1;
}

void ExperDemoSystem::on_RectRbt_clicked() {
    exp->Params->MaskStyle = ui.RectRbt->isChecked() ? 1 : 0;
}

/* 位移台参数控件 */
void FindeStagePos(Experiment* exp) {
    //每次按钮移动位移台的时候，更新Neuron里的坐标位置
    if (FindStagePosition(*(exp->MyStage))) {
        exp->Neuron->stagePosition_x = exp->MyStage->AxisPos_x[0];
        exp->Neuron->stagePosition_y = exp->MyStage->AxisPos_y[0];
        exp->MyCamera->resetMaxAndMin = true;  //重新设定画质对比度
    }
}

void ExperDemoSystem::on_Up_clicked() {
    //std::cout << "Up" << std::endl;

    if (exp->MyStage == NULL) return; //如果位移台为空，不接收指令
    if (!exp->MyStage->stageIsPresent) return; //如果位移台还没有准备好，不接收指令

	// 确保程序的运动逻辑是基于非锁定状态执行的。
	if (exp->Params) {
		exp->Params->AxisLockState = 0;
	}
    // 这会让用户界面同步反映出当前已经是自由移动状态。
    ui.radioAxisFree->setChecked(true);

    MoveRelative(exp->MyStage->handle, (float)(0 - moveStep), AXIS_Y);
    FindeStagePos(exp);
}

void ExperDemoSystem::on_Down_clicked() {
    //std::cout << "Down" << std::endl;

    if (exp->MyStage == NULL) return;
    if (!exp->MyStage->stageIsPresent) return;

	if (exp->Params) {
		exp->Params->AxisLockState = 0;
	}
	ui.radioAxisFree->setChecked(true);

    MoveRelative(exp->MyStage->handle, moveStep, AXIS_Y);
    FindeStagePos(exp);
}

void ExperDemoSystem::on_Right_clicked() {
    //std::cout << "Right" << std::endl;

    if (exp->MyStage == NULL) return;
    if (!exp->MyStage->stageIsPresent) return;

	if (exp->Params) {
		exp->Params->AxisLockState = 0;
	}
	ui.radioAxisFree->setChecked(true);

    MoveRelative(exp->MyStage->handle, (float)(0 - moveStep), AXIS_X);
    FindeStagePos(exp);
}

void ExperDemoSystem::on_Left_clicked() {
    //std::cout << "Left" << std::endl;

    if (exp->MyStage == NULL) return;
    if (!exp->MyStage->stageIsPresent) return;

	if (exp->Params) {
		exp->Params->AxisLockState = 0;
	}
	ui.radioAxisFree->setChecked(true);

    MoveRelative(exp->MyStage->handle, moveStep, AXIS_X);
    FindeStagePos(exp);
}

void ExperDemoSystem::m_StageSpeed_valueChanged() {

    if (exp->MyStage == NULL) return;
    if (!exp->MyStage->stageIsPresent) return;

    double value = ui.StageSpeed->value();

    float vel_x = static_cast<float>(value);
    float vel_y = static_cast<float>(value);
    spinStage(*exp->MyStage, vel_x, vel_y);
    exp->Params->maxstagespeed = value;
}

void ExperDemoSystem::on_axisRadioButton_toggled(bool checked)
{
	// 只在按钮被“选中”时执行逻辑 (checked == true)
	if (!checked) {
		return;
	}

	if (exp == NULL || exp->Params == NULL) return;
    if (!exp->MyStage->stageIsPresent) return;

	// 获取发送信号的那个按钮
	QObject* buttonObj = sender();

	// 判断是哪个按钮，然后更新状态值
	if (buttonObj == ui.radioAxisFree) {
		exp->Params->AxisLockState = 0;
	}
	else if (buttonObj == ui.radioLockX) {
		exp->Params->AxisLockState = 1;
	}
	else if (buttonObj == ui.radioLockY) {
		exp->Params->AxisLockState = 2;
	}

}

// void ExperDemoSystem::on_YMoveLock_clicked() {
// 
//     if (exp->MyStage == NULL) return;
//     if (!exp->MyStage->stageIsPresent) return;
// 
//     if (ui.YMoveLock->text() == "YMoveLock") {
//         ui.YMoveLock->setText("Set Y Free");
//         ui.YMoveLock->setStyleSheet("background-color: #dc0e0e;");
// 
//         exp->Params->YMoveLock = 1;
//     }
//     else {
//         ui.YMoveLock->setText("YMoveLock");
//         ui.YMoveLock->setStyleSheet("background-color: #f0f0f0;");
// 
//         exp->Params->YMoveLock = 0;
//     }
// }

void ExperDemoSystem::on_Track_clicked() {
    auto button = ui.Track;
    
    if (ui.ExpBegin->text() == "End Experiment") {
        //如果没有开始实验，该按钮是不起作用的
        if (button->text() == "Start Track") {
            button->setText("End Track");

            //开始跟踪
            if (exp->Params->stageTrackingOn != 1) {
                Toggle(&(exp->Params->stageTrackingOn));
            }

            button->setStyleSheet("background-color: #dc0e0e;");//红色
        }
        else {
            button->setText("Start Track");

            //停止跟踪
            if (exp->Params->stageTrackingOn != 0) {
                Toggle(&(exp->Params->stageTrackingOn));
            }

            auto acc = ui.AccSet->value();

            AcDecSet(exp->MyStage->handle, acc, AXIS_X);
            AcDecSet(exp->MyStage->handle, acc, AXIS_Y);

            if (ui.ExpBegin->text() == "End Experiment")
                button->setStyleSheet("background-color: rgb(85, 170, 0);");
        }
    }
}

void ExperDemoSystem::on_OriginPoint_clicked() {
    nematodePosition_x = exp->MyStage->AxisPos_x[0];
    nematodePosition_y = exp->MyStage->AxisPos_y[0];
}

void ExperDemoSystem::m_AccSet_valueChanged() {
    if (exp->MyStage == NULL) return;
    if (!exp->MyStage->stageIsPresent) return;

    float acc = static_cast<float>(ui.AccSet->value());
    
    AcDecSet(exp->MyStage->handle, acc, AXIS_X);
    AcDecSet(exp->MyStage->handle, acc, AXIS_Y);
}

/* 光电控制模块控件 */
void ExperDemoSystem::on_LedSlider_valueChanged(int value) {
    ui.LedSpinBox->setValue(value);
    exp->Params->BrightnessPerc = value;
    //std::cout << "exp->Params->BrightnessPerc: " << exp->Params->BrightnessPerc << std::endl;

    //向LED控制器写入数据
    if (exp->MyLed == NULL) return;
    TLDC_TurnLedOnOff(*(exp->MyLed), LED0, exp->Params->Led0_status, exp->Params->BrightnessPerc);
}

void ExperDemoSystem::m_LedSpinBox_valueChanged() {
    int value = ui.LedSpinBox->value();

    ui.LedSlider->setValue(value);
    exp->Params->BrightnessPerc = value;
    //std::cout << "exp->Params->BrightnessPerc: " << exp->Params->BrightnessPerc << std::endl;
}

void ExperDemoSystem::on_LedOnOff_clicked() {

    auto button = ui.LedOnOff;
    if (button->text() == "ON") {
        button->setText("OFF");
        button->setStyleSheet("background-color: #dc0e0e;");

        exp->Params->Led0_status = 1;
    }
    else {
        button->setText("ON");
        button->setStyleSheet("background-color: rgb(85, 170, 0);");

        exp->Params->Led0_status = 0;
    }

    if (exp->MyLed == NULL) return;
    
    //调节Led控制器灯光
    //std::cout << "exp->Params->BrightnessPerc: " << exp->Params->BrightnessPerc << std::endl;
    TLDC_TurnLedOnOff(*(exp->MyLed), LED0, exp->Params->Led0_status, exp->Params->BrightnessPerc);
}


/* 电场刺激模块控件 */
void ExperDemoSystem::on_voltageSlider_valueChanged(int value)
{
	ui.voltageBox->setValue(value);
//     exp->Params->vDir = value < 0 ? '1' : '0';     //如果数值为负数，方向为1，数值为正数，方向为0     //注释掉，仅用来保证UI界面同步，不改变Params值以保证记录模块记录的是真实电场
// 	int vol = abs(value);    //去绝对值
// 	if (vol < 10)
//         exp->Params->vNum = vol + '0';
// 	else
// 	{
// 		const char* vStr = "abcdefg";
//         exp->Params->vNum = *(vStr + (vol - 10));
// 	}
}

void ExperDemoSystem::on_voltageBox_valueChanged()
{
    int value = ui.voltageBox->value();
    ui.voltageSlider->setValue(value);
    
//     // 更新 vDir 和 vNum 的值
//     exp->Params->vDir = value < 0 ? '1' : '0';     //如果数值为负数，方向为1，数值为正数，方向为0
//     int vol = abs(value);    //去绝对值
//     if (vol < 10)
//         exp->Params->vNum = vol + '0';
//     else
//     {
//         const char* vStr = "abcdefg";
//         exp->Params->vNum = *(vStr + (vol - 10));
//     }
}

void ExperDemoSystem::on_waveformBox_currentIndexChanged(int)
{
//     int index = ui.waveformBox->currentIndex();
//     exp->Params->vWave = index + '0';  // 将索引值转换为字符
//     std::cout << "exp->Params->vWave: " << exp->Params->vWave << std::endl;
}

void ExperDemoSystem::on_Electric1_Stop_clicked() {
	// 设置所有按钮的背景色为白色
	ui.Ins2_right->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_down->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_up->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_left->setStyleSheet("background-color: rgb(255, 255, 255);");

	// 不选取通道
	exp->Params->EF_Mode = 0;
	std::cout << "exp->Params->EF_Mode: " << exp->Params->EF_Mode << std::endl;
	if (ui.LedOnOff->text() == "OFF")
		emit ui.LedOnOff->clicked();

    exp->Params->vChannel = '0';

	// 【新增】: 明确设置所有电场参数为“停止”状态，以保证记录的准确性
	exp->Params->vChannel = '0';
	exp->Params->vWave = '0';
	exp->Params->vDir = '0';
	exp->Params->vNum = '0';

	// 发送更新的 EF_Mode 值
	exp->sender->changeMessage(exp->Params->vChannel, exp->Params->vDir, exp->Params->vNum, exp->Params->vWave);

    //解除位移台锁定状态
	if (exp->Params) {
		exp->Params->AxisLockState = 0;
	}
	ui.radioAxisFree->setChecked(true);
    m_efElapsedTime.restart();
}

void ExperDemoSystem::on_Electric1_Right_clicked() {
	// 设置 Ins2_right 的背景色为绿色，设置其他按钮的背景色为白色
	ui.Ins2_right->setStyleSheet("background-color: rgb(85, 255, 127);");
	ui.Ins2_down->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_up->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_left->setStyleSheet("background-color: rgb(255, 255, 255);");

	// 选取12通道
	exp->Params->EF_Mode = 1;
	std::cout << "exp->Params->EF_Mode: " << exp->Params->EF_Mode << std::endl;
	if (ui.LedOnOff->text() == "OFF")
		emit ui.LedOnOff->clicked();


	// 【新增】: 在此处从UI读取最新参数并更新exp->Params
	int value = ui.voltageBox->value();
	int index = ui.waveformBox->currentIndex();

	exp->Params->vChannel = '1'; // 此按钮设置通道为'1'
	exp->Params->vWave = index + '0';
	exp->Params->vDir = value < 0 ? '1' : '0';
	int vol = abs(value);
	if (vol < 10) {
		exp->Params->vNum = vol + '0';
	}
	else {
		const char* vStr = "abcdefg";
		exp->Params->vNum = *(vStr + (vol - 10));
	}


	// 发送更新的 EF_Mode 值
    exp->sender->changeMessage(exp->Params->vChannel, exp->Params->vDir, exp->Params->vNum, exp->Params->vWave);

	if (exp->Params) {
		exp->Params->AxisLockState = 0;
	}
	ui.radioAxisFree->setChecked(true);
    m_efElapsedTime.restart();
}

void ExperDemoSystem::on_Electric1_Down_clicked() {
	// 设置 Ins2_down 的背景色为绿色，设置其他按钮的背景色为白色
	ui.Ins2_right->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_down->setStyleSheet("background-color: rgb(85, 255, 127);");
	ui.Ins2_up->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_left->setStyleSheet("background-color: rgb(255, 255, 255);");

	// 选取23通道
	exp->Params->EF_Mode = 2;
	std::cout << "exp->Params->EF_Mode: " << exp->Params->EF_Mode << std::endl;
	if (ui.LedOnOff->text() == "OFF")
		emit ui.LedOnOff->clicked();


	// 【新增】: 在此处从UI读取最新参数并更新exp->Params
	int value = ui.voltageBox->value();
	int index = ui.waveformBox->currentIndex();

	exp->Params->vChannel = '2'; // 此按钮设置通道为'2'
	exp->Params->vWave = index + '0';
	exp->Params->vDir = value < 0 ? '1' : '0';
	int vol = abs(value);
	if (vol < 10) {
		exp->Params->vNum = vol + '0';
	}
	else {
		const char* vStr = "abcdefg";
		exp->Params->vNum = *(vStr + (vol - 10));
	}


	// 发送更新的 EF_Mode 值
    exp->sender->changeMessage(exp->Params->vChannel, exp->Params->vDir, exp->Params->vNum, exp->Params->vWave);

	if (exp->Params) {
		exp->Params->AxisLockState = 0;
	}
	ui.radioAxisFree->setChecked(true);
    m_efElapsedTime.restart();
}

void ExperDemoSystem::on_Electric1_Left_clicked() {
	// 设置 Ins2_left 的背景色为绿色，设置其他按钮的背景色为白色
	ui.Ins2_right->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_down->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_up->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_left->setStyleSheet("background-color: rgb(85, 255, 127);");

	// 选取34通道
	exp->Params->EF_Mode = 3;
	std::cout << "exp->Params->EF_Mode: " << exp->Params->EF_Mode << std::endl;
	if (ui.LedOnOff->text() == "OFF")
		emit ui.LedOnOff->clicked();

    
	// 【新增】: 在此处从UI读取最新参数并更新exp->Params
	int value = ui.voltageBox->value();
	int index = ui.waveformBox->currentIndex();

	exp->Params->vChannel = '3'; // 此按钮设置通道为'3'
	exp->Params->vWave = index + '0';
	exp->Params->vDir = value < 0 ? '1' : '0';
	int vol = abs(value);
	if (vol < 10) {
		exp->Params->vNum = vol + '0';
	}
	else {
		const char* vStr = "abcdefg";
		exp->Params->vNum = *(vStr + (vol - 10));
	}


 	// 发送更新的 EF_Mode      
    exp->sender->changeMessage(exp->Params->vChannel, exp->Params->vDir, exp->Params->vNum, exp->Params->vWave);

	if (exp->Params) {
		exp->Params->AxisLockState = 0;
	}
	ui.radioAxisFree->setChecked(true);
    m_efElapsedTime.restart();
}      


void ExperDemoSystem::on_Electric1_Up_clicked() {
	// 设置 Ins2_up 的背景色为绿色，设置其他按钮的背景色为白色
	ui.Ins2_right->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_down->setStyleSheet("background-color: rgb(255, 255, 255);");
	ui.Ins2_up->setStyleSheet("background-color: rgb(85, 255, 127);");
	ui.Ins2_left->setStyleSheet("background-color: rgb(255, 255, 255);");

	// 选取41通道
	exp->Params->EF_Mode = 4;
	std::cout << "exp->Params->EF_Mode: " << exp->Params->EF_Mode << std::endl;
	if (ui.LedOnOff->text() == "OFF")
		emit ui.LedOnOff->clicked();

    
	// 【新增】: 在此处从UI读取最新参数并更新exp->Params
	int value = ui.voltageBox->value();
	int index = ui.waveformBox->currentIndex();

	exp->Params->vChannel = '4'; // 此按钮设置通道为'4'
	exp->Params->vWave = index + '0';
	exp->Params->vDir = value < 0 ? '1' : '0';
	int vol = abs(value);
	if (vol < 10) {
		exp->Params->vNum = vol + '0';
	}
	else {
		const char* vStr = "abcdefg";
		exp->Params->vNum = *(vStr + (vol - 10));
	}


	// 发送更新的 EF_Mode 值
    exp->sender->changeMessage(exp->Params->vChannel, exp->Params->vDir, exp->Params->vNum, exp->Params->vWave);

	if (exp->Params) {
		exp->Params->AxisLockState = 0;
	}
	ui.radioAxisFree->setChecked(true);
    m_efElapsedTime.restart();
}


// 计时器UI更新函数
void ExperDemoSystem::updateTimerDisplay(){
	// 获取自m_efElapsedTime上次调用start()或restart()以来经过的毫秒数
	qint64 elapsedTime = m_efElapsedTime.elapsed();

	// 逻辑修改：判断是否超过 60 秒 (60 * 1000 = 60000 毫秒)
	if (elapsedTime >= 60000) {
		// 超过60秒，设置文本颜色为红色，并加粗（可选）
		ui.ef_timerLabel->setStyleSheet("color: red; font-weight: bold;");
	}
	else {
		// 未超过60秒，恢复默认样式（例如黑色或系统默认色），
		// 这一步很重要，防止计时器重置后仍然是红色
		ui.ef_timerLabel->setStyleSheet("");
		// 如果你的界面背景是深色，默认字体是白色，这里可以用 "color: white;" 或 "color: black;" 明确指定
	}

	// 将毫秒数格式化为 MM:ss.z (分钟:秒.十分之一秒) 的格式
	QString formattedTime = QTime::fromMSecsSinceStartOfDay(elapsedTime).toString("mm:ss.z");

	// 在UI上我们创建的那个QLabel控件上更新文本
	ui.ef_timerLabel->setText(formattedTime);
}

    /* 远程连接模块 */
//在该模块下创建随机刺激、强化学习后刺激组件
/**
 * @brief 当“随机刺激”按钮被点击时触发的槽函数。
 * * 该函数会根据预设的参数空间，随机选择一种波形和一种电压等级，
 * 并将这些值设置到用户界面(UI)的相应控件上。
 * * 注意：此函数只更新UI显示，不会实际发送UDP指令来施加电场。
 */
void ExperDemoSystem::on_Random_Stimulus_clicked()
{
    // 1. 定义刺激参数空间
    // =================================================================

    // 根据我们之前设计的方案，定义电压等级 (单位: V)
    const QVector<int> voltageLevels = { 4, 8, 12 };

    // 定义波形的总数。
    // 假设您的 "Waveform" 下拉框 (QComboBox) 中有5个选项，
    // 它们的索引从0到4分别对应: 正弦波, 方波, 直流, 锯齿波, 三角波。
    const int totalWaveformTypes = 3;

    // 2. 生成随机参数
    // =================================================================

    // (a) 从预设的电压等级数组中，随机选择一个电压值
    // QRandomGenerator::global()->bounded(N) 会生成一个 [0, N-1] 范围内的随机整数
    int randomVoltageIndex = QRandomGenerator::global()->bounded(voltageLevels.size());
    int chosenVoltage = voltageLevels[randomVoltageIndex];

    // (b) 随机选择一个波形的索引
    int chosenWaveformIndex = QRandomGenerator::global()->bounded(totalWaveformTypes) + 1;

    // 3. 将随机选择的参数更新到UI控件上
    // =================================================================

	if (ui.voltageSlider) {
		ui.voltageSlider->setValue(chosenVoltage);
	}
	else {
		std::cout << "Warning: Voltage slider control named horizontalSlider_Voltage not found." << std::endl;
	}

	if (ui.waveformBox) {
		ui.waveformBox->setCurrentIndex(chosenWaveformIndex);
	}
	else {
		std::cout << "Warning: Waveform dropdown control named comboBox_Waveform not found." << std::endl;
	}


    // 4. (可选) 在应用程序的输出窗口打印日志，方便您确认本次随机选择的结果
    // =================================================================
//     if (ui->comboBox_Waveform) {
//         QString chosenWaveformName = ui->comboBox_Waveform->itemText(chosenWaveformIndex);
//         qDebug() << "随机刺激参数已在UI上设置 -> 波形:" << chosenWaveformName
//             << "(索引:" << chosenWaveformIndex << ")"
//             << ", 电压:" << chosenVoltage << "V";
//     }
}


//     void ExperDemoSystem::on_Random_Stimulus_clicked() {
// 		// 首先将电场刺激设置为 "Stop"
// 		//on_Electric1_Stop_clicked();
// 		//exp->sender->changeMessage(exp->Params->vChannel, exp->Params->vDir, exp->Params->vNum);
// 
// 		// 检查LED是否关闭
// 		if (ui.LedOnOff->text() == "ON") {
// 			// 随机设置光强（从 25, 50, 75, 100 中选择）
// 			int randomBrightness = QRandomGenerator::global()->bounded(0, 4) * 25 + 25;
// 			ui.LedSlider->setValue(randomBrightness); 
// 			ui.LedSpinBox->setValue(randomBrightness);
// 			exp->Params->BrightnessPerc = randomBrightness;
// 
// 			// 打开LED
// 			ui.LedOnOff->setText("OFF");
// 			ui.LedOnOff->setStyleSheet("background-color: #dc0e0e;");
// 			exp->Params->Led0_status = 1;
// 
// 			// 调用 LED 控制函数
// 			if (exp->MyLed != NULL) {
// 				TLDC_TurnLedOnOff(*(exp->MyLed), LED0, exp->Params->Led0_status, exp->Params->BrightnessPerc);
// 			}
// 
// 			// 随机设置光刺激时间（3秒、5秒、10秒）
// 			int randomTime = (qrand() % 3) + 3;
// 			QTimer::singleShot(randomTime * 1000, this, [this]() {
// 				// 关闭LED
// 				ui.LedOnOff->setText("ON");
// 				ui.LedOnOff->setStyleSheet("background-color: rgb(85, 170, 0);");
// 				exp->Params->Led0_status = 0;
// 
// 				// 调用 LED 控制函数
// 				if (exp->MyLed != NULL) {
// 					TLDC_TurnLedOnOff(*(exp->MyLed), LED0, exp->Params->Led0_status, exp->Params->BrightnessPerc);
// 				}
// 
// 				// 顺时针改变电场方向
// 				// 记录当前的电场方向
// 				int currentDirection = exp->Params->EF_Mode;
//                 int nextDirection = (currentDirection % 4) + 1; // 1-4 转换为 2-5 然后取模
// 
// 				switch (nextDirection) {
// 				case 1: // Right
// 					on_Electric1_Right_clicked();
// 					break;
// 				case 2: // Down
// 					on_Electric1_Down_clicked();
// 					break;
// 				case 3: // Left
// 					on_Electric1_Left_clicked();
// 					break;
// 				case 4: // Up
// 					on_Electric1_Up_clicked();
// 					break;
// 				}
// 
// 				// 更新电场参数
// 				//exp->sender->changeMessage(exp->Params->vChannel, exp->Params->vDir, exp->Params->vNum);
// 				});
// 		}
// 	}
    

    //用于Tcp连接的槽函数-客户端
    void ExperDemoSystem::TcpSocketSlotConnection()
    {
        connect(m_server, &QTcpServer::newConnection, this, &ExperDemoSystem::handleNewConnection);
    }

    //用于读取服务器发送过来的数据
    void ExperDemoSystem::readFromServer(QString line) {
	    if (socketStatus) {
		    if (line == "0") {
			    //emit ui.operation1->clicked();
		    }
		    else if (line == "1") {
			    emit ui.operation2->clicked();
		    }
		    else if (line == "2") {
			    emit ui.operation3->clicked();
		    }
	    }
    }

    //void ExperDemoSystem::on_operation1_clicked() {
    //    //预设操作：使线虫停止

	   // ui.Ins1->setStyleSheet("background-color: rgb(85, 255, 127);");
	   // ui.Ins2->setStyleSheet("background-color: rgb(255, 255, 255);");
	   // ui.Ins3->setStyleSheet("background-color: rgb(255, 255, 255);");
    //#if 0
	   // //emit ui.EF_Stop->clicked();
	   // if (ui.LedOnOff->text() == "ON")
		  //  emit ui.LedOnOff->clicked();
    //#else    //此处需要改回无控制 电压为0
    //    //std::cout << "Square Wave -4 ~ 4v " << std::endl;
	   // //exp->sender->SquareWave('3', 1, 50);
    //    //exp->sender->stopSquareWave();
	   // //exp->Params->curType = exp->sender->curType;
    //#endif
    //}

    void ExperDemoSystem::on_operation2_clicked() {
        //预设操作：使线虫向反方向行走
	    ui.Ins2->setStyleSheet("background-color: rgb(85, 255, 127);");
	    ui.Ins1->setStyleSheet("background-color: rgb(255, 255, 255);");
	    ui.Ins3->setStyleSheet("background-color: rgb(255, 255, 255);");

        //emit ui.EF_Mode1->clicked();    
        if (ui.LedOnOff->text() == "OFF")
            emit ui.LedOnOff->clicked();
    }

    void ExperDemoSystem::on_operation3_clicked() {
        //预设操作：使线虫向目标行走
	    ui.Ins3->setStyleSheet("background-color: rgb(85, 255, 127);");
	    ui.Ins1->setStyleSheet("background-color: rgb(255, 255, 255);");
	    ui.Ins2->setStyleSheet("background-color: rgb(255, 255, 255);");

        //emit ui.EF_Mode2->clicked();    
        if (ui.LedOnOff->text() == "OFF")
            emit ui.LedOnOff->clicked();
    }

void ExperDemoSystem::on_RemoteConnect_clicked() {
    if (!socketStatus) {   // 开始连接远程服务器
        ui.RemoteStatus->setText("Waiting for connection...");

        //QString IP = ui.IPtext->text();
		//m_socket->connectToHost(IP, 6340);
        m_server->listen(QHostAddress::Any, 6340);
        socketStatus = true;
    }
    else {  // 断开连接
        //m_socket->disconnectFromHost();
        m_server->close();
        m_socket = nullptr;
    }
}

void ExperDemoSystem::onSocketError(QAbstractSocket::SocketError socketError)
{
	switch (socketError) {
	case QAbstractSocket::HostNotFoundError:
        ui.RemoteStatus->setText("Host not found.");
		break;
	case QAbstractSocket::ConnectionRefusedError:
        ui.RemoteStatus->setText("Connection refused by the server.");
		break;
	case QAbstractSocket::RemoteHostClosedError:
        ui.RemoteStatus->setText("The remote host closed the connection.");
		break;
		// 处理其他错误类型
	default:
        ui.RemoteStatus->setText(m_socket->errorString());
        qDebug() << m_socket->errorString();
		break;
	}
}

void ExperDemoSystem::handleNewConnection()
{
    m_socket = m_server->nextPendingConnection();
	connect(m_socket, &QTcpSocket::readyRead, this, &ExperDemoSystem::recv_data);
	connect(m_socket, &QTcpSocket::connected, this, &ExperDemoSystem::onConnected);
	connect(m_socket, &QTcpSocket::errorOccurred, this, &ExperDemoSystem::onSocketError);
	connect(m_socket, &QTcpSocket::disconnected, this, &ExperDemoSystem::onDisconnected);
}

void ExperDemoSystem::recv_data() {
    QByteArray data = m_socket->readAll();
    QString message = QString::fromUtf8(data);
    ui.RemoteStatus->setText("Receive Msg:" + message);
    readFromServer(message);
}

void ExperDemoSystem::onConnected() {
    ui.RemoteStatus->setText("Connected to client");
    ui.RemoteConnect->setStyleSheet("background-color: #dc0e0e;");
    socketStatus = true;
}

void ExperDemoSystem::onError() {
    ui.RemoteStatus->setText("Error: ");
}

void ExperDemoSystem::onDisconnected() {
    ui.RemoteStatus->setText("Disconnected from server");
    ui.RemoteConnect->setStyleSheet("background-color: rgb(230,230,230);");
    socketStatus = false;
}

void ExperDemoSystem::on_SendButton_clicked() {
    if (m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
        QString message = ui.messageSend->text();
        QByteArray data = message.toUtf8();
        m_socket->write(data);
        m_socket->flush();
    }
    else
    {
        ui.RemoteStatus->setText("Server Disconnected");
    }
}

void ExperDemoSystem::on_SendButton_2_clicked()
{
	if (m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
		QString message = "0";
		QByteArray data = message.toUtf8();
		m_socket->write(data);
		m_socket->flush(); // 强制将数据写入到网络中
	}
	else
	{
		ui.RemoteStatus->setText("Server Disconnected");
	}
}

void ExperDemoSystem::on_SendButton_3_clicked()
{
	if (m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
		QString message = ui.messageSend->text();
		QByteArray data = "1";
		m_socket->write(data);
		m_socket->flush(); // 强制将数据写入到网络中
	}
	else
	{
		ui.RemoteStatus->setText("Server Disconnected");
	}
}

void ExperDemoSystem::on_RemoteExpBegin_clicked()
{
	if (m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
		QString message = ui.messageSend->text();
		QByteArray data = "0";
		m_socket->write(data);
		m_socket->flush(); // 强制将数据写入到网络中
	}
	else
	{
		ui.RemoteStatus->setText("Server Disconnected");
	}
}

void ExperDemoSystem::on_Remote_Left_clicked()
{
	if (m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
		QString message = ui.messageSend->text();
		QByteArray data = "1";
		m_socket->write(data);
		m_socket->flush(); // 强制将数据写入到网络中
	}
	else
	{
		ui.RemoteStatus->setText("Server Disconnected");
	}
}

void ExperDemoSystem::on_Remote_Right_clicked()
{
	if (m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
		QString message = ui.messageSend->text();
		QByteArray data = "2";
		m_socket->write(data);
		m_socket->flush(); // 强制将数据写入到网络中
	}
	else
	{
		ui.RemoteStatus->setText("Server Disconnected");
	}
}

/* 储存模块控件 */
void ExperDemoSystem::on_SaveData_clicked() {
    auto button = ui.SaveData;

    //只有当实验开始的时候才能进行数据记录
    if (ui.ExpBegin->text() == "Begin Experiment") return;

    if (button->text() == "Save") {
        button->setText("Saving ...");
        button->setStyleSheet("background-color: #dc0e0e;");

        //更改存储路径
        if (!ui.Path->text().isEmpty()) {
            QByteArray byteArray = ui.Path->text().toUtf8();
            const char* cstring = byteArray.data();
            strcpy(exp->dirname, cstring);
        }

        //更改文件名字
        if (!ui.dataPath->text().isEmpty()) {
            QString dataName = ui.dataPath->text() + QString::number(ui.expNum->value());
            QByteArray byteArray = dataName.toUtf8();
            const char* cstring = byteArray.data();
            strcpy(exp->outfname, cstring);
        }

        emit ui.OriginPoint->clicked();  //重置标准坐标用于记录
        exp->e = SetupRecording(exp);
        exp->Params->Record = 1;
    }
    else {
        button->setText("Save");
        button->setStyleSheet("background-color: rgb(85, 170, 0);");

        FinishRecording(exp);
    }
}

void ExperDemoSystem::on_ChoseFolder_clicked() {
    QString folderPath = QFileDialog::getExistingDirectory(this, "Select Folder", "D:/Liu/ExpRecordTmp");
    if (!folderPath.isEmpty()) {
        ui.Path->setText(folderPath + "/");
    }
}

//重载键盘触发事件 - 将按键与PushButton连接
void ExperDemoSystem::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
    case Qt::Key_Up:
        emit ui.Up->clicked();
        event->accept();
        break;
    case Qt::Key_Down:
        emit ui.Down->clicked();
        event->accept();
        break;
    case Qt::Key_Left:
        emit ui.Left->clicked();
        event->accept();
        break;
    case Qt::Key_Right:
        emit ui.Right->clicked();
        event->accept();
        break;
    default:
        QWidget::keyPressEvent(event); // 处理其他按键事件
        break;
    }
}

//连接SpinBox完成信号
void ExperDemoSystem::m_SpinBoxFinished() {

    /* 图像显示参数控件 */
    connect(ui.Diameter, &QSpinBox::editingFinished, this, &ExperDemoSystem::m_Diameter_valueChanged);
    connect(ui.UpBoundary, &QSpinBox::editingFinished, this, &ExperDemoSystem::m_UpBoundary_valueChanged);
    connect(ui.RectHeight, &QSpinBox::editingFinished, this, &ExperDemoSystem::m_RectHeight_valueChanged);
    connect(ui.BinThreshBox, &QSpinBox::editingFinished, this, &ExperDemoSystem::m_BinThreshBox_valueChanged);
    connect(ui.ErodesThresh, &QSpinBox::editingFinished, this, &ExperDemoSystem::m_ErodesThresh_valueChanged);

    /* 位移台参数控件 */
    connect(ui.StageSpeed, &QSpinBox::editingFinished, this, &ExperDemoSystem::m_StageSpeed_valueChanged);
    connect(ui.radioAxisFree, &QRadioButton::toggled, this, &ExperDemoSystem::on_axisRadioButton_toggled);
    connect(ui.radioLockX, &QRadioButton::toggled, this, &ExperDemoSystem::on_axisRadioButton_toggled);
    connect(ui.radioLockY, &QRadioButton::toggled, this, &ExperDemoSystem::on_axisRadioButton_toggled);

    /* 光电控制模块控件 */
    connect(ui.LedSpinBox, &QSpinBox::editingFinished, this, &ExperDemoSystem::m_LedSpinBox_valueChanged);
}

//用于连续触发按键，连接按键信号和槽函数
void ExperDemoSystem::KeyTimerConnect() {
    // 初始化定时器
    m_upButtonTimer = new QTimer(this);
    m_downButtonTimer = new QTimer(this);
    m_leftButtonTimer = new QTimer(this);
    m_rightButtonTimer = new QTimer(this);

    connect(m_upButtonTimer, &QTimer::timeout, this, &ExperDemoSystem::onUpButtonTimeout);
    connect(m_downButtonTimer, &QTimer::timeout, this, &ExperDemoSystem::onDownButtonTimeout);
    connect(m_leftButtonTimer, &QTimer::timeout, this, &ExperDemoSystem::onLeftButtonTimeout);
    connect(m_rightButtonTimer, &QTimer::timeout, this, &ExperDemoSystem::onRightButtonTimeout);

    // 连接按钮的按下和释放信号到相应的槽函数
    connect(ui.Up, &QPushButton::pressed, this, &ExperDemoSystem::onUpButtonPressed);
    connect(ui.Up, &QPushButton::released, this, &ExperDemoSystem::onUpButtonReleased);
    connect(ui.Down, &QPushButton::pressed, this, &ExperDemoSystem::onDownButtonPressed);
    connect(ui.Down, &QPushButton::released, this, &ExperDemoSystem::onDownButtonReleased);
    connect(ui.Left, &QPushButton::pressed, this, &ExperDemoSystem::onLeftButtonPressed);
    connect(ui.Left, &QPushButton::released, this, &ExperDemoSystem::onLeftButtonReleased);
    connect(ui.Right, &QPushButton::pressed, this, &ExperDemoSystem::onRightButtonPressed);
    connect(ui.Right, &QPushButton::released, this, &ExperDemoSystem::onRightButtonReleased);
}

void ExperDemoSystem::onUpButtonPressed() {
    m_upButtonTimer->start(100); // 启动定时器，间隔100毫秒
}

void ExperDemoSystem::onUpButtonReleased() {
    m_upButtonTimer->stop(); // 停止定时器
}

void ExperDemoSystem::onUpButtonTimeout() {
    on_Up_clicked(); // 定时器超时后调用按钮的点击槽函数
}

void ExperDemoSystem::onDownButtonPressed() {
    m_downButtonTimer->start(100); // 启动定时器，间隔100毫秒
}

void ExperDemoSystem::onDownButtonReleased() {
    m_downButtonTimer->stop(); // 停止定时器
}

void ExperDemoSystem::onDownButtonTimeout() {
    on_Down_clicked(); // 定时器超时后调用按钮的点击槽函数
}

void ExperDemoSystem::onLeftButtonPressed() {
    m_leftButtonTimer->start(100); // 启动定时器，间隔100毫秒
}

void ExperDemoSystem::onLeftButtonReleased() {
    m_leftButtonTimer->stop(); // 停止定时器
}

void ExperDemoSystem::onLeftButtonTimeout() {
    on_Left_clicked(); // 定时器超时后调用按钮的点击槽函数
}

void ExperDemoSystem::onRightButtonPressed() {
    m_rightButtonTimer->start(100); // 启动定时器，间隔100毫秒
}

void ExperDemoSystem::onRightButtonReleased() {
    m_rightButtonTimer->stop(); // 停止定时器
}

void ExperDemoSystem::onRightButtonTimeout() {
    on_Right_clicked(); // 定时器超时后调用按钮的点击槽函数
}

