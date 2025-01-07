/**
 * @file D5Robot.cpp
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "D5Robot.h"

namespace D5R {

	Joints JAWPOINT{ 0, 0, 7500000, 5000000, 0 }; // 钳口位置，需要实验确定

	D5Robot::D5Robot() {}

	D5Robot::D5Robot(
		const char* serialPort,
		std::string natorID,
		uint8_t topRMDID,
		uint8_t botRMDID,
		std::string topCameraID) {
		InitNator(natorID);
		InitRMD(serialPort, topRMDID, botRMDID);
		// InitCamera(topCameraID);
	}
	D5Robot::~D5Robot() {
		if (topCamera != nullptr) {
			delete topCamera;
			topCamera = nullptr;
		}
		if (botCamera != nullptr) {
			delete botCamera;
			botCamera = nullptr;
		}
	}

	void D5Robot::InitNator(std::string natorID) {
		natorMotor = new NatorMotor(natorID);
	}

	/**
	 * @brief Initializes the RMD motor with the given port name and IDs.
	 *
	 * This function sets up the RMD motor with the given port name and IDs.
	 * The IDs are used to identify the motor when sending commands.
	 *
	 * @param portName The serial port name to use for the motor.
	 * @param topRMDID The ID of the top RMD motor.
	 * @param botRMDID The ID of the bottom RMD motor.
	 */
	void D5Robot::InitRMD(const char* portName, uint8_t topRMDID, uint8_t botRMDID) {
		_port = new SerialPort(portName);
		topRMDMotor = new RMDMotor(_port->GetHandle(), topRMDID);
		botRMDMotor = new RMDMotor(_port->GetHandle(), botRMDID);
	}

	void D5Robot::InitTopCamera(std::string topCameraId) {
		topCamera = new CameraTop(topCameraId);
	}

	void D5Robot::InitBotCamera(std::string botCameraId) {
		botCamera = new CameraBot(botCameraId);
	}

	/**
	 * @brief 将当前位置设为零点
	 *
	 */
	void D5Robot::SetZero() {
		if (!natorMotor) {
			throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::SetZero: natorMotor is not initialized. Call InitNator first.");
		}
		if (!topRMDMotor || !botRMDMotor) {
			throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::SetZero: RMDMotor is not initialized. Call InitRMD first.");
		}

		natorMotor->SetZero();
		topRMDMotor->SetZero();
		botRMDMotor->SetZero();
	}

	/**
	 * @brief 电机下电
	 *
	 */
	void D5Robot::Stop() {
		if (!natorMotor) {
			throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::Stop: natorMotor is not initialized. Call `InitNator` first.");
		}
		if (!topRMDMotor || !botRMDMotor) {
			throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::Stop: RMDMotor is not initialized. Call `InitRMD` first.");
		}

		natorMotor->Stop();
		topRMDMotor->Stop();
		botRMDMotor->Stop();
	}

	/**
	 * @brief D5R绝对移动
	 *
	 * @param j 目标关节位置
	 */
	void D5Robot::JointsMoveAbsolute(const Joints j) {
		if (!natorMotor) {
			throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::JointsMoveAbsolute: natorMotor is not initialized. Call `InitNator` first.");
		}
		if (!topRMDMotor || !botRMDMotor) {
			throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::JointsMoveAbsolute: RMDMotor is not initialized. Call `InitRMD` first.");
		}

		NTU_Point p{ j.p2, j.p3, j.p4 };
		natorMotor->GoToPoint_A(p);
		topRMDMotor->GoAngleAbsolute(j.r1);
		botRMDMotor->GoAngleAbsolute(j.r5);
	}

	/**
	 * @brief D5R相对移动
	 *
	 * @param j 目标关节距离
	 */
	void D5Robot::JointsMoveRelative(const Joints j) {
		if (!natorMotor) {
			throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::JointsMoveRelative: natorMotor is not initialized. Call `InitNator` first.");
		}
		if (!topRMDMotor || !botRMDMotor) {
			throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::JointsMoveRelative: RMDMotor is not initialized. Call `InitRMD` first.");
		}

		NTU_Point p{ j.p2, j.p3, j.p4 };
		natorMotor->GoToPoint_R(p);
		topRMDMotor->GoAngleRelative(j.r1);
		botRMDMotor->GoAngleRelative(j.r5);
	}

	/**
	 * @brief 任务空间绝对位移
	 *
	 * @param ts 任务空间目标
	 */
	void D5Robot::TaskMoveAbsolute(const TaskSpace ts) {
		JointSpace js = KineHelper::Inverse(ts);
		JointsMoveAbsolute(js.ToControlJoint());
	}

	/**
	 * @brief 任务空间相对位移
	 *
	 * @param ts 任务空间距离
	 */
	void D5Robot::TaskMoveRelative(const TaskSpace ts) {
		auto currentPose = GetCurrentPose();
		JointSpace deltaJoint = KineHelper::InverseDifferential(ts, currentPose);
		JointsMoveRelative(deltaJoint.ToControlJoint());
	}

	/**
	 * @brief 获取当前关节位置
	 *
	 * @return Joints
	 */
	Joints D5Robot::GetCurrentJoint() {
		if (!natorMotor) {
			throw RobotException(ErrorCode::D5RNatorNotInitialized, "In D5Robot::GetCurrentJoint: natorMotor is not initialized. Call `InitNator` first.");
		}
		if (!topRMDMotor || !botRMDMotor) {
			throw RobotException(ErrorCode::D5RRMDMotorNotInitialized, "In D5Robot::GetCurrentJoint: RMDMotor is not initialized. Call `InitRMD` first.");
		}

		Joints j;

		j.r1 = topRMDMotor->GetSingleAngle_s();
		j.r5 = botRMDMotor->GetSingleAngle_s();

		NTU_Point np = this->natorMotor->GetPosition();
		j.p2 = np.x;
		j.p3 = np.y;
		j.p4 = np.z;

		return j;
	}

	/**
	 * @brief 获取当前任务空间位置
	 *
	 * @return TaskSpace
	 */
	TaskSpace D5Robot::GetCurrentPose() {
		auto joint = GetCurrentJoint();
		JointSpace js(joint);
		auto ts = KineHelper::Forward(js);
		return ts;
	}

	/**
	 * @brief 钳口视觉伺服控制
	 *
	 */
	void D5Robot::VCJawChange() {
		if (!topCamera) {
			throw RobotException(ErrorCode::D5RCameraNotInitialized, "topCamera is not initialized");
		}
		if (!botCamera) {
			throw RobotException(ErrorCode::D5RCameraNotInitialized, "botCamera is not initialized");
		}
		// 设置窗口  测试时使用
		cv::namedWindow("test", cv::WINDOW_NORMAL);
		cv::resizeWindow("test", cv::Size(1300, 1000));

		// 初始位置
		JointsMoveAbsolute(JAWPOINT);
		Sleep(2000);

		VC vc;
		HalconCpp::HObject ho_search_ROI_DL, ho_search_ROI_DR, ho_ROI_DL, ho_ROI_DR;
		HalconCpp::HTuple hv_start, hv_range, hv_Height_DT, hv_Width_DT, hv_Height_DS, hv_Width_DS;

		//参数设置
		hv_start = -0.087;
		hv_range = 0.174;

		//模板size
		hv_Height_DT = 200;
		hv_Width_DT = 50;

		//搜索size
		hv_Height_DS = 300;
		hv_Width_DS = 150;

		// 获取图片
		cv::Mat img;
		topCamera->Read(img);
		HalconCpp::HObject ho_img = vc.Mat2HImage(img);

		// 第一次匹配钳口
		HalconCpp::HObject ho_init_search_rect, ho_ImageReduced;
		HalconCpp::HTuple hv_Row_DL, hv_Col_DL, hv_Angle_DL, hv_Score_DL;
		HalconCpp::HTuple hv_Row_DR, hv_Col_DR, hv_Angle_DR, hv_Score_DR;
		HalconCpp::GenRectangle1(&ho_init_search_rect, 600, 900, 1500, 1500);
		HalconCpp::ReduceDomain(ho_img, ho_init_search_rect, &ho_ImageReduced);
		HalconCpp::FindShapeModel(ho_ImageReduced, vc.GetJaw().temp_dl, hv_start, hv_range, 0.8, 1, 0.5,
			(HalconCpp::HTuple("least_squares").Append("max_deformation 2")), 0, 0.9, &hv_Row_DL,
			&hv_Col_DL, &hv_Angle_DL, &hv_Score_DL);
		HalconCpp::FindShapeModel(ho_ImageReduced, vc.GetJaw().temp_dr, hv_start, hv_range, 0.8, 1, 0.5,
			(HalconCpp::HTuple("least_squares").Append("max_deformation 2")), 0, 0.9, &hv_Row_DR,
			&hv_Col_DR, &hv_Angle_DR, &hv_Score_DR);

		// 定义局部变量以保存数据
		HalconCpp::HTuple hv_Last_Row_DL = hv_Row_DL;
		HalconCpp::HTuple hv_Last_Row_DR = hv_Row_DR;
		HalconCpp::HTuple hv_Last_Col_DL = hv_Col_DL;
		HalconCpp::HTuple hv_Last_Col_DR = hv_Col_DR;
		HalconCpp::HTuple hv_Last_Angle_DL = hv_Angle_DL;
		HalconCpp::HTuple hv_Last_Angle_DR = hv_Angle_DR;

		HalconCpp::HTuple hv_Angle = (hv_Last_Angle_DR + hv_Last_Angle_DL) / 2;
		HalconCpp::HTuple hv_Row = (hv_Last_Row_DL + hv_Last_Row_DR) * 0.5 - 150;
		HalconCpp::HTuple hv_Col = (hv_Last_Col_DL + hv_Last_Col_DR) * 0.5;

		// 获取对应钳口库ROI区域
		vc.JawLibSegmentation(img, 2);
		// 获取夹钳位置信息
		std::vector<cv::Point2f> clampPos = vc.SIFT(img, Models::CLAMP);
		float clampAngle = static_cast<float>(atan2f(clampPos[0].y - clampPos[1].y, clampPos[0].x - clampPos[1].x) * (-180) / CV_PI);
		// 粗对准
		TaskSpace pError{ (clampPos[0].y - vc.GetROIPos().y - vc.GetRoughPosPoint().y) * vc.GetMapParam(),
						  (clampPos[0].x - vc.GetROIPos().x - vc.GetRoughPosPoint().x) * vc.GetMapParam(),
						  0, 0,
						  (clampAngle - hv_Angle.D() * 180 / CV_PI - 90) };
		JointSpace jError{};
		while (abs(pError.Px) > 0.1 || abs(pError.Py) > 0.1 || abs(pError.Rz) > 0.1) {
			// 测试时的显示
			cv::imshow("test", img);
			cv::waitKey(0);

			pError.Px = 0.4 * pError.Px;
			pError.Rz = -0.5 * pError.Rz;
			pError.Py = 0.4 * pError.Py;
			jError = KineHelper::InverseDifferential(pError, GetCurrentPose());
			JointsMoveRelative(jError.ToControlJoint());
			Sleep(500);
			topCamera->Read(img);
			clampPos = vc.SIFT(img, Models::CLAMP);
			clampAngle = static_cast<float>(atan2f(clampPos[0].y - clampPos[1].y, clampPos[0].x - clampPos[1].x) * (-180) / CV_PI);
			pError = { (clampPos[0].y - vc.GetROIPos().y - vc.GetRoughPosPoint().y) * vc.GetMapParam(),
					   (clampPos[0].x - vc.GetROIPos().x - vc.GetRoughPosPoint().x) * vc.GetMapParam(),
						0, 0,
					   (clampAngle - hv_Angle.D() * 180 / CV_PI - 90) };
		}

		cv::Mat img_2;
		botCamera->Read(img_2);
		// 更新水平线
		vc.GetHorizontalLine(img_2, 1);
		// 下移
		double h = vc.GetVerticalDistance(img_2, 1);
		JointsMoveRelative({ 0, 0, 0, int(-h * 1000000), 0 });
		Sleep(1000);

		// 插入
		// 定义后退标志 
		int flag = 0;
		pError = { (clampPos[0].y - hv_Row.D()) * vc.GetMapParam(),
				   (clampPos[0].x - hv_Col.D()) * vc.GetMapParam(),
					0, 0,
				   (clampAngle - hv_Angle.D() * 180 / CV_PI - 90) };
		while (abs(pError.Px) > 0.1 || abs(pError.Py) > 0.1 || abs(pError.Rz) > 0.1) {
			// 测试时的显示
			cv::imshow("test", img);
			cv::waitKey(0);

			if (flag != 2) {
				pError.Px = 0.2 * pError.Px;
				pError.Rz = -0.3 * pError.Rz;
				pError.Py = 0.2 * pError.Py;
				jError = KineHelper::InverseDifferential(pError, GetCurrentPose());
				JointsMoveRelative({ (int)jError.R1 * 100,(int)jError.P2 * 1000000, 0,0,0 });

				const int stepLength = 1000; // nm
				int n = pError.Px * 1000000 / stepLength;
				for (int i = 0; i < n; ++i) {
					NT_GotoPositionRelative_S(natorMotor->GetHandle(), NTU_AXIS_X, stepLength, 0);
					Sleep(20);
				}
				Sleep(100);
			}
			else {
				JointsMoveRelative({ 0,0,-500000,0,0 });
				Sleep(200);
			}

			topCamera->Read(img);
			ho_img = vc.Mat2HImage(img);
			flag = 0;
			// 定位钳口
			GenRectangle2(&ho_search_ROI_DL, hv_Last_Row_DL, hv_Last_Col_DL, hv_Last_Angle_DL,
				hv_Width_DS / 2, hv_Height_DS / 2);
			GenRectangle2(&ho_search_ROI_DR, hv_Last_Row_DR, hv_Last_Col_DR, hv_Last_Angle_DR,
				hv_Width_DS / 2, hv_Height_DS / 2);
			ReduceDomain(ho_img, ho_search_ROI_DL, &ho_ROI_DL);
			ReduceDomain(ho_img, ho_search_ROI_DR, &ho_ROI_DR);
			FindShapeModel(ho_ROI_DL, vc.GetJaw().temp_dl, hv_start, hv_range, 0.7, 1, 0.5, (HalconCpp::HTuple("least_squares").Append("max_deformation 2")),
				0, 0.9, &hv_Row_DL, &hv_Col_DL, &hv_Angle_DL, &hv_Score_DL);
			FindShapeModel(ho_ROI_DR, vc.GetJaw().temp_dr, hv_start, hv_range, 0.7, 1, 0.5, (HalconCpp::HTuple("least_squares").Append("max_deformation 2")),
				0, 0.9, &hv_Row_DR, &hv_Col_DR, &hv_Angle_DR, &hv_Score_DR);
			if (hv_Score_DL.D() >= 0.7)
			{
				hv_Last_Row_DL = hv_Row_DL;
				hv_Last_Col_DL = hv_Col_DL;
				hv_Last_Angle_DL = hv_Angle_DL;
			}
			else
			{
				ReduceDomain(ho_img, ho_init_search_rect, &ho_ImageReduced);
				FindShapeModel(ho_ImageReduced, vc.GetJaw().temp_dl, hv_start, hv_range, 0.7, 1, 0.5,
					(HalconCpp::HTuple("least_squares").Append("max_deformation 2")), 0, 0.9, &hv_Row_DL,
					&hv_Col_DL, &hv_Angle_DL, &hv_Score_DL);
				if (hv_Score_DL.D() >= 0.7)
				{
					hv_Last_Row_DL = hv_Row_DL;
					hv_Last_Col_DL = hv_Col_DL;
					hv_Last_Angle_DL = hv_Angle_DL;
				}
				else {
					flag++;
				}
			}
			if (hv_Score_DR.D() >= 0.7)
			{
				hv_Last_Row_DR = hv_Row_DR;
				hv_Last_Col_DR = hv_Col_DR;
				hv_Last_Angle_DR = hv_Angle_DR;
			}
			else
			{
				ReduceDomain(ho_img, ho_init_search_rect, &ho_ImageReduced);
				FindShapeModel(ho_ImageReduced, vc.GetJaw().temp_dr, hv_start, hv_range, 0.7, 1, 0.5,
					(HalconCpp::HTuple("least_squares").Append("max_deformation 2")), 0, 0.9, &hv_Row_DR,
					&hv_Col_DR, &hv_Angle_DR, &hv_Score_DR);
				if (hv_Score_DR.D() >= 0.7)
				{
					hv_Last_Row_DR = hv_Row_DR;
					hv_Last_Col_DR = hv_Col_DR;
					hv_Last_Angle_DR = hv_Angle_DR;
				}
				else {
					flag++;
				}
			}
			hv_Angle = (hv_Last_Angle_DR + hv_Last_Angle_DL) / 2;
			hv_Row = (hv_Last_Row_DL + hv_Last_Row_DR) * 0.5 - 150;
			hv_Col = (hv_Last_Col_DL + hv_Last_Col_DR) * 0.5;
			// 定位夹钳
			clampPos = vc.SIFT(img, Models::CLAMP);
			clampAngle = static_cast<float>(atan2f(clampPos[0].y -
				clampPos[1].y, clampPos[0].x - clampPos[1].x) * (-180) / CV_PI);
			pError = { (clampPos[0].y - hv_Row.D()) * vc.GetMapParam(),
					   (clampPos[0].x - hv_Col.D()) * vc.GetMapParam(),
						0, 0,
					   (clampAngle - hv_Angle.D() * 180 / CV_PI - 90) };
		}

		//上抬
		JointsMoveRelative({ 0, 0, 0, -2500000, 0 });
		Sleep(100);
	}

} // namespace D5R