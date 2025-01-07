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

	Joints JAWPOINT{ 0, 0, 7500000, 5000000, 0 }; // ǯ��λ�ã���Ҫʵ��ȷ��

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
	 * @brief ����ǰλ����Ϊ���
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
	 * @brief ����µ�
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
	 * @brief D5R�����ƶ�
	 *
	 * @param j Ŀ��ؽ�λ��
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
	 * @brief D5R����ƶ�
	 *
	 * @param j Ŀ��ؽھ���
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
	 * @brief ����ռ����λ��
	 *
	 * @param ts ����ռ�Ŀ��
	 */
	void D5Robot::TaskMoveAbsolute(const TaskSpace ts) {
		JointSpace js = KineHelper::Inverse(ts);
		JointsMoveAbsolute(js.ToControlJoint());
	}

	/**
	 * @brief ����ռ����λ��
	 *
	 * @param ts ����ռ����
	 */
	void D5Robot::TaskMoveRelative(const TaskSpace ts) {
		auto currentPose = GetCurrentPose();
		JointSpace deltaJoint = KineHelper::InverseDifferential(ts, currentPose);
		JointsMoveRelative(deltaJoint.ToControlJoint());
	}

	/**
	 * @brief ��ȡ��ǰ�ؽ�λ��
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
	 * @brief ��ȡ��ǰ����ռ�λ��
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
	 * @brief ǯ���Ӿ��ŷ�����
	 *
	 */
	void D5Robot::VCJawChange() {
		if (!topCamera) {
			throw RobotException(ErrorCode::D5RCameraNotInitialized, "topCamera is not initialized");
		}
		//if (!botCamera) {
		//	throw RobotException(ErrorCode::D5RCameraNotInitialized, "botCamera is not initialized");
		//}
		// ���ô���  ����ʱʹ��
		cv::namedWindow("test", cv::WINDOW_NORMAL);
		cv::resizeWindow("test", cv::Size(1300, 1000));

		// ��ʼλ��
		JointsMoveAbsolute(JAWPOINT);
		Sleep(2000);

		VC vc;
		HalconCpp::HObject ho_search_ROI_DL, ho_search_ROI_DR, ho_ROI_DL, ho_ROI_DR;
		HalconCpp::HTuple hv_start, hv_range, hv_Height_DT, hv_Width_DT, hv_Height_DS, hv_Width_DS;

		//��������
		hv_start = -0.131;
		hv_range = 0.262;

		//ģ��size
		hv_Height_DT = 200;
		hv_Width_DT = 50;

		//����size
		hv_Height_DS = 300;
		hv_Width_DS = 150;

		// ��ȡͼƬ
		cv::Mat img;
		topCamera->Read(img);
		HalconCpp::HObject ho_img = vc.Mat2HImage(img);

		// ��ȡ��Ӧǯ�ڿ�ROI����
		vc.JawLibSegmentation(img, 2);

		// ��һ��ƥ��ǯ��
		HalconCpp::HObject ho_init_search_rect, ho_ImageReduced;
		HalconCpp::HTuple hv_Row_DL, hv_Col_DL, hv_Angle_DL, hv_Score_DL;
		HalconCpp::HTuple hv_Row_DR, hv_Col_DR, hv_Angle_DR, hv_Score_DR;//_roiPos, cv::Size2f(850.0f, 2046.0f - _roiPos.y
		HalconCpp::GenRectangle1(&ho_init_search_rect, vc.GetROIPos().y, vc.GetROIPos().x, 1500, vc.GetROIPos().x + 850);
		HalconCpp::ReduceDomain(ho_img, ho_init_search_rect, &ho_ImageReduced);
		HalconCpp::FindShapeModel(ho_ImageReduced, vc.GetJaw().temp_dl, hv_start, hv_range, 0.8, 1, 0.5,
			(HalconCpp::HTuple("least_squares").Append("max_deformation 2")), 0, 0.9, &hv_Row_DL,
			&hv_Col_DL, &hv_Angle_DL, &hv_Score_DL);
		HalconCpp::FindShapeModel(ho_ImageReduced, vc.GetJaw().temp_dr, hv_start, hv_range, 0.8, 1, 0.5,
			(HalconCpp::HTuple("least_squares").Append("max_deformation 2")), 0, 0.9, &hv_Row_DR,
			&hv_Col_DR, &hv_Angle_DR, &hv_Score_DR);

		// ����ֲ������Ա�������
		HalconCpp::HTuple hv_Last_Row_DL = hv_Row_DL;
		HalconCpp::HTuple hv_Last_Row_DR = hv_Row_DR;
		HalconCpp::HTuple hv_Last_Col_DL = hv_Col_DL;
		HalconCpp::HTuple hv_Last_Col_DR = hv_Col_DR;
		HalconCpp::HTuple hv_Last_Angle_DL = hv_Angle_DL;
		HalconCpp::HTuple hv_Last_Angle_DR = hv_Angle_DR;

		HalconCpp::HTuple hv_Angle = (hv_Last_Angle_DR + hv_Last_Angle_DL) / 2;
		HalconCpp::HTuple hv_Row = (hv_Last_Row_DL + hv_Last_Row_DR) * 0.5 - 150;
		HalconCpp::HTuple hv_Col = hv_Last_Col_DL * 0.49 + hv_Last_Col_DR * 0.51;


		// ��ȡ��ǯλ����Ϣ
		std::vector<cv::Point2f> clampPos = vc.SIFT(img, Models::CLAMP);
		float clampAngle = static_cast<float>(atan2f(clampPos[0].y - clampPos[1].y, clampPos[0].x - clampPos[1].x) * (-180) / CV_PI);
		// �ֶ�׼
		TaskSpace pError{ (clampPos[0].y - vc.GetROIPos().y - vc.GetRoughPosPoint().y) * vc.GetMapParam(),
						  (clampPos[0].x - vc.GetROIPos().x - vc.GetRoughPosPoint().x) * vc.GetMapParam(),
						  0, 0,
						  (clampAngle - hv_Angle.D() * 180 / CV_PI - 90) };
		JointSpace jError{};
		while (abs(pError.Px) > 0.1 || abs(pError.Py) > 0.1 || abs(pError.Rz) > 0.1) {
			// ����ʶ��Ч��
			cv::Point2f h_1(hv_Col.D(), hv_Row.D());
			int h_2_x = static_cast<int>(h_1.x + 100 * cos(hv_Angle.D() + CV_PI / 2));
			int h_2_y = static_cast<int>(h_1.y + 100 * sin(hv_Angle.D() + CV_PI / 2));
			cv::line(img, h_1, cv::Point(h_2_x, h_2_y), cv::Scalar(0), 4);
			cv::line(img, clampPos[0], clampPos[1], cv::Scalar(0), 4);
			// ����ʱ����ʾ
			cv::imshow("test", img);
			cv::waitKey(0);

			pError.Px = 0.8 * pError.Px;
			pError.Rz = -0.6 * pError.Rz;
			pError.Py = 0.8 * pError.Py;
			int n = static_cast<int>(std::max(abs(pError.Px), abs(pError.Py)) / 0.01);
			jError = KineHelper::InverseDifferential(pError, GetCurrentPose());
			JointsMoveRelative(jError.ToControlJoint());
			Sleep(20 * n + 250);
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
		// ����ˮƽ��
		vc.GetHorizontalLine(img_2, 1);
		// ����
		double h = vc.GetVerticalDistance(img_2, 1);
		JointsMoveRelative({ 0, 0, 0, int(-h * 1000000), 0 });
		Sleep(2000);

		botCamera->Read(img_2);
		cv::imshow("test", img_2);
		cv::waitKey(0);
		//return;

		// ����
		// ������˱�־ 
		int flag = 0;
		pError = { (clampPos[0].y - hv_Row.D()) * vc.GetMapParam(),
				   (clampPos[0].x - hv_Col.D()) * vc.GetMapParam(),
					0, 0,
				   (clampAngle - hv_Angle.D() * 180 / CV_PI - 90) };
		while (abs(pError.Px) > 0.1 || abs(pError.Py) > 0.1 || abs(pError.Rz) > 0.1) {
			// ����ʶ��Ч��
			cv::Point2f h_1(hv_Col.D(), hv_Row.D());
			int h_2_x = static_cast<int>(h_1.x - 100 * cos(hv_Angle.D() + CV_PI / 2));
			int h_2_y = static_cast<int>(h_1.y - 100 * sin(hv_Angle.D() + CV_PI / 2));
			cv::line(img, h_1, cv::Point(h_2_x, h_2_y), cv::Scalar(0), 4);
			cv::line(img, clampPos[0], clampPos[1], cv::Scalar(0), 4);
			// ����ʱ����ʾ
			cv::imshow("test", img);
			cv::waitKey(0);

			if (flag != 2) {
				pError.Px = 0.4 * pError.Px;
				pError.Rz = -0.5 * pError.Rz;
				pError.Py = 0.8 * pError.Py;
				jError = KineHelper::InverseDifferential(pError, GetCurrentPose());
				JointsMoveRelative({ static_cast<int>(jError.R1 * 100),
					static_cast<int>(jError.P2 * 1000000), 0, 0, 0 });

				const int stepLength = 1000; // nm
				int n = jError.P3 * 1000000 / stepLength;
				for (int i = 0; i < n; ++i) {
					NT_GotoPositionRelative_S(natorMotor->GetHandle(), NTU_AXIS_Y, stepLength, 0);
					Sleep(20);
				}
				Sleep(200);
			}
			else {
				JointsMoveRelative({ 0,0,-500000,0,0 });
				Sleep(1000);
			}

			topCamera->Read(img);
			ho_img = vc.Mat2HImage(img);
			flag = 0;
			// ��λǯ��
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
			hv_Col = hv_Last_Col_DL * 0.45 + hv_Last_Col_DR * 0.55;
			// ��λ��ǯ
			clampPos = vc.SIFT(img, Models::CLAMP);
			clampAngle = static_cast<float>(atan2f(clampPos[0].y -
				clampPos[1].y, clampPos[0].x - clampPos[1].x) * (-180) / CV_PI);
			pError = { (clampPos[0].y - hv_Row.D()) * vc.GetMapParam(),
					   (clampPos[0].x - hv_Col.D()) * vc.GetMapParam(),
						0, 0,
					   (clampAngle - hv_Angle.D() * 180 / CV_PI - 90) };
		}

		//��̧
		JointsMoveRelative({ 0, 0, 0, -2500000, 0 });
		Sleep(100);
	}

} // namespace D5R