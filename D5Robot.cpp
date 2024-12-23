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
#include <stdlib.h>
#include <string.h>
#include <msclr/marshal.h>

namespace D5R {

    Joints JAWPOINT{ 0, 0, 7500000, 5000000, 0 }; // ǯ��λ�ã���Ҫʵ��ȷ��

    D5Robot::D5Robot() {}

    D5Robot::D5Robot(const char* serialPort, String^ natorID,
        uint8_t topRMDID, uint8_t botRMDID) {
        throw gcnew System::NotImplementedException();
    }

    //D5Robot::D5Robot(
    //    const char* serialPort,
    //    const String^ natorID,
    //    uint8_t topRMDID,
    //    uint8_t botRMDID) {
    //    InitNator(natorID);
    //    InitRMD(serialPort, topRMDID, botRMDID);
    //    // InitCamera(topCameraID);
    //}
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

    void D5Robot::InitNator(String^ natorID) {
        using namespace msclr::interop;
        
        marshal_context^ context = gcnew marshal_context();
        String^ from = natorID;
        const char* str = context->marshal_as<const char*>(natorID);
        this->natorMotor = new NatorMotor(std::string(str));
        delete context;
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

    void D5Robot::InitTopCamera(String^ topCameraId) {
		using namespace msclr::interop;

        marshal_context^ context = gcnew marshal_context();
		auto str = context->marshal_as<const char*>(topCameraId);
        topCamera = new CameraTop(std::string(str));
        delete context;
    }

    void D5Robot::InitBotCamera(String^ botCameraId) {
        using namespace msclr::interop;

        marshal_context^ context = gcnew marshal_context();
        auto str = context->marshal_as<const char*>(botCameraId);
        botCamera = new CameraBot(std::string(str));
        delete context;
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
        // ��ʼλ��
        JointsMoveAbsolute(JAWPOINT);
        Sleep(2000);
        // ��ȡROI����
        cv::Mat img;
        topCamera->Read(img);
        topCamera->GetROI(img);
        // ��ȡһ��ǯ��λ����Ϣ
        std::vector<cv::Point2f> jawPos;
        topCamera->SIFT(img, JAW, jawPos);
        float jawAngle = static_cast<float>(atan2f(jawPos[1].y - jawPos[0].y, jawPos[1].x - jawPos[0].x) * (-180) / CV_PI);
        // �ֶ�׼
        std::vector<double> posError = topCamera->GetPhysicError(Rough, jawAngle);
        TaskSpace pError{ posError[1], posError[0], 0, 0, posError[2] };
        JointSpace jError{};
        while (abs(pError.Px) > 0.1 || abs(pError.Py) > 0.1 || abs(pError.Rz) > 0.1) {
            pError.Px = 0.4 * pError.Px;
            pError.Rz = -0.5 * pError.Rz;
            pError.Py = 0.4 * pError.Py;
            jError = KineHelper::InverseDifferential(pError, GetCurrentPose());
            JointsMoveRelative(jError.ToControlJoint());
            Sleep(500);
            posError.clear();
            posError = topCamera->GetPhysicError(Rough, jawAngle);
            pError = { posError[1], posError[0], 0, 0, posError[2] };
        }

        // ����
        // JointsMoveRelative({0, 0, 0, 2600000, 0}); // ������~
        cv::Mat img_2;
        botCamera->Read(img_2);
        double h = botCamera->GetDistance(img_2);
        JointsMoveRelative({ 0, 0, 0, int(-h * 1000000), 0 });
        Sleep(1000);

        // // ����
        posError.clear();
        posError = topCamera->GetPhysicError(Fine);
        pError = { posError[1], posError[0], 0, 0, 0 };
        while (abs(pError.Px) > 0.1 || abs(pError.Py) > 0.1) {
            if (abs(posError[2]) > 20) {
                JointsMoveRelative({ 0, 0, -1000000, 0, 0 });
                Sleep(500);
                posError.clear();
                posError = topCamera->GetPhysicError(Fine);
                pError = { posError[1], posError[0], 0, 0, -0.5 * posError[2] };
                continue;
            }
            pError.Px = 0.25 * pError.Px;
            pError.Py = 0.4 * pError.Py;
            jError = KineHelper::InverseDifferential(pError, GetCurrentPose());
            JointsMoveRelative(jError.ToControlJoint());
            Sleep(500);
            posError.clear();
            try {
                posError = topCamera->GetPhysicError(Fine);
            }
            catch (RobotException err) {
                JointsMoveRelative({ 0, 0, -1000000, 0, 0 });
                Sleep(500);
                posError = topCamera->GetPhysicError(Fine);
            }
            pError = { posError[1], posError[0], 0, 0, 0 };
        }

        // // ��̧
        JointsMoveRelative({ 0, 0, 0, -2500000, 0 });
    }

} // namespace D5R