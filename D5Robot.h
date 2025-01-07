/**
 * @file D5Robot.h
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "CameraBot.h"
#include "CameraTop.h"
#include "KineHelper.hpp"
#include "NatorMotor.h"
#include "RMDMotor.h"
#include "SerialPort.h"
#include "halconcpp/HalconCpp.h"

#include "LogUtil.h"
#include "RobotException.hpp"
#include "Visual.h"
#include "VC.h"

namespace D5R {

    class D5Robot {
    private:
        SerialPort* _port; // ������������ RMDMotor ����֮ǰ

    public:
        NatorMotor* natorMotor;
        RMDMotor* topRMDMotor;
        RMDMotor* botRMDMotor;
        CameraTop* topCamera = nullptr;
        CameraBot* botCamera = nullptr;

        D5Robot();
        D5Robot(const char* serialPort, std::string natorID = NatorId,
            uint8_t topRMDID = 1, uint8_t botRMDID = 2,
            std::string topCameraID = TopCameraId);
        ~D5Robot();
        void InitNator(std::string natorID = NatorId);
        void InitRMD(const char* portName, uint8_t topRMDID = 1, uint8_t botRMDID = 2);
        void InitTopCamera(std::string topCameraId = TopCameraId);
        void InitBotCamera(std::string botCameraId = BotCameraId);
        void SetZero();
        void Stop();
        void JointsMoveAbsolute(const Joints j);
        void JointsMoveRelative(const Joints j);
        void TaskMoveAbsolute(const TaskSpace ts);
        void TaskMoveRelative(const TaskSpace ts);
        void VCJawChange();

        Joints GetCurrentJoint();
        TaskSpace GetCurrentPose();

    private:
        inline static const std::string NatorId = "usb:id:2250716012";
        inline static const std::string TopCameraId = "00-21-49-03-4D-95";
        inline static const std::string BotCameraId = "00-21-49-03-4D-94";
    };
} // namespace D5R
