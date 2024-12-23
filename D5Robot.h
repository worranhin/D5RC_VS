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

#include "LogUtil.h"
#include "RobotException.hpp"

using namespace System;

namespace D5R {

    public ref class D5Robot {
    private:
        SerialPort* _port; // 该声明必须在 RMDMotor 声明之前

    public:
        NatorMotor* natorMotor;
        RMDMotor* topRMDMotor;
        RMDMotor* botRMDMotor;
        CameraTop* topCamera = nullptr;
        CameraBot* botCamera = nullptr;

        D5Robot();
        D5Robot(const char* serialPort, String^ natorID,
            uint8_t topRMDID, uint8_t botRMDID);
        ~D5Robot();
        void InitNator(String^ natorID);
        void InitNator() {
            InitNator(_natorId);
        }

        void InitRMD(const char* portName, uint8_t topRMDID, uint8_t botRMDID);
        void InitRMD(const char* portName) {
            InitRMD(portName, 1, 2);
        }

        void InitTopCamera(String^ topCameraId);
        void InitTopCamera() {
			InitTopCamera(_topCameraId);
        }

        void InitBotCamera(String^ botCameraId);
        void InitBotCamera() {
            InitBotCamera(_bottomCameraId);
        }

        void SetZero();
        void Stop();
        void JointsMoveAbsolute(const Joints j);
        void JointsMoveRelative(const Joints j);
        void TaskMoveAbsolute(const TaskSpace ts);
        void TaskMoveRelative(const TaskSpace ts);
        void VCJawChange();

        Joints GetCurrentJoint();
        TaskSpace GetCurrentPose();

    public:
        static String^ _natorId = gcnew String("usb:id:2250716012");
        static String^ _topCameraId = gcnew String("00-21-49-03-4D-95");
        static String^ _bottomCameraId = gcnew String ("00-21-49-03-4D-94");
    };
} // namespace D5R
