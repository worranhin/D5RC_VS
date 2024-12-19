#pragma once

#include "pch.h"
#include "D5Robot.h"
#include "ErrorCode.h"
#include "RobotException.hpp"
#include <comdef.h>
#include <comutil.h>

#ifdef D5R_EXPORTS
#define D5R_API __declspec(dllexport)
#else
#define D5R_API __declspec(dllimport)
#endif

using namespace D5R;

extern "C" {
D5R_API ErrorCode D5R_GetLastError();
D5R_API BSTR D5R_GetVersion();
D5R_API ErrorCode CreateD5RobotInstance(D5Robot*& instance,
                                        const char* serialPort);
D5R_API ErrorCode CreateD5RobotInstance2(D5Robot*& instance,
                                         const char* serialPort,
                                         const char* natorID,
                                         uint8_t topRMDID,
                                         uint8_t bottomRMDID,
                                         const char* upCameraID);
D5R_API ErrorCode DestroyD5RobotInstance(D5Robot* instance);
D5R_API ErrorCode CallSetZero(D5Robot* instance);
D5R_API ErrorCode CallStop(D5Robot* instance);
D5R_API ErrorCode CallJointsMoveAbsolute(D5Robot* instance, const Joints j);
D5R_API ErrorCode CallJointsMoveRelative(D5Robot* instance, const Joints j);
D5R_API ErrorCode CallTaskMoveAbsolute(D5Robot* instance, const TaskSpace ts);
D5R_API ErrorCode CallTaskMoveRelative(D5Robot* instance, const TaskSpace ts);
}