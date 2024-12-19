#include "pch.h"
#include "D5RAPI.h"

#define MAJOR_VERSION 0
#define MINOR_VERSION 1
#define PATCH_VERSION 0

#define TRY_BLOCK(content)            \
  try {                               \
    content return ErrorCode::OK;     \
  } catch (const RobotException& e) { \
    return e.code;                    \
  } catch (...) {                     \
    return ErrorCode::SystemError;    \
  }

static RobotException lastError = RobotException(ErrorCode::OK);

ErrorCode D5R_GetLastError() {
  return lastError.code;
}

BSTR D5R_GetVersion() {
  static std::string version = std::to_string(MAJOR_VERSION) + "." +
                               std::to_string(MINOR_VERSION) + "." +
                               std::to_string(PATCH_VERSION);

  return _com_util::ConvertStringToBSTR(version.c_str());
}

ErrorCode CreateD5RobotInstance(D5Robot*& instance, const char* serialPort) {
  try {
    instance = new D5Robot(serialPort);
    return ErrorCode::OK;
  } catch (const RobotException& e) {
    return e.code;
  } catch (...) {
    return ErrorCode::CreateInstanceError;
  }
}

ErrorCode CreateD5RobotInstance2(D5Robot*& instance,
                                 const char* serialPort,
                                 const char* natorID,
                                 uint8_t topRMDID,
                                 uint8_t bottomRMDID,
                                 const char* upCameraID) {
  try {
    instance =
        new D5Robot(serialPort, natorID, topRMDID, bottomRMDID, upCameraID);
    return ErrorCode::OK;
  } catch (const RobotException& e) {
    return e.code;
  } catch (...) {
    return ErrorCode::CreateInstanceError;
  }
}

D5Robot* CreateD5RobotInstance3(const char* serialPort,
                               const char* natorID,
                               uint8_t topRMDID,
                               uint8_t bottomRMDID) {
  try {
    return new D5Robot(serialPort, natorID, topRMDID, bottomRMDID);
    // return ErrorCode::OK;
  } catch (const RobotException& e) {
    lastError = e;
    return nullptr;
    // return e.code;
  } catch (...) {
    lastError = RobotException(ErrorCode::CreateInstanceError);
    return nullptr;
    // return ErrorCode::CreateInstanceError;
  }
}

ErrorCode DestroyD5RobotInstance(D5Robot* instance) {
  // delete instance;
  // return ErrorCode::OK;
  // TRY_BLOCK(delete instance;)
  if (instance == nullptr)
    return ErrorCode::DestroyInstanceError_nullptr;
  try {
    delete instance;
    return ErrorCode::OK;
  } catch (const RobotException& e) {
    return e.code;
  } catch (...) {
    return ErrorCode::CreateInstanceError;
  }
}

ErrorCode CallSetZero(D5Robot* instance) {
  TRY_BLOCK(instance->SetZero();)
}

ErrorCode CallStop(D5Robot* instance) {
  TRY_BLOCK(instance->Stop();)
}

ErrorCode CallJointsMoveAbsolute(D5Robot* instance, const Joints j) {
  TRY_BLOCK(instance->JointsMoveAbsolute(j);)
}

ErrorCode CallJointsMoveRelative(D5Robot* instance, const Joints j) {
  TRY_BLOCK(instance->JointsMoveRelative(j);)
}

ErrorCode CallTaskMoveAbsolute(D5Robot* instance, const TaskSpace ts) {
  try {
    instance->TaskMoveAbsolute(ts);
    return ErrorCode::OK;
  } catch (const RobotException& e) {
    return e.code;
  }
}

ErrorCode CallTaskMoveRelative(D5Robot* instance, const TaskSpace ts) {
  try {
    instance->TaskMoveRelative(ts);
    return ErrorCode::OK;
  } catch (const RobotException& e) {
    return e.code;
  }
}