
#pragma once

#include "D5Robot.h"

using namespace System;
using namespace D5R;

namespace D5RWrapperClassLibrary {
public
ref class D5RobotWrapper {
  // TODO: 在此处为此类添加方法。
 public:
  D5RobotWrapper() : _instance(new D5Robot) {};
  ~D5RobotWrapper() { delete _instance; }
  void InitNator() {
	  InitNator(_natorId);
  }
  void InitNator(std::string natorId) {
	  _instance->InitNator(natorId);
  }
  void InitRMD(const char* portName, uint8_t topRMDID, uint8_t botRMDID) {
	  _instance->InitRMD(portName, topRMDID, botRMDID);
  }
  void InitRMD(const char* portName) {
	  InitRMD(portName, 1, 2);
  }
  void InitTopCamera(std::string topCameraId) {
	  _instance->InitTopCamera(topCameraId);
  }
  void InitTopCamera() {
	  _instance->InitTopCamera(_topCameraId);
  }
  void InitBotCamera(std::string botCameraId) {
	  _instance->InitBotCamera(botCameraId);
  }
  void InitBotCamera() {
	  InitBotCamera(_bottomCameraId);
  }
  void SetZero() {
	  _instance->SetZero();
  }
  void Stop() {
	  _instance->Stop();
  }
  void JointsMoveAbsolute(const Joints j) {
	  _instance->JointsMoveAbsolute(j);
  }
  void JointsMoveRelative(const Joints j) {
	  _instance->JointsMoveRelative(j);
  }
  Joints GetCurrentJoint() {
	  return _instance->GetCurrentJoint();
  }


 protected:
  !D5RobotWrapper() { delete _instance; }

 private:
  D5Robot* _instance;
  static const char* _natorId = "usb:id:2250716012";
  static const char* _topCameraId = "00-21-49-03-4D-95";
  static const char* _bottomCameraId = "00-21-49-03-4D-94";
};
}  // namespace D5RWrapperClassLibrary

// wrap_native_class_for_mgd_consumption.cpp
// compile with: /clr /LD
// #include <vcclr.h>
// #include <windows.h>
// #using < System.dll>
//
// using namespace System;
//
// class UnmanagedClass {
// public:
//  LPCWSTR GetPropertyA() { return 0; }
//  void MethodB(LPCWSTR) {}
//};
//
// public
// ref class ManagedClass {
// public:
//  // Allocate the native object on the C++ Heap via a constructor
//  ManagedClass() : m_Impl(new UnmanagedClass) {}
//
//  // Deallocate the native object on a destructor
//  ~ManagedClass() { delete m_Impl; }
//
// protected:
//  // Deallocate the native object on the finalizer just in case no destructor
//  is
//  // called
//  !ManagedClass() { delete m_Impl; }
//
// public:
//  property String ^
//      get_PropertyA {
//        String ^ get() { return gcnew String(m_Impl->GetPropertyA()); }
//      }
//
//      void
//      MethodB(String ^ theString) {
//    pin_ptr<const WCHAR> str = PtrToStringChars(theString);
//    m_Impl->MethodB(str);
//  }
//
// private:
//  UnmanagedClass* m_Impl;
//};