#pragma once

#include "ErrorCode.h"
#include <string>

using namespace System;

namespace ExceptionLib {
	public ref class RobotException: public Exception
	{
		// TODO: 在此处为此类添加方法。
    public:
        ErrorCode code;
        String^ msg;
        RobotException() {
            msg = gcnew String("");
        };
        RobotException(ErrorCode code) { this->code = code; }
        RobotException(ErrorCode code, const char* msg) {
            this->code = code;
            this->msg = gcnew String(msg);
        }
        RobotException(const RobotException% other) : code(other.code) {}
        RobotException% operator=(const RobotException% other) {
            this->code = other.code;
            return *this;
        }

        String^ what() {
            String^ whatInfo;
            if (!this->msg->IsNullOrEmpty) {
                whatInfo = this->msg + " Errorcode: " + this->code.ToString();
            }
            else {
                whatInfo = (gcnew String("The Error code: ")) + this->code.ToString(); // 注意这里的字符串不能以 e/E 开头，否则可能会乱码，原因不明
            }

            return whatInfo;
        }
	};
}
