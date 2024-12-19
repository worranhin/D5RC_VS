#include "CVK.h"

namespace D5R {
	CVK::CVK() {};
	CVK::CVK(HANDLE handle) {
		_handle = handle;
	}
	CVK::~CVK() {};

	void CVK::GotoJawPlatform() {
		char command_L[] = "rc-10";
		char command_R[] = "rt-10";
		if (!WriteFile(_handle, command_L, sizeof(command_L), &_bytesWritten, NULL)) {
			throw RobotException(ErrorCode::SerialSendError, "In CVK::GotoJawPlatform, L, ");
		}
		if (!WriteFile(_handle, command_R, sizeof(command_R), &_bytesWritten, NULL)) {
			throw RobotException(ErrorCode::SerialSendError, "In CVK::GotoJawPlatform, R, ");
		}
	}
	void CVK::GotoMaterialPlatform() {
		char command_L[] = "rc+10";
		char command_R[] = "rt+10";
		if (!WriteFile(_handle, command_L, sizeof(command_L), &_bytesWritten, NULL)) {
			throw RobotException(ErrorCode::SerialSendError, "In CVK::GotoJawPlatform, L, ");
		}
		if (!WriteFile(_handle, command_R, sizeof(command_R), &_bytesWritten, NULL)) {
			throw RobotException(ErrorCode::SerialSendError, "In CVK::GotoJawPlatform, R, ");
		}
	}
}