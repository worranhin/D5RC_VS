#include "SerialPort.h"
namespace D5R {
    #include <string>

    SerialPort::SerialPort(const char* serialPort, int baudRate) {
        std::wstring wideSerialPort = std::wstring(serialPort, serialPort + strlen(serialPort));
        _handle = CreateFileW(wideSerialPort.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

        if (_handle == INVALID_HANDLE_VALUE) {
            throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to open serial port");
            return;
        }

        BOOL bSuccess = SetupComm(_handle, 100, 100);
        if (!bSuccess) {
            throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to Setup Comm");
            return;
        }

        COMMTIMEOUTS commTimeouts = { 0 };
        commTimeouts.ReadIntervalTimeout = 50;         // 读取时间间隔超时
        commTimeouts.ReadTotalTimeoutConstant = 100;   // 总读取超时
        commTimeouts.ReadTotalTimeoutMultiplier = 10;  // 读取超时乘数
        commTimeouts.WriteTotalTimeoutConstant = 100;  // 总写入超时
        commTimeouts.WriteTotalTimeoutMultiplier = 10; // 写入超时乘数

        bSuccess = SetCommTimeouts(_handle, &commTimeouts);
        if (!bSuccess) {
            throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to Set Comm Timeouts");
            return;
        }

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        if (!GetCommState(_handle, &dcbSerialParams)) {
            throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to Get Comm State");
            return;
        }
        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        if (!SetCommState(_handle, &dcbSerialParams)) {
            throw RobotException(ErrorCode::SerialInitError, "In SerialPort constructor: Failed to Set Comm State");
            return;
        }
    }

    SerialPort::~SerialPort() { CloseHandle(_handle); }

    /**
     * @brief 返回串口句柄
     *
     * @return HANDLE
     */
    HANDLE SerialPort::GetHandle() { return _handle; }

} // namespace D5R
