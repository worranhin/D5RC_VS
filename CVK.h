#pragma once
#include "RobotException.hpp"
#include <iostream>
#include <Windows.h>
#include <cstdint>
namespace D5R {
	class CVK
	{
	public:
		CVK();
		CVK(HANDLE handle);
		~CVK();
		void GotoJawPlatform();
		void GotoMaterialPlatform();
	private:
		HANDLE _handle;
		DWORD _bytesWritten;
	};
}//namespace D5R


