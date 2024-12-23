#include "pch.h"
//#include "D5Robot.h"
#include "CppUnitTest.h"
#include "HalconCpp.h"
//#include "opencv2/opencv.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace D5R;

namespace D5RUnitTest {

	TEST_CLASS(D5RUnitTest) {

public:

	TEST_METHOD(TestD5RobotInit) {
		D5Robot^ robot = gcnew D5Robot();
		Assert::IsNotNull<D5Robot^>(&robot);
	}  // namespace D5RUnitTest

	TEST_METHOD(TestHalconReadImage) {
		using namespace HalconCpp;
		HObject Image;
		HTuple fileName = "test.png";
		Hlong Row, Column, Button;
		HTuple paramName[1], paramValue[1];

		try {
			ReadImage(&Image, fileName);
			Assert::IsTrue(Image.IsInitialized());
			HWindow window(0,0, 800, 600);
			window.DispObj(Image);
			window.DispText("Click me!", "window", 12, 12, "black", HTuple(), HTuple());
			window.GetMbutton(&Row, &Column, &Button);
			Assert::IsNotNull<Hlong>(&Button);
		}
		catch(const HException& exception) {
			const char* message = exception.ErrorMessage();
			Logger::WriteMessage(message);
			Assert::Fail();
		}
	}

	TEST_METHOD(TestOpenCVReadImage) {
		//using namespace cv;
		//Mat image;
		//cv::String fileName = "test.png";
		//cv::String windowName = "Press A Key to Exit";
		//image = imread(fileName, IMREAD_COLOR);
		//Assert::IsFalse(image.empty());
		//namedWindow(windowName, WINDOW_NORMAL);     // Create a window for display.
		////resizeWindow(const String & winname, int width, int height);
		//resizeWindow(windowName, 800, 600);
		//imshow(windowName, image);  // Show our image inside it.
		//auto key = waitKey(0);
		//Assert::AreNotEqual(key, -1);
	}
	};

}
