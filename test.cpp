#include "D5Robot.h"
#include "KineHelper.hpp"
#include "RobotException.hpp"
#include "Visual.h"



const std::string port = "\\\\.\\COM16";
const std::string port_cvk = "\\\\.\\COM10";

int main() {
    std::cout << "Hello Robot!" << std::endl;
     D5R::D5Robot robot;

     robot.InitNator();

     robot.InitRMD(port.c_str());

     robot.InitTopCamera();

     robot.InitBotCamera();

     //robot.JointsMoveAbsolute({0, 0, 7500000, 5000000, 0});
      //robot.JointsMoveAbsolute({0,  0, 0,5000000, 0});
    // robot.JointsMoveRelative({0, 0, 1000000, 0, 0});
    // robot.JointsMoveAbsolute({0, 0, 7500000, 0, 0});

     // ²âÊÔCVKµç»ú
     //D5R::SerialPort* handleCVK = new D5R::SerialPort(port_cvk.c_str());
     //D5R::CVK* cvk = new D5R::CVK(handleCVK->GetHandle());
     //cvk->GotoJawPlatform();
     //cvk->GotoMaterialPlatform();


     
  //   cv::Mat img = cv::imread("./image/1_7/botC_1736238491.png", 0);
	 //VC vc;
	 //vc.GetHorizontalLine(img, 1);
  //   double h = vc.GetVerticalDistance(img, 1);
	 //std::cout << h << std::endl;
     //// Test_GetJawCircleCenter(img);
     //if (img.empty()) {
     //    std::cerr << "error" << std::endl;
     //    return -1;
     //}
     //cv::imshow("ssdf", img);
     //cv::waitKey(0);

  //   cv::Mat img;
	 //robot.topCamera->Read(img);
	 //Test_GetPosTemplate(img);
     robot.VCJawChange();

     //Test_GetAndSaveImg(robot.topCamera);
     //Test_GetAndSaveImg(robot.botCamera);
    // cv::Mat img = cv::imread("../image/12_10/topC_404.png");
    // Test_Deformation_SURF(img);

    return 0;
}