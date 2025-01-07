#include "D5Robot.h"
#include "KineHelper.hpp"
#include "RobotException.hpp"
#include "Visual.h"



const std::string port = "\\\\.\\COM16";
const std::string port_cvk = "\\\\.\\COM10";

int main() {
    std::cout << "Hello Robot!" << std::endl;
     D5R::D5Robot robot;

    // robot.InitNator();

    // robot.InitRMD(port.c_str());

    // robot.InitTopCamera();

    // robot.InitBotCamera();

    // robot.JointsMoveAbsolute({0, 0, 7500000, 5000000, 0});
    // robot.JointsMoveRelative({0, 0, 1000000, 0, 0});
    // robot.JointsMoveAbsolute({0, 0, 7500000, 0, 0});

     // ����CVK���
     //D5R::SerialPort* handleCVK = new D5R::SerialPort(port_cvk.c_str());
     //D5R::CVK* cvk = new D5R::CVK(handleCVK->GetHandle());
     //cvk->GotoJawPlatform();
     //cvk->GotoMaterialPlatform();



     //cv::Mat img = cv::imread("./image/12_12/topC_1733978331.png");
     //// Test_GetJawCircleCenter(img);
     //if (img.empty()) {
     //    std::cerr << "error" << std::endl;
     //    return -1;
     //}
     //cv::imshow("ssdf", img);
     //cv::waitKey(0);

     Test_halcon();

    // robot.VCJawChange();

    // Test_GetAndSaveImg(robot.topCamera);
    // Test_GetAndSaveImg(robot.botCamera);
    // cv::Mat img = cv::imread("../image/12_10/topC_404.png");
    // Test_Deformation_SURF(img);

    return 0;
}