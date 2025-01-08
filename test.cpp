#include "D5Robot.h"
#include "KineHelper.hpp"
#include "RobotException.hpp"
#include "Visual.h"



const std::string port = "\\\\.\\COM16";
const std::string port_cvk = "\\\\.\\COM10";

int main() {
	std::cout << "Hello Robot!" << std::endl;
	D5R::D5Robot robot;

	//robot.InitNator();

	//robot.InitRMD(port.c_str());

	robot.InitTopCamera();

	//robot.InitBotCamera();

	//robot.JointsMoveAbsolute({0, 0, 7500000, 5000000, 0});
	 //robot.JointsMoveAbsolute({0,  0, 0,5000000, 0});
   // robot.JointsMoveRelative({0, 0, 1000000, 0, 0});
   // robot.JointsMoveAbsolute({0, 0, 7500000, 0, 0});
	//VC vc;
	//cv::Mat d = cv::imread("C:/Users/Administrator/Desktop/12_30/topC_0.png", 0);
	//vc.JawLibSegmentation(d, 2);
	//TaskSpaceError pError;
	//for (int i = 200; i < 900; i += 10) {
	//	std::string imgname = "C:/Users/Administrator/Desktop/12_30/topC_" + std::to_string(i) + ".png";
	//	cv::Mat img = cv::imread(imgname, 0);
	//	pError = vc.GetTaskSpaceError(img, FINE);
	//	std::cout << i << ": " << std::endl;
	//	std::cout << "Px: " << pError.Px << std::endl;
	//	std::cout << "Py: " << pError.Py << std::endl;
	//	std::cout << "Rz: " << pError.Rz << std::endl;
	//	std::cout << std::endl;
	//}





	//try {
	//    robot.VCJawChange();
	//}
	//catch (D5R::RobotException &e) {
	//    std::cout << e.what() << std::endl;
	//}

	Test_GetAndSaveImg(robot.topCamera);
	//Test_GetAndSaveImg(robot.botCamera);

	return 0;
}