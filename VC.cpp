#include "VC.h"

VC::VC() {
	// topC
	// ��ǯģ��
	_clamp.img = cv::imread("./model/clampTemplate/clamp.png", 0);
	_clamp.center = cv::Point2f(324.0f, 119.0f);
	_clamp.point = cv::Point2f(328.2f, 212.9f);
	cv::FileStorage fs1("./model/clampTemplate/KeyPoints_Clamp.yml", cv::FileStorage::READ);
	fs1["keypoints"] >> _clamp.keypoints;
	fs1.release();
	cv::FileStorage fs2("./model/clampTemplate/Descriptors_Clamp.yml", cv::FileStorage::READ);
	fs2["descriptors"] >> _clamp.descriptors;
	fs2.release();

	// �к�ǯ��ģ��
	HalconCpp::ReadShapeModel("./model/jawTemplate/shm/Temp_DL.shm", &_jawMid.temp_dl);
	HalconCpp::ReadShapeModel("./model/jawTemplate/shm/Temp_DR.shm", &_jawMid.temp_dr);

	// ǯ�ڿⶨλģ��
	_posTemplate_2 = cv::imread("./model/posTemplate/PosTemple_2.png", 0);

	// �ֶ�λ�㣬�����_roiPos����
	_roughPosPoint = cv::Point2f(450, 1050);


	// botC
	// ��ǯģ��
	_clampBot.model = cv::imread("./model/botCTemplate/clamp_bot.png", 0);
	_clampBot.pos.push_back(cv::Point2f(93.9549, 95.2925));
	_clampBot.pos.push_back(cv::Point2f(513.976, 91.9765));

	// ǯ�ڿ�ƽ̨ˮƽ��
	_jawLibLine_a = -0.00116294;
	_jawLibLine_b = 1718.94;
	// ͼ������ʾӳ�����
	_mapParam = 0.00945084;
}
VC::~VC() {}

/**
 * @brief ��OpenCV Mat��ʽ��ͼƬת����Halcon HObject��ʽ
 * @param img
 * @return
 */
HalconCpp::HObject VC::Mat2HImage(cv::Mat img) {
	if (img.empty() || img.channels() != 1) {
		std::cout << "Error, invaid img" << std::endl;
		return {};
	}

	int width = img.cols, height = img.rows;
	uchar* temp = new uchar[height * width];
	memcpy(temp, img.data, height * width);
	HalconCpp::HObject ho_img;
	HalconCpp::GenImage1(&ho_img, "byte", width, height, (Hlong)(temp));
	delete[] temp;
	return ho_img;
}
/**
 * @brief ��Halcon HObject��ʽת��ΪOpenCV Mat��ʽ
 * @param img
 * @return
 */
cv::Mat VC::HImage2Mat(HalconCpp::HObject img) {
	HalconCpp::HTuple channels;
	HalconCpp::CountChannels(img, &channels);
	if (channels.I() != 1) {
		std::cout << "Error, Invaid img type" << std::endl;
		return {};
	}

	HalconCpp::HTuple hv_Pointer, hv_type, width, height;
	HalconCpp::GetImagePointer1(img, &hv_Pointer, &hv_type, &width, &height);
	int w = width.I();
	int h = height.I();
	int size = w * h;
	cv::Mat cv_img = cv::Mat::zeros(h, w, CV_8UC1);
	memcpy(cv_img.data, (void*)(hv_Pointer.L()), size);
	return cv_img;

}
/**
 * @brief �ָ�ǯ�ڿ⣬��������ǯ�ڿ�roi��Ϣ
 * @param img
 * @param index
 */
void VC::JawLibSegmentation(cv::Mat img, int index) {
	cv::Mat result;
	cv::Point minLoc, maxLoc;
	double minVal, maxVal;
	switch (index)
	{
	case 1:
	case 2:
		cv::matchTemplate(img, _posTemplate_2, result, cv::TM_SQDIFF_NORMED);
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
		// ����ƥ��Ч��
		//cv::rectangle(img, minLoc, cv::Point(minLoc.x + 280, minLoc.y + 280), cv::Scalar(0), 4);
		//cv::imshow("test", img);
		//cv::waitKey(0);
		_roiPos = cv::Point2f(minLoc.x - 300.0f, minLoc.y + 300.0f);
		break;
	case 3:
	default:
		break;
	}
}
/**
 * @brief �������е�ģ�����SIFTƥ�䣬����ģ���ж�λ����img�е�λ����Ϣ
 * @param img
 * @param m
 * @return pst, ƥ��ʧ���򷵻ؿ�
 */
std::vector<cv::Point2f> VC::SIFT(cv::Mat img, Models m) {
	cv::Mat model;
	std::vector<cv::Point2f> modelPosition;
	std::vector<cv::KeyPoint> keyPoints_Model;
	cv::Mat descriptors_model;
	switch (m)
	{
	case CLAMP:
		model = _clamp.img.clone();
		modelPosition.push_back(_clamp.center);
		modelPosition.push_back(_clamp.point);
		keyPoints_Model = _clamp.keypoints;
		descriptors_model = _clamp.descriptors;
		break;
	case JAW_MIN:
		break;
	case JAW_MID:
		break;
	case JAW_MAX:
		break;
	default:
		break;
	}
	// ROI
	cv::Rect roi = cv::Rect(static_cast<cv::Point>(_roiPos), cv::Size(850, 2046 - static_cast<int>(_roiPos.y)));
	cv::Mat ROI = img(roi).clone();

	// SIFT������
	cv::Ptr<cv::SIFT> sift = cv::SIFT::create(400);
	std::vector<cv::KeyPoint> keyPoints_Img;
	sift->detect(ROI, keyPoints_Img);
	// ����
	cv::Mat descriptors_Img;
	sift->compute(ROI, keyPoints_Img, descriptors_Img);
	// ƥ��
	cv::Ptr<cv::DescriptorMatcher> matcher =
		cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
	std::vector<std::vector<cv::DMatch>> knn_matches;
	const float ratio_thresh = 0.7f;
	std::vector<cv::DMatch> goodMatches;
	matcher->knnMatch(descriptors_model, descriptors_Img, knn_matches, 2);
	for (auto& knn_matche : knn_matches) {
		if (knn_matche[0].distance < ratio_thresh * knn_matche[1].distance) {
			goodMatches.push_back(knn_matche[0]);
		}
	}
	// ��ʾƥ����
	//cv::Mat img_matches_res;
	//cv::drawMatches(model, keyPoints_Model, ROI, keyPoints_Img, goodMatches, img_matches_res, cv::Scalar::all(-1),
	//	cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//cv::imshow("test", img_matches_res);
	//cv::waitKey(0);
	if (goodMatches.size() < 8) {
		std::cerr << "Failed to SIFT" << std::endl;
		return {};
	}
	std::vector<cv::Point2f> model_P, img_P;
	for (const auto& match : goodMatches) {
		model_P.push_back(keyPoints_Model[match.queryIdx].pt);
		img_P.push_back(keyPoints_Img[match.trainIdx].pt);
	}
	cv::Mat homography = cv::findHomography(model_P, img_P, cv::RANSAC);
	std::vector<cv::Point2f> pst;
	std::vector<cv::Point2f> pst_Global;
	cv::perspectiveTransform(modelPosition, pst, homography);
	for (auto& p : pst) {
		p.x += _roiPos.x;
		p.y += _roiPos.y;
		pst_Global.push_back(p);
	}
	// ����ƥ��Ч��
	//cv::line(img, pst_Global[0], pst_Global[1], cv::Scalar(0), 4);
	//cv::imshow("test", img);
	//cv::waitKey(0);
	return pst_Global;

}
/**
 * @brief ����ƽ̨ˮƽ��
 * @param img
 * @param index 1��ǯ�ڿ� 2������̨
 */
void VC::GetHorizontalLine(cv::Mat img, int index) {
	// ��ǯ��̨�°벿����ס����ֹ���ţ�����ʹ�ø���ǯ��̨������߶ȶ���
	cv::Point2f roiPos(200, 1500);
	cv::Rect roi = cv::Rect(roiPos, cv::Size(2200, 548));
	cv::Mat ROI = img(roi).clone();

	// ͼ����
	cv::Mat gauss;
	cv::GaussianBlur(ROI, gauss, cv::Size(5, 5), 25);
	cv::Mat bin;
	cv::threshold(gauss, bin, 180, 255, cv::THRESH_BINARY);
	cv::Mat edge;
	cv::Canny(bin, edge, 50, 150);
	
	//// ����Ч��
	//cv::imshow("test", ROI);
	//cv::waitKey(0);
	//cv::imshow("test", bin);
	//cv::waitKey(0);
	//cv::imshow("test", edge);
	//cv::waitKey(0);

	std::vector<cv::Vec4f> lines;
	cv::HoughLinesP(edge, lines, 1, CV_PI / 180, 200, 500, 300);
	std::cout << lines.size() << std::endl;
	// ��С�������
	int n = lines.size() * 2;
	float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
	for (auto& line : lines) {
		sum_x += (line[0] + line[2] + 2 * roiPos.x);
		sum_y += (line[1] + line[3] + 2 * roiPos.y);
		sum_xy += ((line[0] + roiPos.x) * (line[1] + roiPos.y) + (line[2] + roiPos.x) * (line[3] + roiPos.y));
		sum_x2 += ((line[0] + roiPos.x) * (line[0] + roiPos.x) + (line[2] + roiPos.x) * (line[2] + roiPos.x));
		
	}
	float mean_x = sum_x / n;
	float mean_y = sum_y / n;

	if (index == 1) {
		_jawLibLine_a = (sum_xy - n * mean_x * mean_y) / (sum_x2 - n * mean_x * mean_x);
		_jawLibLine_b = (mean_y - _jawLibLine_a * mean_x);

		//// �������Ч��
		//cv::line(img, cv::Point2f(50, 50 * _jawLibLine_a + _jawLibLine_b), 
		//	cv::Point2f(2000, 2000 * _jawLibLine_a + _jawLibLine_b), cv::Scalar(0), 4);
		//cv::namedWindow("a", cv::WINDOW_NORMAL);
		//cv::resizeWindow("a", cv::Size(1295, 1024));
		//cv::imshow("a", img);
		//cv::waitKey(0);
	}
	else {
		_ringLibLine_a = (sum_xy - n * mean_x * mean_y) / (sum_x2 - n * mean_x * mean_x);
		_ringLibLine_b = (mean_y - _ringLibLine_a * mean_x);
	}
}
/**
 * @brief ��ȡ������z���ƶ�����
 * @param img
 * @param index
 * @return
 */
double VC::GetVerticalDistance(cv::Mat img, int index) {
	cv::Mat res;
	cv::Point minLoc, maxLoc;
	double minVal, maxVal;
	float a, b;
	if (index == 1) {
		a = _jawLibLine_a;
		b = _jawLibLine_b;
	}
	else {
		a = _ringLibLine_a;
		b = _ringLibLine_b;
	}
	cv::matchTemplate(img, _clampBot.model, res, cv::TM_SQDIFF_NORMED);
	cv::minMaxLoc(res, &minVal, &maxVal, &minLoc, &maxLoc);
	cv::Point2f minLoc_(minLoc.x, minLoc.y);
	double distance = 0;
	for (int i = 0; i < _clampBot.pos.size(); ++i) {
		distance += (abs(a * (_clampBot.pos[i].x + minLoc_.x) -
			_clampBot.pos[i].y - minLoc_.y + b) / sqrt(a * a + 1));
	}
	distance /= _clampBot.pos.size();
	// ����Ч��
	cv::rectangle(img, minLoc_, cv::Point2f(minLoc_.x + _clampBot.model.size().width, 
		minLoc_.y + _clampBot.model.size().height), cv::Scalar(255), 4);
	cv::line(img, cv::Point2f(50, 50 * a + b), 
		cv::Point2f(2000, 2000 * a + b), cv::Scalar(0), 4);
	cv::namedWindow("a", cv::WINDOW_NORMAL);
	cv::resizeWindow("a", cv::Size(1295, 1024));
	cv::imshow("a", img);
	cv::waitKey(0);
	return -distance * _mapParam + 0.35;

}


// ���ڱ����ӿ�
Clamp VC::GetClamp() { return _clamp; }

Jaw VC::GetJaw() { return _jawMid; }

cv::Point2f VC::GetROIPos() { return _roiPos; }

cv::Point2f VC::GetRoughPosPoint() { return _roughPosPoint; }

double VC::GetMapParam() { return _mapParam; }
