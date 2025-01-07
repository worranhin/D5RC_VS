#include "VC.h"

VC::VC() {
	// topC
	// 夹钳模板
	_clamp.img = cv::imread("./model/clampTemplate/clamp.png", 0);
	_clamp.center = cv::Point2f(330.0f, 29.0f);
	_clamp.point = cv::Point2f(331.2f, 149.3f);
	cv::FileStorage fs1("./model/clampTemplate/KeyPoints_Clamp.yml", cv::FileStorage::READ);
	fs1["keypoints"] >> _clamp.keypoints;
	fs1.release();
	cv::FileStorage fs2("./model/clampTemplate/Descriptors_Clamp.yml", cv::FileStorage::READ);
	fs2["descriptors"] >> _clamp.descriptors;
	fs2.release();

	// 中号钳口模板
	HalconCpp::ReadShapeModel("./model/jawTemplate/shm/Temp_DL.shm", &_jawMid.temp_dl);
	HalconCpp::ReadShapeModel("./model/jawTemplate/shm/Temp_DR.shm", &_jawMid.temp_dr);

	// 钳口库定位模板
	_posTemplate_2 = cv::imread("./model/posTemplate/PosTemple_rect.png", 0);

	// 粗定位点，相对于_roiPos而言
	_roughPosPoint = cv::Point2f(435, 1050);


	// botC
	// 夹钳模板
	_clampBot.model = cv::imread("./model/botCTemplate/clamp_bot.png", 0);
	_clampBot.pos.push_back(cv::Point2f(93.9549, 95.2925));
	_clampBot.pos.push_back(cv::Point2f(513.976, 91.9765));

	// 钳口库平台水平线
	_jawLibLine_a = -0.00116294;
	_jawLibLine_b = 1718.94;
	// 图像与显示映射参数
	_mapParam = 0.00945084;
}
VC::~VC() {}

/**
 * @brief 将OpenCV Mat格式的图片转换成Halcon HObject格式
 * @param img
 * @return
 */
HalconCpp::HObject VC::Mat2HImage(cv::Mat img) {
	if (img.empty() || img.channels() != 1) {
		std::cout << "Error, invaid img" << std::endl;
		return {};
	}

	int width = img.cols, height = img.rows;
	HalconCpp::HObject ho_img;
	HalconCpp::GenImage1(&ho_img, "byte", width, height, (const HalconCpp::HTuple&)img.data);
	return ho_img;
}
/**
 * @brief 将Halcon HObject格式转换为OpenCV Mat格式
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
 * @brief 分割钳口库，更新类内钳口库roi信息
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
		_roiPos = cv::Point2f(minLoc.x - 300.0f, minLoc.y + 300.0f);
		break;
	case 3:
	default:
		break;
	}
}
/**
 * @brief 根据现有的模板进行SIFT匹配，返回模板中定位点在img中的位置信息
 * @param img
 * @param m
 * @return pst, 匹配失败则返回空
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
	cv::Rect roi = cv::Rect2f(_roiPos, cv::Size2f(850.0f, 2046.0f - _roiPos.y));
	cv::Mat ROI = img(roi).clone();

	// SIFT特征点
	cv::Ptr<cv::SIFT> sift = cv::SIFT::create(400);
	std::vector<cv::KeyPoint> keyPoints_Img;
	sift->detect(ROI, keyPoints_Img);
	// 描述
	cv::Mat descriptors_Img;
	sift->compute(ROI, keyPoints_Img, descriptors_Img);
	// 匹配
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
	cv::perspectiveTransform(modelPosition, pst, homography);
	for (auto p : pst) {
		p += _roiPos;
	}
	return pst;
}
/**
 * @brief 更新平台水平线
 * @param img
 * @param index 1：钳口库 2：物料台
 */
void VC::GetHorizontalLine(cv::Mat img, int index) {
	// 将钳口台下半部分遮住，防止干扰，具体使用根据钳口台与相机高度而定
	cv::Point2f roiPos(200, 1700);
	cv::Rect roi = cv::Rect(roiPos, cv::Size(2200, 348));
	cv::Mat ROI = img(roi).clone();

	// 图像处理
	cv::Mat bin;
	cv::threshold(ROI, bin, 50, 255, cv::THRESH_BINARY);
	cv::Mat gauss;
	cv::GaussianBlur(bin, gauss, cv::Size(5, 5), 25);
	cv::Mat dst, edge;
	cv::Scharr(gauss, dst, CV_32F, 1, 0);
	cv::convertScaleAbs(dst, edge);
	std::vector<cv::Vec4f> lines;
	cv::HoughLinesP(edge, lines, 1, CV_PI / 180, 200, 500, 300);

	// 最小二乘拟合
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
	}
	else {
		_ringLibLine_a = (sum_xy - n * mean_x * mean_y) / (sum_x2 - n * mean_x * mean_x);
		_ringLibLine_b = (mean_y - _ringLibLine_a * mean_x);
	}
}
/**
 * @brief 获取机器人z轴移动距离
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
	return -distance * _mapParam + 0.3;

}


// 类内变量接口
Clamp VC::GetClamp() { return _clamp; }

Jaw VC::GetJaw() { return _jawMid; }

cv::Point2f VC::GetROIPos() { return _roiPos; }

cv::Point2f VC::GetRoughPosPoint() { return _roughPosPoint; }

double VC::GetMapParam() { return _mapParam; }
