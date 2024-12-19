/**
 * @file Visual.h
 * @author drawal (2581478521@qq.com)
 * @brief 视觉debug函数库，开发中
 * @version 0.1
 * @date 2024-12-08
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "CameraBot.h"
#include "CameraTop.h"
#include <time.h>
 #include <halconcpp/HalconCpp.h>

void Test_GetAndSaveImg(D5R::CameraTop* topCamera);


void Test_GetAndSaveImg(D5R::CameraBot* botCamera);
void Test_GetPosTemplate(cv::Mat img);
void Test_GetROI(cv::Mat img, cv::Mat temp);
void Test_GetJawTemplate(cv::Mat img);
void Test_GetJawCircleCenter(cv::Mat img);
void Test_GetSIFTParam(cv::Mat model, D5R::ModelType m);
void Test_Match(cv::Mat image, D5R::ModelType m);
void Test_GetClampTemplate(cv::Mat img);
void Test_GetBotCameraPosLine(cv::Mat img);
void Test_GetClampTemplate_BotC(cv::Mat img);
double Test_GetFallingHeight(cv::Mat img);
void Test_halcon();
