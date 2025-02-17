/**
 * @file GxCamera.cpp
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief Implementation of GxCamera Class
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "GxCamera.h"

namespace D5R {

    int GxCamera::_instanceNum = 0;

    /**
     * @brief Construct a new Gx Camera:: Gx Camera object
     *
     * @param id 大恒相机MAC地址
     */
    GxCamera::GxCamera(std::string_view id) : _id(id) { Init(); }

    /**
     * @brief 获取GxCamera最后的错误信息
     *
     * @return const char*
     */
    const char* GxCamera::GetGxError() {
        size_t size = 256;
        static char err_info[256];
        GXGetLastError(nullptr, err_info, &size);
        return err_info;
    }

    /**
     * @brief GxCamera初始化
     *
     */
    void GxCamera::Init() {
        GX_STATUS status;

        // 初始化Lib
        if (GxCamera::_instanceNum++ == 0) { // 若 _instanceNum != 0, 代表已经  InitLib 则跳过这一步
            status = GXInitLib();
            if (status != GX_STATUS_SUCCESS) {
                throw RobotException(ErrorCode::CameraInitError, "In GxCamera::Init, Failed to init the lib, error: " + std::string(GetGxError()));
            }
        }
        // 枚举相机
        uint32_t nums{};
        status = GXUpdateAllDeviceList(&nums, 1000);
        if (status != GX_STATUS_SUCCESS) {
            throw RobotException(ErrorCode::CameraInitError, "In GxCamera::Init, Failed to update device list, error: " + std::string(GetGxError()));
        }
        if (nums == 0) {
            throw RobotException(ErrorCode::CameraInitError, "In GxCamera::Init, Could not find any camera device");
        }
        PASS_("Success to enum the camera device, num = %d", nums);
        // 获取设备信息
        std::unordered_set<std::string> mac_set{};
        std::vector<GX_DEVICE_BASE_INFO> device_info(nums);
        size_t base_info_size = nums * sizeof(GX_DEVICE_BASE_INFO);
        status = GXGetAllDeviceBaseInfo(device_info.data(), &base_info_size);
        if (status != GX_STATUS_SUCCESS) {
            throw RobotException(ErrorCode::CameraInitError, "In GxCamera::Init, Failed to get device info, error: " + std::string(GetGxError()));
        }

        // 定义设备打开参数
        GX_OPEN_PARAM open_param;
        open_param.accessMode = GX_ACCESS_EXCLUSIVE;
        open_param.openMode = GX_OPEN_MAC;
        for (uint32_t i = 0; i < nums; ++i) {
            GX_DEVICE_IP_INFO ip_info;
            if (device_info[i].deviceClass != GX_DEVICE_CLASS_GEV)
                continue;
            status = GXGetDeviceIPInfo(i + 1, &ip_info);
            if (status != GX_STATUS_SUCCESS) {
                throw RobotException(ErrorCode::CameraInitError, "In GxCamera::Init, Failed to get device IP, error: " + std::string(GetGxError()));
            }
            mac_set.emplace(ip_info.szMAC);
        }
        // 打开相机
        if (_id.empty()) {
            status = GXOpenDeviceByIndex(1, &_handle);
        }
        else if (mac_set.find(_id) != mac_set.end()) {
            open_param.pszContent = const_cast<char*>(_id.c_str());
            status = GXOpenDevice(&open_param, &_handle);
        }
        else {
            for (const auto& mac : mac_set) {
                INFO_("current MAC: %s", mac.c_str());
            }
            throw RobotException(ErrorCode::CameraInitError, "In GxCamera::Init, Could not find the device matched with the provided MAC");
        }

        if (status != GX_STATUS_SUCCESS) {
            throw RobotException(ErrorCode::CameraInitError, "In GxCamera::Init, Failed to open the camera, error: " + std::string(GetGxError()));
        }
        else {
            // 设置相机流通道包长属性
            uint32_t packet_size{};
            GXGetOptimalPacketSize(_handle, &packet_size);
            GXSetIntValue(_handle, "GevSCPSPacketSize", packet_size);
        }
        // 设置工作模式为有触发的连续模式
        GXSetEnum(_handle, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
        // GXSetEnum(_handle, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        GXSetEnum(_handle, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        // 设置曝光模式
        GXSetEnum(_handle, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
        GXSetEnum(_handle, GX_ENUM_EXPOSURE_TIME_MODE,
            GX_EXPOSURE_TIME_MODE_STANDARD);
        // 取流
        status = GXGetInt(_handle, GX_INT_PAYLOAD_SIZE, &_payload);
        if (status != GX_STATUS_SUCCESS || _payload <= 0) {
            throw RobotException(ErrorCode::CameraInitError, "In GxCamera::Init, Failed to get payload size, error: " + std::string(GetGxError()));
        }
        _data.pImgBuf = malloc(_payload);
        status = GXSendCommand(_handle, GX_COMMAND_ACQUISITION_START);
        if (status != GX_STATUS_SUCCESS) {
            free(_data.pImgBuf);
            throw RobotException(ErrorCode::CameraInitError, "In GxCamera::Init, Failed to start stream, error: " + std::string(GetGxError()));
        }

        cv::Mat matrix = (cv::Mat_<double>(3, 3) << 105.9035, 0.04435, 0, 0, 105.689, 0, 0, 0, 1);
        cv::Mat dist = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
        cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(matrix, dist, cv::Size(2592, 2048), 0);
        cv::initUndistortRectifyMap(matrix, dist, cv::Mat::eye(3, 3, CV_64F), newCameraMatrix, cv::Size(2592, 2048), CV_32FC1, _map1, _map2);
    }

    /**
     * @brief 图像信息转码解码
     *
     * @param image 解码后获取的图像
     * @return true
     * @return false
     */
    bool GxCamera::Retrieve(cv::OutputArray image) {
        // 获取像素格式、图像宽高、图像缓冲区
        int32_t pixel_format = _data.nPixelFormat;
        int32_t width = _data.nWidth, height = _data.nHeight;
        void* buffer = _data.pImgBuf;

        // 解码与转码
        if (pixel_format == GX_PIXEL_FORMAT_MONO8) {
            cv::Mat src_img(height, width, CV_8U, buffer);
            cv::Mat dst;
            cv::remap(src_img, dst, _map1, _map2, cv::INTER_NEAREST);
            image.assign(dst);
        }
        else if (pixel_format == GX_PIXEL_FORMAT_BAYER_GR8 ||
            pixel_format == GX_PIXEL_FORMAT_BAYER_RG8 ||
            pixel_format == GX_PIXEL_FORMAT_BAYER_GB8 ||
            pixel_format == GX_PIXEL_FORMAT_BAYER_BG8) {
            cv::Mat src_img(height, width, CV_8U, buffer);
            cv::Mat dst_img;
            const static std::unordered_map<int32_t, int> bayer_map{
                {GX_PIXEL_FORMAT_BAYER_GB8, cv::COLOR_BayerGB2BGR},
                {GX_PIXEL_FORMAT_BAYER_GR8, cv::COLOR_BayerGR2BGR},
                {GX_PIXEL_FORMAT_BAYER_BG8, cv::COLOR_BayerBG2BGR},
                {GX_PIXEL_FORMAT_BAYER_RG8, cv::COLOR_BayerRG2BGR},
            };
            cv::cvtColor(src_img, dst_img, bayer_map.at(pixel_format));
            cv::Mat dst;
            cv::remap(dst_img, dst, _map1, _map2, cv::INTER_NEAREST);
            image.assign(dst);
        }
        else {
            ERROR_("Invalid pixel format");
            return false;
        }
        return true;
    }

    /**
     * @brief 获取相机图像
     *
     * @param image 读取的图像
     * @return true
     * @return false
     */
    bool GxCamera::Read(cv::OutputArray image) {
        GXSendCommand(_handle, GX_COMMAND_TRIGGER_SOFTWARE); // 发送软触发命令

        auto status = GXGetImage(_handle, &_data, 1000);
        if (status != GX_STATUS_SUCCESS) {
            ERROR_("Failed to read image, error: %s", GetGxError());
            Reconnect();
            return false;
        }
        auto flag = Retrieve(image);
        return flag;
    }

    /**
     * @brief 设备重连
     *
     */
    void GxCamera::Reconnect() {
        ERROR_("camera device reconnect ");
        Release();
        Sleep(100);
        Init();
    }

    /**
     * @brief 相机句柄释放
     *
     */
    void GxCamera::Release() {
        auto status = GXSendCommand(_handle, GX_COMMAND_ACQUISITION_STOP);
        if (status != GX_STATUS_SUCCESS) {
            ERROR_("Failed to stop stream, error: %s", GetGxError());
        }
        free(_data.pImgBuf);
        status = GXCloseDevice(_handle);
        if (status != GX_STATUS_SUCCESS) {
            ERROR_("Failed to close camera, error: %s", GetGxError());
        }

        if (--GxCamera::_instanceNum == 0) { // 如果全部相机都已注销，则 CloseLib
            GXCloseLib();
        }
    }

    GxCamera::~GxCamera() { Release(); }
} // namespace D5R