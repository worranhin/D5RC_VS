# D5RC_VS

该代码用于实现五自由度机器人装配控制

> 由于个人能力原因，无法实现 OpenCV 与 Halcon 的 dll 链接问题，原[CMake 构建版本](https://github.com/drawal001/D5RC)迁移至本仓库的 VS 版本，并取消 CMake 构建方案，如果有对原方案感兴趣的大佬，欢迎交流。

## 使用

> 本库仅在 Windows 平台下开发并测试

## 配置

1. [Visual studio 2022](https://visualstudio.microsoft.com/zh-hans/#vs-section)
2. [OpenCV 4.10.0](https://opencv.org/releases/)
3. [halcon 24.11 Progress-Steady](https://www.mvtec.com/cn/downloads)
4. [大恒相机水星一代](https://www.daheng-imaging.com/downloads/)
5. Nartor 电机
6. RMD 脉塔电机

### 环境

> 本库将所需使用的库文件打包，发布在 [Release]() 中，需要的可自行下载，并在 lib 目录下直接解压

**需要注意的是：**

1. Nator 电机 SDK 需额外配置，进入 `lib/Nator/SDK1.4.12` 按指示操作。
2. 大恒相机需下载其驱动软件，[链接在此](https://www.daheng-imaging.com/downloads/)，且需要工业级以太网视觉采集卡，若出现相机可识别，但无法打开的问题，可使用大恒相机 SDK 中的`GxGigEIPConfig.exe`重新配置 IP
3. Nator 电机与 RMD 电机驱动程序不在此获取
