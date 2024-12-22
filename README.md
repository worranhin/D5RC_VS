# D5RC_VS

该代码用于实现五自由度机器人装配控制

> ~~由于个人能力原因，无法实现 OpenCV 与 Halcon 的 dll 链接问题，原[CMake 构建版本](https://github.com/drawal001/D5RC)迁移至本仓库的 VS 版本，并取消 CMake 构建方案，如果有对原方案感兴趣的大佬，欢迎交流。~~

> Cmake 应该是没什么问题的，但在多方商量下，现阶段还是选择使用 Visual Studio，毕竟本项目只打算在 Windows 下测试并运行。

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

> 本库将所需使用的库文件打包，发布在 [Release](https://github.com/drawal001/D5RC_VS/releases/tag/v0.1.0) 中，需要的可自行下载，并按下文内容配置。~~并在 lib 目录下直接解压~~

**需要注意的是：**

1. Nator 电机 SDK 需额外配置，进入 `lib/Nator/SDK1.4.12` 按指示操作
2. 大恒相机需下载其驱动软件，[链接在此](https://www.daheng-imaging.com/downloads/)，且需要工业级以太网视觉采集卡，若出现相机可识别，但无法打开的问题，可使用大恒相机 SDK 中的`GxGigEIPConfig.exe`重新配置 IP
3. halcon 需每月获取许可文件并更名为 **license.dat**，放置在 D5RC_VS.sln 所在的文件目录下
4. Nator 电机与 RMD 电机驱动程序不在此获取

#### Halcon 配置

1. 去上述官网安装 halcon 24.11
1. 确认拥有 C++ 11 以上版本的工具链
1. 配置项目环境（本项目已配置完成）
	1. 确认添加包含目录  `$(HALCONROOT)\include` `$(HALCONROOT)\include\halconcpp`
	1. 确认 C++ 的链接库包含 `$(HALCONROOT)\lib\$(HALCONARCH)\halcon.lib` 和 `$(HALCONROOT)\lib\$(HALCONARCH)\halconcpp.lib`
	1. 确认 dll 路径已包含在环境变量 PATH 中 `$(HALCONROOT)\bin\%HALCONARCH%\` （正常情况下安装 Halcon 时它会自动完成的，但如果程序跑不了的话可以检查一下）

> `$(HALCONROOT)` 表示环境变量，可能在不同操作系统或 IDE 会有不同表示方式

> 更多信息参考官方文档 [Programer's Guide](https://www.mvtec.com/fileadmin/Redaktion/mvtec.com/products/halcon/documentation/halcon/programmers_guide.pdf)（7.5节）

#### OpenCV 配置

1. 安装 OpenCV（也可以直接解压仓库自带的库文件包获得）
1. 设置环境变量： `setx OpenCV_DIR D:\path\to\opencv-4.10.0\opencv\build\x64\vc16` （自行修改为 OpenCV 所在目录）
1. 添加 OpenCV 的 DLL 文件位置 `%OpenCV_DIR%\bin` 到系统环境变量 PATH 中  
1. 配置项目环境（本项目已配置完成）
	1. 添加包含目录 `$(OpenCV_DIR)\..\..\include`
	1. 添加库目录 `$(OPENCV_DIR)\lib`
	1. 添加导入库 `opencv_world4100d.lib` **或** `opencv_world4100.lib` （后缀 `d` 表示这是 debug 需要的库，如 `imshow` 这类函数需要它，它和无 `d` 后缀的库一般不同时存在）

> 更多信息参考官方文档 [Installation in Windows](https://docs.opencv.org/4.x/d3/d52/tutorial_windows_install.html#tutorial_windows_install_path) 以及 [How to build applications with OpenCV inside the "Microsoft Visual Studio"](https://docs.opencv.org/4.x/dd/d6e/tutorial_windows_visual_studio_opencv.html)

#### 大恒相机配置

1. 去[官网](https://www.daheng-imaging.com/downloads/)安装大恒相机的 SDK，安装过程中将自动添加 DLL 目录到 PATH 中
1. 建议如上设置环境变量 `setx GALAXY_DIR "C:\path\to\Daheng Imaging\GalaxySDK\Development\VC SDK"`
1. 配置项目环境（本项目暂未修改）
	1. 添加包含目录 `$(GALAXY_DIR)\inc`
	1. 添加库目录 `$(GALAXY_DIR)\lib\x64` 根据自己的系统架构选择
	1. 添加导入库 `GxIAPI.lib` 和 `DxImageProc.lib`

#### Nators 电机配置

1. 解压项目的依赖压缩包获得 Nators 的 SDK
1. 建议如上设置环境变量 `setx NATORS_DIR "c:\path\to\Nator"`
1. 进入 SDK 所在目录下的 `SDK1.4.12`，按要求放置 dll 文件（这个不太确定能不能通过添加 PATH 实现，总之这是对方技术支持提供的方法且可行）
1. 配置项目环境（本项目暂未修改）
	1. 添加包含目录 `$(NATORS_DIR)\include`
	1. 添加库目录 `$(NATORS_DIR)\lib` 或 `$(NATORS_DIR)\SDK1.4.12\64`（这个也不太确定，按道理应该是后者，但之前测试时一直用的是前者，需要日后进一步测试）
	1. 添加导入库 `NTControl.lib`

#### RMD 电机配置

RMD 电机通过 USB 串口转 RS485 控制，无需 SDK，但可能需要相应的驱动，并自行查看对应的 COM 口。

#### PropertySheet 的使用

为了方便在多个项目中设置上述配置，本项目的根目录下提供了若干 PropertySheet（属性表）。若要在解决方案（Solution）下新建项目（Project），只需要在属性管理器中为指定的项目添加现有属性表即可。
属性管理器可以在顶部菜单栏的视图选项中找到并打开。