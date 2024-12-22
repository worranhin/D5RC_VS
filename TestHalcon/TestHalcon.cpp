// TestHalcon.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <HalconCpp.h>


int main()
{
    std::cout << "Hello World!\n";

    using namespace HalconCpp;

    try {
      HTuple WindowHandle1, Row, Column, Button;
      HTuple WindowHandle2, WindowHandle3, WindowHandle4;
      HObject Image, LaplaceImage, Sum, Difference;

      try {
        ReadImage(&Image, "D:/Documents/SCUT/D5RC_VS/image/12_12/topC_1733978331.png");
      } catch (HException& exception) {
        if (H_ERR_FNF == exception.ErrorCode()) {
          printf(
              "Could not read image <fabrik>.\nSet include path to image "
              "files in environment variable HALCONIMAGES\n");
          exit(1);
        } else {
          throw(exception);
        }
      }

      printf("Open graphics window\n");
      SetWindowAttr("background_color", "black");
      OpenWindow(0, 0, -HTuple(1), -HTuple(1), 0, "", "", &WindowHandle1);
      DispObj(Image, WindowHandle1);

      printf("Continue with a mouse-click\n");
      GetMbutton(WindowHandle1, &Row, &Column, &Button);

      /* Application of Laplace-operator                                     */
      printf("Apply user_laplace\n");
      //UserLaplace(Image, &LaplaceImage);
      SetWindowAttr("background_color", "black");
      OpenWindow(100, 100, -HTuple(1), -HTuple(1), 0, "", "", &WindowHandle2);
      DispObj(LaplaceImage, WindowHandle2);

      printf("Continue with a mouse-click\n");
      GetMbutton(WindowHandle2, &Row, &Column, &Button);

      /* Sum of original and Laplace                                         */
      /* This causes an amplification of the contrast                        */
      printf("Apply user_addition\n");
      //UserAddition(Image, LaplaceImage, &Sum);
      SetWindowAttr("background_color", "black");
      OpenWindow(200, 200, -HTuple(1), -HTuple(1), 0, "", "", &WindowHandle3);
      DispObj(Sum, WindowHandle3);

      printf("Continue with a mouse-click\n");
      GetMbutton(WindowHandle3, &Row, &Column, &Button);

      /* Calculate the Difference between orginal und Laplace                */
      printf("Apply user_diff\n");
      //UserDiff(Image, LaplaceImage, &Difference);
      SetWindowAttr("background_color", "black");
      OpenWindow(300, 300, -HTuple(1), -HTuple(1), 0, "", "", &WindowHandle4);
      DispObj(Difference, WindowHandle4);

      printf("Finish with a mouse-click\n");
      GetMbutton(WindowHandle4, &Row, &Column, &Button);
      CloseWindow(WindowHandle4);
      CloseWindow(WindowHandle3);
      CloseWindow(WindowHandle2);
      CloseWindow(WindowHandle1);
    } catch (HException& exception) {
      const char* procname = exception.ProcName();
      if (procname && '\0' != procname[0]) {
        printf("  Error #%u in %s: %s\n", exception.ErrorCode(),
               exception.ProcName().TextA(), exception.ErrorMessage().TextA());
      } else {
        printf("  Error #%u: %s\n", exception.ErrorCode(),
               exception.ErrorMessage().TextA());
      }
    }

    return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
