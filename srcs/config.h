#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
using namespace std;

struct PATH{
    string sep = "/";
    string root_dir = "F:/Desktop/cali/code/StereoCalib/StereoCalibMatch";
    string data_dir = root_dir + sep + "data";
    string calib_dir = data_dir + sep + "calib";
    string intrinsic_file = data_dir + sep + "intrinsic_file.yml";
    string extrinsic_file = data_dir + sep + "extrinsic_file.yml";
    string test_dir = data_dir + sep + "test";
    string output_dir = root_dir + sep + "outputs";
    string disp8_img = output_dir + sep + "disp8.jpg";
    string dispRGB = output_dir + sep + "dispRGB.jpg";
    string disp_txt = output_dir + sep + "disp.txt";
    string point_cloud_txt = output_dir + sep + "point3D.txt";
};

struct PARA{
    const int W = 1280;  // 图片宽(单位pixel)
    const int H = 720;   // 图像高
};

#endif //CONFIG_H
