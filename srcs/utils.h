#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace std;
using namespace cv;

// 将一幅图片切分为两幅
bool d2(Mat &img, Mat &imgL, Mat &imgR){
    Size img_size = img.size();
    int w = int(img_size.width / 2), h = img_size.height;
    Rect RectL(0, 0, w, h); Rect RectR(w, 0, w, h);
    imgL = img(RectL); imgR = img(RectR);
    return ((!imgL.empty()) && (!imgR.empty()));
}

void imread2(const string &filename, Mat &img, int color_model=1){
    img = imread(filename, color_model);
    if (img.empty()){
        cerr << "图像为空:" << filename << endl; assert(0);
    }
}

void show(Mat &img, const string &title){
    imshow(title, img);
    waitKey(0);
    destroyWindow(title);
}

bool is_exist(const string &filename){
    ifstream f(filename.c_str());
    return f.good();
}

bool save_disp(Mat &disp8, const string &filename) {
    ofstream fw(filename, ios::out);
    if (!fw.is_open()){
        cerr << "未成功打开视差文件" << endl;
        return false;
    }
    fw << disp8.rows << endl;
    fw << disp8.cols << endl;
    for (int y = 0; y < disp8.rows; y++) {
        for (int x = 0; x < disp8.cols; ++x) {
            // 这里视差矩阵是CV_16S 格式的，故用 short 类型读取
            double d = disp8.at<short>(y, x);
            fw << d << endl;
        }
    }
    fw.close();
    cout << "成功保存视差文件:" << filename << endl;
    return true;
}

bool save3dPoint(Mat &xyz, const string &filename){
    const double max_z = 1.0e4;
    ofstream fw(filename, ios::out);
    if (!fw.is_open()){
        cerr << "写入点云文件失败!" << filename << endl;
        return false;
    }
    Vec3f point;
    for (int y = 0; y < xyz.rows; ++y) {
        for (int x = 0; x < xyz.cols; ++x) {
            point = xyz.at<Vec3f>(y, x);
            if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z){
                continue;
            }
            fw << point[0] << " " << point[1] << " " << point[2] << endl;
        }
    }
    fw.close();
    cout << "保存点云文件成功:" << filename << endl;
    return true;
}

bool reproject3d(Mat &disp8, Mat &xyz, Mat &Q){
    reprojectImageTo3D(disp8, xyz, Q, false, -1);
    return true;
}

#endif //UTILS_H
