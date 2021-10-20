#ifndef CoreBM_H
#define CoreBM_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include "utils.h"

using namespace std;
using namespace cv;

class CoreBM {
private:
    Size imgSize;
    Mat img, imgL, imgR, imgL_r, imgR_r, imgShow;
    Mat M_L, M_R, D_L, D_R;
    Rect validROI_L, validROI_R;
    Mat  R, T, rt_L, rt_R, P_L, P_R;
    Mat mapx_L, mapy_L, mapx_R, mapy_R;
    Mat disp;
public:
    Mat Q;

private:
    int unitDisparity;
    int numberOfDisparities;
    Ptr<StereoBM> bm = cv::StereoBM::create();

public:
    void init(const string &intrinsic, const string &extrinsic, int W, int H){
        imgSize = Size(W, H);
        // 01 加载内参
        load_intrinsic(intrinsic);
        // 02 加载外参
        load_extrinsic(extrinsic);
        // 03 立体矫正
        rectifyMap();
        // 04 BM算法初始化
        initBM();
    }

    void match(const string &filename, Mat &disp8){
        imread2(filename,img, 0);
        d2(img,imgL,imgR);
        show(img,"原始图像");
        remap(imgL, imgL_r, mapx_L, mapy_L, INTER_LINEAR);
        remap(imgR, imgR_r, mapx_R, mapy_R, INTER_LINEAR);
        hconcat(imgL_r, imgR_r, imgShow);
        show(imgShow, "矫正图像");
        bm->compute(imgL_r, imgR_r, disp);
        // 将16位符号整形的视差矩阵转换为8位无符号整形矩阵
        disp.convertTo(disp8, CV_8U, 255. / (numberOfDisparities * 16));

    }

private:
    bool load_intrinsic(const string &filename){
        FileStorage fr(filename, FileStorage::READ);
        if (!fr.isOpened()){
            cerr << "内参文件未打开:" << filename << endl;
            return false;
        }
        fr["M_L"] >> M_L; fr["D_L"] >> D_L;
        fr["M_R"] >> M_R; fr["D_R"] >> D_R;
        return true;
    }

    bool load_extrinsic(const string &filename){
        FileStorage fr(filename, FileStorage::READ);
        if (!fr.isOpened()){
            cerr << "外参文件未打开:" << filename << endl;
            return false;
        }
        else{
            fr["R"] >> R; fr["T"] >> T;
            return true;
        }
    }

    // 立体矫正
    void rectifyMap(){
        stereoRectify(
                M_L, D_L,
                M_R, D_R,
                imgSize,
                R, T,
                rt_L, rt_R,
                P_L, P_R,
                Q,
                CALIB_ZERO_DISPARITY,
                -1,
                imgSize,
                &validROI_L, &validROI_R);
        initUndistortRectifyMap(M_L, D_L, rt_L, P_L,
                                imgSize, CV_16SC2, mapx_L, mapy_L);
        initUndistortRectifyMap(M_R, D_R, rt_R, P_R,
                                imgSize, CV_16SC2, mapx_R, mapy_R);


    }

    // 参考:https://blog.csdn.net/Lewispu/article/details/80571481?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-2.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-2.control
    void initBM(){
        unitDisparity = 15;
        numberOfDisparities = unitDisparity * 16;
        bm->setROI1(validROI_L);
        bm->setROI2(validROI_R);
        bm->setPreFilterCap(13);
        bm->setBlockSize(19);       // SAD窗口大小,设置为奇数
        bm->setMinDisparity(0);  // 最小视差
        // 在该数值确定的视差范围内进行搜索,
        // 视差窗口,即最大视差值与最小视差值之差, 大小必须是16的整数倍
        bm->setNumDisparities(numberOfDisparities);
        bm->setTextureThreshold(1000); // 保证有足够的纹理来克服噪声
        bm->setUniquenessRatio(1);
        bm->setSpeckleWindowSize(200); //检查视差连通区域变化度的窗口大小, 值为0时取消 speckle 检查
        bm->setSpeckleRange(32); // 视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零
        // 左视差图（直接计算得出）和右视差图(通过cvValidateDisparity计算得出)之间的最大容许差异，默认为-1
        bm->setDisp12MaxDiff(-1);
    }
};


#endif //CoreBM_H
