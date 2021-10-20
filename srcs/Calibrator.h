#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "ChessBoard.h"
#include "config.h"
#include "utils.h"

/*　立体标定　*/
class Calibrator {
public:
    cv::Size imgSize;       // 图像大小 1280x720
    cv::Size patSize;       // 棋盘格 cols x rows
    double squareSize;  // 方格尺寸 5.f
    cv::Mat M_L, M_R;       // 内参矩阵(左/右)
    cv::Mat D_L, D_R;       // 畸变系数(左/右)
    std::vector<cv::Mat> rvecs_L, rvecs_R;  // 旋转矩阵(左/右)
    std::vector<cv::Mat> tvecs_L, tvecs_R;  // 平移矩阵(左/右)
    double error_L = 0., error_R = 0.; // 标定误差(单目)

    cv::Mat R, T;          // 旋转矩阵（R）+矩阵（T）
    cv::Mat E, F;          // 本质矩阵（）
    double error_LR = 0.;  // 标定误差（双目）

    bool is_calib = false; // 是否完成标定

private:
    // 存储图像
    cv::Mat img, imgL, imgR, imgGrayL, imgGrayR, imgShow;
    std::vector<cv::Mat> imgsL, imgsR;
    // 标定图片
    vector<std::string> calib_files, calib_files_good;
    // 世界坐标
    std::vector<cv::Point3f> corners_xyz;
    std::vector<std::vector<cv::Point2f>> corners_uv_all_L, corners_uv_all_R;
    // 像素坐标
    std::vector<cv::Point2f> corners_uv_L, corners_uv_R;
    std::vector<std::vector<cv::Point3f>> corners_xyz_all;

public:
    bool calibStereo(const std::string &folder,
                     int W, int H,
                     ChessBoard &chessBoard,
                     bool show = true){
        // 读取标定图片
        glob(folder, calib_files);
        for (auto &file : calib_files){
            // 现在图片是左右合在一起，需要切分为两部分
            imread2(file, img);
            d2(img, imgL, imgR);
            imgsL.push_back(imgL);
            imgsR.push_back(imgR);
        }

        imgSize = cv::Size(W, H);   //图像尺寸
        patSize = cv::Size(chessBoard.cols, chessBoard.rows);   // 棋盘格列和行
        squareSize = chessBoard.square_size; assert(squareSize > 0);

        // 棋盘格角点世界坐标
        // 一般将标定板平面设为世界坐标系的z=0平面，将标定板的左上角作为世界坐标系的原点
        for (int y = 0; y < patSize.height; y++){
            for (int x = 0; x < patSize.width; x++){
                corners_xyz.emplace_back(cv::Point3f(float(x * squareSize), float(y * squareSize), 0));
            }
        }

        // 每张图片检测角点
        std::cout << "开始检测角点..." << std::endl;
        for (int i = 0; i < int(calib_files.size()); i++) {
            std::cout << "读取:" << calib_files[i];
            imgL = imgsL[i]; imgR = imgsR[i];
            bool find = detectCorners(show);
            // 如果左右相机都检测到角点
            if (find){
                calib_files_good.emplace_back(calib_files[i]);
                corners_xyz_all.emplace_back(corners_xyz);
                corners_uv_all_L.emplace_back(corners_uv_L);
                corners_uv_all_R.emplace_back(corners_uv_R);
            }
            std::cout << "\t检测：\t" << find << std::endl;
        }
        std::cout << "角点检测完成!" << std::endl;

        std::cout << "开始单目标定..." << std::endl;
        // 左、右相机分别进行单目标定
        error_L = calibrateCamera(corners_xyz_all,  // 世界坐标
                                  corners_uv_all_L, // 像素坐标
                                  imgSize,            // 标定图像大小
                                  M_L,             // 相机内参
                                  D_L,                // 畸变系数
                                  rvecs_L,                // 旋转矩阵
                                  tvecs_L,                 // 平移矩阵
                                  cv::CALIB_FIX_K3);      // 鱼眼相机(k1,k2,k3)

        error_R = calibrateCamera(corners_xyz_all,
                                  corners_uv_all_R,
                                  imgSize,
                                  M_R,
                                  D_R,
                                  rvecs_R,
                                  tvecs_R,
                                  cv::CALIB_FIX_K3);
        printInfo();
        std::cout << "单目标定完成!" << std::endl;


        std::cout << "开始立体标定..." << std::endl;
        // 双目标定
        error_LR = stereoCalibrate(corners_xyz_all,      // 标定板世界坐标
                                   corners_uv_all_L,    // 像素坐标(左)
                                   corners_uv_all_R,    // 像素坐标(右)
                                   M_L,D_L,  // 相机内参\畸变系数(左)
                                   M_R, D_R, // 相机内参\畸变系数(右)
                                   imgSize,                // 标定图像大小
                                   R,    // 旋转矩阵(右->左)
                                   T,    // 平移矩阵(右->左)
                                   E,    // 本质矩阵(同一点在左\右像素坐标系下的相互转换矩阵,单位mm)
                                   F,    // 基础矩阵
                                   cv::CALIB_FIX_INTRINSIC,
                                   TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5));
        printStereoInfo();
        is_calib = true;
        std::cout << "立体标定完成!" << std::endl;
        return is_calib;
    }

    bool save2xml(const std::string &intrinsic_file, const std::string &extrinsic_file) const{
        std::cout << "写入内参到文件:" << intrinsic_file << std::endl;
        FileStorage fw1(intrinsic_file, FileStorage::WRITE);
        if (fw1.isOpened()){
            fw1 << "M_L" << M_L << "D_L" << D_L <<
                "M_R" << M_R << "D_R" << D_R;
            fw1.release();
        } else{
            std::cerr << "无法打开内参文件:" << intrinsic_file << std::endl; assert(0);
        }
        std::cout << "写入外参到文件:" << extrinsic_file << std::endl;
        FileStorage fw2(extrinsic_file, FileStorage::WRITE);
        if (fw2.isOpened()){
            fw2 << "R" << R  << "T" << T;
            fw2.release();
        }
        else{
            std::cerr << "无法打开外参文件:" << extrinsic_file << std::endl; assert(0);
        }
        return true;
    }

private:
    /*　检测当前imgL\imgR中角点　*/
    bool detectCorners(bool show = true) {
        // 转换为灰度图像
        cvtColor(imgL, imgGrayL, COLOR_BGR2GRAY);
        cvtColor(imgR, imgGrayR, COLOR_BGR2GRAY);
        // 角点粗检测
        bool findL = findChessboardCorners(imgGrayL, patSize, corners_uv_L,
                                           CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
        bool findR = findChessboardCorners(imgGrayR, patSize, corners_uv_R,
                                           CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
        if (findL && findR) {
            // 角点精检测
            cornerSubPix(imgGrayL, corners_uv_L, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 300, 0.01));
            cornerSubPix(imgGrayR, corners_uv_R, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 300, 0.01));
        }
        if (show) {
            // 显示检测信息
            drawChessboardCorners(imgL, patSize, corners_uv_L, findL);
            drawChessboardCorners(imgR, patSize, corners_uv_R, findR);
            hconcat(imgL, imgR, imgShow);
            imshow("chessBoard", imgShow);
            waitKey(1000);
            destroyWindow("chessBoard");
        }
        return (findL && findR);
    }

    /* 打印单目标定信息 */
    void printInfo() const{
        std::cout << "标定误差(L):" << error_L << std::endl;
        std::cout << "标定误差(R):" << error_R << std::endl;
        std::cout << "\n内参(左)\n" << M_L << std::endl;
        std::cout << "\n内参(右)\n" << M_R << std::endl;
        std::cout << "\n畸变(左)\n" << D_L << std::endl;
        std::cout << "\n畸变(右)\n" << D_R << std::endl;
    }

    /* 打印双目标定信息 */
    void printStereoInfo() const{
        std::cout << "标定误差(双目):" << error_LR << std::endl;
        std::cout << "\n旋转矩阵R\n" << R << std::endl;
        std::cout << "\n平移矩阵T\n" << T << std::endl;
        std::cout << "\n本质矩阵E(相机坐标系，mm)\n" << E << std::endl;
        std::cout << "\n基础矩阵F(像素坐标系，pixel)\n" << F << std::endl;
    }
};


#endif //CALIBRATOR_H
