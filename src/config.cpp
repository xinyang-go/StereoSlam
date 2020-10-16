//
// Created by xinyang on 2020/10/9.
//

#include "config.hpp"

cv::FileStorage Config::fs(PROJECT_DIR"/config/param.yml", cv::FileStorage::READ);

const int Config::max_frame_points(config_get<int>("max_frame_points"));

const int Config::min_frame_points(config_get<int>("min_frame_points"));

const int Config::max_key_frames(config_get<int>("max_key_frames"));

const cv::Mat Config::K0(config_get<cv::Mat>("K0"));

const cv::Mat Config::K1(config_get<cv::Mat>("K1"));

const cv::Mat Config::C0(config_get<cv::Mat>("C0"));

const cv::Mat Config::C1(config_get<cv::Mat>("C1"));

const double Config::fx_left(K0.at<double>(0, 0));

const double Config::fy_left(K0.at<double>(1, 1));

const double Config::cx_left(K0.at<double>(0, 2));

const double Config::cy_left(K0.at<double>(1, 2));

const double Config::fx_right(K1.at<double>(0, 0));

const double Config::fy_right(K1.at<double>(1, 1));

const double Config::cx_right(K1.at<double>(0, 2));

const double Config::cy_right(K1.at<double>(1, 2));

const cv::Mat Config::R_01(config_get<cv::Mat>("R_01"));

const cv::Mat Config::t_01(config_get<cv::Mat>("t_01"));

const Sophus::SE3d Config::T_01(to_eigen<Eigen::Matrix3d>(R_01), to_eigen<Eigen::Vector3d>(t_01));

const Sophus::SE3d Config::T_10(T_01.inverse());
