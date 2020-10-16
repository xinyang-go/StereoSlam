//
// Created by xinyang on 2020/10/9.
//

#ifndef STEREO_SLAM_CONFIG_HPP
#define STEREO_SLAM_CONFIG_HPP

#include "common.hpp"

class Config {
private:
    static cv::FileStorage fs;

    template<class T>
    static T config_get(std::string_view node) {
        T val;
        auto key = fs[node.begin()];
        assert(!key.empty());
        key >> val;
        return val;
    }

    template<class Eigen_T>
    static Eigen_T to_eigen(const cv::Mat &m) {
        Eigen_T v;
        cv::cv2eigen(m, v);
        return v;
    }

public:
    // parameters
    static const int max_frame_points;
    static const int min_frame_points;
    static const int max_key_frames;
    static const cv::Mat K0, K1;
    static const cv::Mat C0, C1;
    static const double fx_left, fy_left, cx_left, cy_left;
    static const double fx_right, fy_right, cx_right, cy_right;
    static const cv::Mat R_01, t_01;
    static const Sophus::SE3d T_01;
    static const Sophus::SE3d T_10;
};

#endif //STEREO_SLAM_CONFIG_HPP
