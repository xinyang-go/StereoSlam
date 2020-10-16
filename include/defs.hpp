//
// Created by xinyang on 2020/10/14.
//

#ifndef STEREOSLAM_DEFS_HPP
#define STEREOSLAM_DEFS_HPP

#include "common.hpp"
#include "utils.hpp"

class MapPoint;

class FeaturePoint;

class Frame;

class MapPoint : public EnableBasicTypes<MapPoint> {
protected:
    MapPoint(double _x, double _y, double _z) : pt(_x, _y, _z) {}

public:
    Eigen::Vector3d pt;
    EnableBasicTypes<FeaturePoint>::wMap pts;
};

class FeaturePoint : public EnableBasicTypes<FeaturePoint> {
protected:
    explicit FeaturePoint(double _x, double _y) : pt(_x, _y) {}

    ~FeaturePoint() { if (mp) { mp->pts.erase(id()); }}

    void registerToMapPoint() { if (mp) { mp->pts.emplace(id(), shared_from_this()); }}

public:
    Eigen::Vector2d pt;
    MapPoint::sPtr mp;
    EnableBasicTypes<Frame>::wPtr frame;
};

class Frame : public EnableBasicTypes<Frame> {
protected:
    Frame(cv::Mat l, cv::Mat r, double t = 0) : left_img(std::move(l)), right_img(std::move(r)),
                                                timestamp(t), is_key_frame(false) {}

public:
    cv::Mat left_img, right_img;
    double timestamp;
    FeaturePoint::sLst left_fps, right_fps;
    Sophus::SE3d Tcw;
    bool is_key_frame;
};

#endif //STEREOSLAM_DEFS_HPP
