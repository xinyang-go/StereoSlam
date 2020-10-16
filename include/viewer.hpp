//
// Created by xinyang on 2020/10/8.
//

#ifndef MYSLAM_VIEWER_HPP
#define MYSLAM_VIEWER_HPP

#include "defs.hpp"

class Viewer {
public:
    Viewer();

    int operator()(const Frame::sPtr &frame, int delay = 0);

private:
    cv::viz::Viz3d vis;
    cv::viz::WCoordinateSystem world_coor, camera_coor;
};


#endif //MYSLAM_VIEWER_HPP
