//
// Created by xinyang on 2020/10/14.
//

#ifndef STEREOSLAM_FRONTEND_HPP
#define STEREOSLAM_FRONTEND_HPP

#include "defs.hpp"

class Frontend {
public:
    Frontend();

    Frame::sPtr operator()(const Frame::sPtr &frame);

private:
    Frame::sPtr history_frame;
    Sophus::SE3d relative;
};

#endif //STEREOSLAM_FRONTEND_HPP
