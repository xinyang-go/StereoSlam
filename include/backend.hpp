//
// Created by xinyang on 2020/10/16.
//

#ifndef STEREOSLAM_BACKEND_HPP
#define STEREOSLAM_BACKEND_HPP

#include "defs.hpp"

class Backend {
public:
    Frame::sPtr operator()(const Frame::sPtr &frame);

private:
    bool is_key_frame(const Frame::sPtr &frame);

    Frame::sLst key_frames;
};

#endif //STEREOSLAM_BACKEND_HPP
