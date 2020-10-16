//
// Created by xinyang on 2020/10/16.
//

#include "backend.hpp"
#include "config.hpp"

Frame::sPtr Backend::operator()(const Frame::sPtr &frame) {
    if (is_key_frame(frame)) {
        key_frames.emplace_front(frame);
        if (key_frames.size() > Config::max_key_frames) {
            key_frames.pop_back();
        }
    }

    return frame;
}

bool Backend::is_key_frame(const Frame::sPtr &frame) {
    if (key_frames.empty()) return true;
    Eigen::Vector3d dist = frame->Tcw.translation() - key_frames.front()->Tcw.translation();
    return dist.norm() > 50;
}
