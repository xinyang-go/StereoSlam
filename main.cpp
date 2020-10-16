#include <iostream>
#include <boost/timer/timer.hpp>
#include "frontend.hpp"
#include "viewer.hpp"

int main() {
    cv::VideoCapture v0("/home/xinyang/Videos/stereo/0.avi");
    cv::VideoCapture v1("/home/xinyang/Videos/stereo/1.avi");

    Frontend frontend;
    Viewer viewer;

    Frame::sPtr frame;
    int delay = 0;
    while (true) {
        cv::Mat src0, src1;
        v0 >> src0;
        v1 >> src1;
        try {
            cv::resize(src0, src0, {640, 512});
            cv::resize(src1, src1, {640, 512});
        } catch (const cv::Exception &e) {
            break;
        }
        frame = Frame::create(src0, src1);
        {
            boost::timer::auto_cpu_timer timer("frontend: %ws\n");
            frame = frontend(frame);
        }

        std::cout << "=============" << std::endl;
        int k = viewer(frame, delay);
        if (k == 'z') delay = 10 - delay;
        else if (k == 'q') break;
    }

    return 0;
}
