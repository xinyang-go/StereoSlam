//
// Created by xinyang on 2020/10/8.
//

#include "viewer.hpp"
#include "config.hpp"

Viewer::Viewer() : vis("Visual Odometry"), world_coor(1.0), camera_coor(0.5) {
    cv::Point3d cam_pos(0, -3.0, -3.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose(cam_pose);
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("World", world_coor);
    vis.showWidget("Camera", camera_coor);
}

int Viewer::operator()(const Frame::sPtr &frame, int delay) {
    std::cout << "R:\n" << frame->Tcw.rotationMatrix() << std::endl;
    std::cout << "t    :\n" << frame->Tcw.translation() << std::endl;
    auto Twc = frame->Tcw.inverse();
    cv::Affine3d M(
            cv::Affine3d::Mat3(
                    Twc.rotationMatrix()(0, 0), Twc.rotationMatrix()(0, 1), Twc.rotationMatrix()(0, 2),
                    Twc.rotationMatrix()(1, 0), Twc.rotationMatrix()(1, 1), Twc.rotationMatrix()(1, 2),
                    Twc.rotationMatrix()(2, 0), Twc.rotationMatrix()(2, 1), Twc.rotationMatrix()(2, 2)
            ),
            cv::Affine3d::Vec3(
                    Twc.translation()(0, 0) / 1000.0, Twc.translation()(1, 0) / 1000.0, Twc.translation()(2, 0) / 1000.0
            )
    );
    vis.setWidgetPose("Camera", M);

    cv::Mat im2show_left = frame->left_img.clone();
    for (const auto &fp:frame->left_fps) {
        cv::circle(im2show_left, cv::Point(fp->pt.x(), fp->pt.y()), 3, {0, 0, 255}, cv::FILLED);
        if (fp->mp) {
            Eigen::Vector3d pc = frame->Tcw * fp->mp->pt;
            double dist = pc.z();
            cv::Point2d pu(pc.x() / dist * Config::fx_left + Config::cx_left,
                           pc.y() / dist * Config::fy_left + Config::cy_left);
            cv::circle(im2show_left, pu, 3, {0, 255, 0}, cv::FILLED);
            cv::putText(im2show_left, std::to_string(dist), pu,
                        cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);
        }
    }
    cv::imshow("left", im2show_left);

    cv::Mat im2show_right = frame->right_img.clone();
    for (const auto &fp:frame->right_fps) {
        cv::circle(im2show_right, cv::Point(fp->pt.x(), fp->pt.y()), 3, {0, 0, 255}, cv::FILLED);
        if (fp->mp) {
            Eigen::Vector3d pc = Config::T_10 * frame->Tcw * fp->mp->pt;
            double dist = pc.z();
            cv::Point2d pu(pc.x() / dist * Config::fx_left + Config::cx_left,
                           pc.y() / dist * Config::fy_left + Config::cy_left);
            cv::circle(im2show_right, pu, 3, {0, 255, 0}, cv::FILLED);
            cv::putText(im2show_right, std::to_string(dist), pu,
                        cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);
        }
    }
    cv::imshow("right", im2show_right);

    vis.spinOnce(delay, false);
    return cv::waitKey(delay);
}
