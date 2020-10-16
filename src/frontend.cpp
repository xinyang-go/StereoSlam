//
// Created by xinyang on 2020/10/14.
//

#include "frontend.hpp"
#include "config.hpp"
#include "boost/timer/timer.hpp"
#include <fmt/format.h>

static void trackFeaturePoints(const cv::Mat &img0, FeaturePoint::sLst &fps0, bool remove0,
                               const cv::Mat &img1, FeaturePoint::sLst &fps1, bool remove1,
                               double err_threshold = 2.) {
    // 使用静态变量容器，减少内存分配时间
    static std::vector<cv::Point2f> pts0;
    static std::vector<cv::Point2f> pts1;
    static std::vector<FeaturePoint::sLst::iterator> fps0_iters;
    static std::vector<FeaturePoint::sLst::iterator> fps1_iters;
    static std::vector<uint8_t> status;
    static std::vector<float> errors;
    pts0.clear();
    pts1.clear();
    fps0_iters.clear();
    fps1_iters.clear();
    status.clear();
    errors.clear();

    /// FOR DEBUG
    assert(fps1.empty() || fps0.size() == fps1.size());

    // 准备输入数据
    bool check_points = !fps1.empty();
    size_t fps_size = fps0.size();
    fps1.resize(fps_size);
    for (auto fp0 = fps0.begin(); fp0 != fps0.end(); fp0++) {
        pts0.emplace_back((*fp0)->pt.x(), (*fp0)->pt.y());
        fps0_iters.emplace_back(fp0);
    }
    for (auto fp1 = fps1.begin(); fp1 != fps1.end(); fp1++) {
        fps1_iters.emplace_back(fp1);
    }
    // 进行光流追踪
    cv::calcOpticalFlowPyrLK(img0, img1, pts0, pts1, status, errors, {21, 21}, 3,
                             {cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01});
    // 保存数据
    size_t track_error = 0;
    size_t check_error = 0;

    for (size_t i = 0; i < fps_size; i++) {
        // 追踪失败
        if (status[i] == false) {
            if (remove0) fps0.erase(fps0_iters[i]);
            if (remove1) fps1.erase(fps1_iters[i]);
            track_error++;
            continue;
        }
        // 双向追踪误差判断
        if (check_points) {
            double dx = (*fps1_iters[i])->pt.x() - pts1[i].x;
            double dy = (*fps1_iters[i])->pt.y() - pts1[i].y;
            if (std::sqrt(dx * dx + dy * dy) > err_threshold) {
                if (remove0) fps0.erase(fps0_iters[i]);
                if (remove1) fps1.erase(fps1_iters[i]);
                check_error++;
                continue;
            }
        }
        // 更新数据
        if (*fps1_iters[i]) {
            (*fps1_iters[i])->pt.x() = pts1[i].x;
            (*fps1_iters[i])->pt.y() = pts1[i].y;
        } else {
            auto fp1 = FeaturePoint::create(pts1[i].x, pts1[i].y);
            fp1->mp = (*fps0_iters[i])->mp;
            *fps1_iters[i] = std::move(fp1);
        }
    }
    std::cout << fmt::format("{} points to track. {} track error, {} check error. get {} points.",
                             pts0.size(), track_error, check_error, pts0.size() - track_error - check_error)
              << std::endl;
}

bool estimatePoseWithPnP(const Frame::sPtr &frame) {
    // 使用静态变量容器，减少内存分配时间
    static std::vector<cv::Point2f> pts_2d;
    static std::vector<cv::Point3f> pts_3d;
    static std::vector<FeaturePoint::sLst::iterator> fps_iters;
    static std::vector<int> inliers;
    static cv::Mat rvec, tvec, rmat;
    pts_2d.clear();
    pts_3d.clear();
    fps_iters.clear();
    inliers.clear();

    // 准备输入数据
    for (auto fp = frame->left_fps.begin(); fp != frame->left_fps.end(); fp++) {
        pts_2d.emplace_back((*fp)->pt.x(), (*fp)->pt.y());
        pts_3d.emplace_back((*fp)->mp->pt.x(), (*fp)->mp->pt.y(), (*fp)->mp->pt.z());
        fps_iters.emplace_back(fp);
    }
    // 解PnP
    std::cout << fmt::format("pts_3d.size()={}, pts_2d.size()={}", pts_3d.size(), pts_2d.size()) << std::endl;
    try {
        if (!cv::solvePnPRansac(pts_3d, pts_2d, Config::K0, Config::C0, rvec, tvec,
                                false, 100, 1.0, 0.99, inliers)) {
            std::cout << "pnp solve fail." << std::endl;
            return false;
        }
    } catch (const cv::Exception &e) {
        std::cout << "pnp solve error: " << e.what() << std::endl;
        return false;
    }
    // 保存姿态数据
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    cv::cv2eigen(rmat, R);
    cv::cv2eigen(tvec, t);
    frame->Tcw = {R, t};
    // 剔除外点
    for (int i = 0, j = 0; i < inliers.size(); i++, j++) {
        for (; j < inliers[i]; j++) frame->left_fps.erase(fps_iters[j]);
    }
    return true;
}

void detectFeaturePoints(const cv::Mat &img, FeaturePoint::sLst &fps) {
    // 使用静态变量容器，减少内存分配时间
    static cv::Mat mask(img.size(), CV_8U);
    static std::vector<cv::KeyPoint> kps;
    kps.clear();

    // 为已有特征点的位置创建mask
    if (mask.size() != img.size()) mask = cv::Mat(img.size(), CV_8U, 255);
    else mask.setTo(255);
    for (const auto &fp: fps) {
        cv::rectangle(mask, cv::Rect(fp->pt.x() - 10, fp->pt.y() - 10, 20, 20), 0, cv::FILLED);
    }
    // 进行特征点检测
    int new_features_size = Config::max_frame_points - static_cast<int>(fps.size());
    if (new_features_size <= 0) return;
    cv::Ptr<cv::FeatureDetector> detector = cv::GFTTDetector::create(
            new_features_size, 0.02, 20, 3);
    detector->detect(img, kps, mask);
    // 保存检测结果
    for (const auto &kp: kps) {
        auto fp = FeaturePoint::create(kp.pt.x, kp.pt.y);
        fps.emplace_back(std::move(fp));
    }
}

void triangulateFeaturePoints(const Frame::sPtr &frame) {
    // 使用静态变量容器，减少内存分配时间
    static std::vector<cv::Point2f> pts0;
    static std::vector<cv::Point2f> pts1;
    static std::vector<FeaturePoint::sLst::iterator> fps0_iters;
    static std::vector<FeaturePoint::sLst::iterator> fps1_iters;
    pts0.clear();
    pts1.clear();
    fps0_iters.clear();
    fps1_iters.clear();

    // 准备输入数据
    for (auto lfp = frame->left_fps.begin(), rfp = frame->right_fps.begin();
         lfp != frame->left_fps.end() && rfp != frame->right_fps.end(); lfp++, rfp++) {
        double lx = (*lfp)->pt.x();
        double ly = (*lfp)->pt.y();
        double rx = (*rfp)->pt.x();
        double ry = (*rfp)->pt.y();
        pts0.emplace_back((lx - Config::cx_left) / Config::fx_left,
                          (ly - Config::cy_left) / Config::fy_left);
        pts1.emplace_back((rx - Config::cx_right) / Config::fx_right,
                          (ry - Config::cy_right) / Config::fy_right);
        fps0_iters.emplace_back(lfp);
        fps1_iters.emplace_back(rfp);
    }
    cv::Mat T_left, T_right;
    Eigen::Matrix<float, 3, 4> T1, T2;
    T1 = frame->Tcw.matrix().topRows(3).cast<float>();
    T2 = (Config::T_10 * frame->Tcw).matrix().topRows(3).cast<float>();
    cv::eigen2cv(T1, T_left);
    cv::eigen2cv(T2, T_right);
    // 进行三角测量
    cv::Mat pts_4d;
    cv::triangulatePoints(T_left, T_right, pts0, pts1, pts_4d);
    // 保存数据并剔除误点
    for (int i = 0; i < pts_4d.cols; i++) {
        Eigen::Vector3d pt{pts_4d.at<float>(0, i) / pts_4d.at<float>(3, i),
                           pts_4d.at<float>(1, i) / pts_4d.at<float>(3, i),
                           pts_4d.at<float>(2, i) / pts_4d.at<float>(3, i)};

        /// TODO: 剔除误点

        // 更新3d坐标点
        auto &mp0 = (*fps0_iters[i])->mp;
        auto &mp1 = (*fps1_iters[i])->mp;
        if (mp0) {
            // 已有世界坐标点，则使用滑动平均更新3d坐标
            mp0->pt = mp0->pt * 0.8 + pt * 0.2;
        } else {
            // 暂无世界坐标点，则创建
            auto new_mp = MapPoint::create(pt.x(), pt.y(), pt.z());
            mp0 = mp1 = new_mp;
        }
    }
}

Frontend::Frontend() {

}

Frame::sPtr Frontend::operator()(const Frame::sPtr &frame) {
    if (history_frame) {
        // 如果有历史帧
        // 追踪历史特征点
        {
            boost::timer::auto_cpu_timer timer("history->current: %ws\n");
            trackFeaturePoints(history_frame->left_img, history_frame->left_fps, true,
                               frame->left_img, frame->left_fps, true);
        }
        // 反向追踪特征点以剔除误追踪
        {
            boost::timer::auto_cpu_timer timer("current->history: %ws\n");
            trackFeaturePoints(frame->left_img, frame->left_fps, true,
                               history_frame->left_img, history_frame->left_fps, true);
        }
        // PnP位姿估计
        {
            boost::timer::auto_cpu_timer timer("PnP: %ws\n");
            estimatePoseWithPnP(frame);
        }
    }
    // 补充特征点
    {
        boost::timer::auto_cpu_timer timer("detect: %ws\n");
        detectFeaturePoints(frame->left_img, frame->left_fps);
    }
    // 右侧相机追踪左侧相机特征点
    {
        boost::timer::auto_cpu_timer timer("left->right: %ws\n");
        trackFeaturePoints(frame->left_img, frame->left_fps, true, frame->right_img, frame->right_fps, true);
    }
    // 反向追踪特征点以剔除误追踪
    {
        boost::timer::auto_cpu_timer timer("right->left: %ws\n");
        trackFeaturePoints(frame->left_img, frame->left_fps, true, frame->right_img, frame->right_fps, true);
    }

    /// FOR DEBUG
    assert(frame->left_fps.size() == frame->right_fps.size());

    // 三角测量特征点3d坐标
    {
        boost::timer::auto_cpu_timer timer("triangulate: %ws\n");
        triangulateFeaturePoints(frame);
    }

    if (history_frame) relative = frame->Tcw * history_frame->Tcw.inverse();
    history_frame = frame;
    return history_frame;
}
