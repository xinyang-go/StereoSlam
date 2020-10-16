//
// Created by xinyang on 2020/10/8.
//

#ifndef MYSLAM_COMMON_HPP
#define MYSLAM_COMMON_HPP

// multi-thread
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>

// stl
#include <unordered_map>
#include <vector>
#include <array>
#include <list>

// eigen
#include <Eigen/Dense>

// sophus
#include <sophus/se3.hpp>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>

// g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#endif //MYSLAM_COMMON_HPP
