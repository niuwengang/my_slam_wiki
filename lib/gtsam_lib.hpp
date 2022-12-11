
#ifndef GTSAM_LIB_HPP_
#define GTSAM_LIB_HPP_

//gtsam 4.0 lib
// #include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
// #include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/nonlinear/Marginals.h>

//c++ lib
#include <vector>
//eigen lib
#include <Eigen/Core>
#include <Eigen/Dense>

class GtsamOpter
{
public:
    void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix);
    void AddSe3Edge(int vertex_index1,
                    int vertex_index2,
                    const Eigen::Isometry3d &relative_pose,
                    const Eigen::Vector3d &translation_noise,
                    const Eigen::Vector3d &rotation_noise);
    void AddSe3PriorXYZEdge(int se3_vertex_index,
                            const Eigen::Vector3d &xyz,
                            const Eigen::Vector3d &translation_noise);

    void Optimize();

public:
    gtsam::Values estimates_; // 估计值
    gtsam::NonlinearFactorGraph graph_;//优化器
    /*鲁棒核*/
    bool need_robust_kernel_=false;
    std::string robust_kernel_name_="";
    size_t robust_kernel_size_=0;
};

#endif // GTSAM_LIB_HPP_

