#include "./gtsam_lib.hpp"

/**
 * @brief  添加顶点
 * @note 可选择是否固定先验边
 * @todo
 **/
void GtsamOpter::AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix)
{
    gtsam::Key key = estimates_.size();
    gtsam::Pose3 p(pose.matrix());
    estimates_.insert(key, p);

    if (need_fix)
    {
        gtsam::Vector6 fixed_noise;
        fixed_noise << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(fixed_noise);
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(key, p, prior_noise));
    }
}

/**
 * @brief  添加邻边
 * @note
 * @todo
 **/
void GtsamOpter::AddSe3Edge(int vertex_index1,
                            int vertex_index2,
                            const Eigen::Isometry3d &relative_pose,
                            const Eigen::Vector3d &translation_noise,
                            const Eigen::Vector3d &rotation_noise)
{
    /*测量*/
    gtsam::Pose3 measurement(relative_pose.matrix());
    /*噪声*/
    Eigen::Matrix<double, 6, 1> noise;
    noise << rotation_noise, translation_noise;
    gtsam::noiseModel::Diagonal::shared_ptr noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6(noise));
    /*添加*/
    if (need_robust_kernel_)
    {
        gtsam::noiseModel::Robust::shared_ptr robust_loss;
        if (robust_kernel_name_ == "Huber")
        {
            robust_loss = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(robust_kernel_size_), noise_model);
        }
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(vertex_index1, vertex_index2, measurement, robust_loss));
    }
    else
    {
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(vertex_index1, vertex_index2, measurement, noise_model));
    }
}

/**
 * @brief  添加单边
 * @note
 * @todo
 **/
void GtsamOpter::AddSe3PriorXYZEdge(int se3_vertex_index,
                                    const Eigen::Vector3d &xyz,
                                    const Eigen::Vector3d &translation_noise)
{
    /*测量*/
    gtsam::Vector3 measurement(xyz);
    /*噪声*/
    gtsam::noiseModel::Diagonal::shared_ptr unaryNoise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(translation_noise));
    /*添加*/
    graph_.add(gtsam::PoseTranslationPrior<gtsam::Pose3>(se3_vertex_index, measurement, unaryNoise));
}

/**
 * @brief  执行优化
 * @note
 * @todo
 **/
void GtsamOpter::Optimize()
{
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosity("SILENT");                                                          // 打印优化选项
    params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");                                    // 线性求解器选项
    estimates_ = gtsam::LevenbergMarquardtOptimizer(graph_, estimates_, params).optimize(); // 优化，并更新初值
}

