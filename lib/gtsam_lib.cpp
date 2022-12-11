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

// c++ lib

// gtsam lb

// /*
// 模拟lio sam 后端优化
// */

// class UnaryFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2>
// {
//     double mx_, my_; ///< X and Y measurements

// public:
//     UnaryFactor(gtsam::Key j, double x, double y, const gtsam::SharedNoiseModel &model) : gtsam::NoiseModelFactor1<gtsam::Pose2>(model, j), mx_(x), my_(y) {}

//     gtsam::Vector evaluateError(const gtsam::Pose2 &q,
//                                 boost::optional<gtsam::Matrix &> H = boost::none) const
//     {
//         const gtsam::Rot2 &R = q.rotation();
//         if (H)
//         {
//             (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0,
//                     R.s(), R.c(), 0.0)
//                        .finished();
//         }
//         return (gtsam::Vector(2) << q.x() - mx_, q.y() - my_).finished();
//     }
// };

// int main(int argc, char **argv)
// {
//     // 1. 创建因子图
//     gtsam::NonlinearFactorGraph graph;

//     // 2. 添加先验约束
//     gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
//         gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));

//     graph.add(gtsam::PriorFactor<gtsam::Pose2>(1, gtsam::Pose2(0, 0, 0), priorNoise));

//     // 3.添加imu里程计提供的帧间约束
//     gtsam::noiseModel::Diagonal::shared_ptr imuOdomNoise =
//         gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));

//     graph.add(gtsam::BetweenFactor<gtsam::Pose2>(1, 2, gtsam::Pose2(2, 0, 0), imuOdomNoise));
//     graph.add(gtsam::BetweenFactor<gtsam::Pose2>(2, 3, gtsam::Pose2(2, 0, M_PI_2), imuOdomNoise));
//     graph.add(gtsam::BetweenFactor<gtsam::Pose2>(3, 4, gtsam::Pose2(2, 0, M_PI_2), imuOdomNoise));
//     graph.add(gtsam::BetweenFactor<gtsam::Pose2>(4, 5, gtsam::Pose2(2, 0, M_PI_2), imuOdomNoise));

//     // 4.添加 回环检测提供的回环约束
//         gtsam::noiseModel::Diagonal::shared_ptr loopClosureNoise =
//         gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
//     graph.add(gtsam::BetweenFactor<gtsam::Pose2>(4, 5, gtsam::Pose2(2, 0, M_PI_2), loopClosureNoise));

//     // 5. 添加gps提供的单边约束
//     gtsam::noiseModel::Diagonal::shared_ptr unaryNoise =
//         gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1));

//     graph.add(boost::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));

//     // 6. 设置位姿初值 即前端里程计
//     gtsam::Values initial;
//     initial.insert(1, gtsam::Pose2(0.0, 0.0, 0.0));
//     initial.insert(2, gtsam::Pose2(1.9, 0.0, 0.0));
//     initial.insert(3, gtsam::Pose2(4.1, 0.0, 0.0));
//     initial.insert(4, gtsam::Pose2(4.1, 2.1, 0.0));
//     initial.insert(5, gtsam::Pose2(2.0, 1.9, 0.0));

//     // 7. 调用优化器
//     gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//     result.print();

//     // 8. 后验概率
//     std::cout.precision(2);
//     gtsam::Marginals marginals(graph, result);
//     std::cout << "x1 covariance:\n"
//               << marginals.marginalCovariance(1) << std::endl;
//     std::cout << "x2 covariance:\n"
//               << marginals.marginalCovariance(2) << std::endl;
//     std::cout << "x3 covariance:\n"
//               << marginals.marginalCovariance(3) << std::endl;
//     std::cout << "x4 covariance:\n"
//               << marginals.marginalCovariance(4) << std::endl;
//     std::cout << "x5 covariance:\n"
//               << marginals.marginalCovariance(5) << std::endl;

//         //9.将优化的变量节点的均值更新到原来的节点向量中
// }