// c++ lib
#include <bits/stdc++.h>
// gtsam lb

#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

/*
模拟lio sam 后端优化
*/

class UnaryFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2>
{
    double mx_, my_; ///< X and Y measurements

public:
    UnaryFactor(gtsam::Key j, double x, double y, const gtsam::SharedNoiseModel &model) : gtsam::NoiseModelFactor1<gtsam::Pose2>(model, j), mx_(x), my_(y) {}

    gtsam::Vector evaluateError(const gtsam::Pose2 &q,
                                boost::optional<gtsam::Matrix &> H = boost::none) const
    {
        const gtsam::Rot2 &R = q.rotation();
        if (H)
        {
            (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0,
                    R.s(), R.c(), 0.0)
                       .finished();
        }
        return (gtsam::Vector(2) << q.x() - mx_, q.y() - my_).finished();
    }
};

int main(int argc, char **argv)
{
    // 1. 创建因子图
    gtsam::NonlinearFactorGraph graph;

    // 2. 添加先验约束
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));

    graph.add(gtsam::PriorFactor<gtsam::Pose2>(1, gtsam::Pose2(0, 0, 0), priorNoise));

    // 3.添加imu里程计提供的帧间约束
    gtsam::noiseModel::Diagonal::shared_ptr imuOdomNoise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));

    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(1, 2, gtsam::Pose2(2, 0, 0), imuOdomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(2, 3, gtsam::Pose2(2, 0, M_PI_2), imuOdomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(3, 4, gtsam::Pose2(2, 0, M_PI_2), imuOdomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(4, 5, gtsam::Pose2(2, 0, M_PI_2), imuOdomNoise));

    // 4.添加 回环检测提供的回环约束
        gtsam::noiseModel::Diagonal::shared_ptr loopClosureNoise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(4, 5, gtsam::Pose2(2, 0, M_PI_2), loopClosureNoise));

    // 5. 添加gps提供的单边约束
    gtsam::noiseModel::Diagonal::shared_ptr unaryNoise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1));

    graph.add(boost::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));

    // 6. 设置位姿初值 即前端里程计
    gtsam::Values initial;
    initial.insert(1, gtsam::Pose2(0.0, 0.0, 0.0));
    initial.insert(2, gtsam::Pose2(1.9, 0.0, 0.0));
    initial.insert(3, gtsam::Pose2(4.1, 0.0, 0.0));
    initial.insert(4, gtsam::Pose2(4.1, 2.1, 0.0));
    initial.insert(5, gtsam::Pose2(2.0, 1.9, 0.0));

    // 7. 调用优化器
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print();

    // 8. 后验概率
    std::cout.precision(2);
    gtsam::Marginals marginals(graph, result);
    std::cout << "x1 covariance:\n"
              << marginals.marginalCovariance(1) << std::endl;
    std::cout << "x2 covariance:\n"
              << marginals.marginalCovariance(2) << std::endl;
    std::cout << "x3 covariance:\n"
              << marginals.marginalCovariance(3) << std::endl;
    std::cout << "x4 covariance:\n"
              << marginals.marginalCovariance(4) << std::endl;
    std::cout << "x5 covariance:\n"
              << marginals.marginalCovariance(5) << std::endl;
}
