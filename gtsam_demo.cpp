#include <bits/stdc++.h>
#include "./lib/gtsam_lib.hpp"

int main()
{

    GtsamOpter opt;
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    T1.pretranslate(Eigen::Vector3d(0, 0, 0));
   //T1.translation(Eigen::Vector3d(0, 0, 0));
   // opt.AddSe3Node(T1);
    return 0;
}
