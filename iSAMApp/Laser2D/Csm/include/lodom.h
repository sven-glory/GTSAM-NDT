#ifndef LODOM_H
#define LODOM_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max

class LODOM
{
  public:

    LODOM(double ldist, double adist);
    ~LODOM();
    Eigen::Vector3d lodomPG(const std::vector<unsigned int>& dis);
  private:

    bool initialized_;

    Eigen::Isometry3d Tnow;
    Eigen::Isometry3d Tlast;
    Eigen::Isometry3d Tkey;
    Eigen::Isometry3d Tm;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    double kf_dist_linear_;
    double kf_dist_angular_;

    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    void initParams();

    void processScan(LDP& curr_ldp_scan);
    void ScanToLDP(const std::vector<unsigned int> dis, LDP& ldp);

    // void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    // void imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg);
    // void velCallback (const geometry_msgs::Twist::ConstPtr& twist_msg);
    // void velStmpCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg);

    bool newKeyframeNeeded(Eigen::Isometry3d &T);

    // void getPrediction(double& pr_ch_x, double& pr_ch_y,
    //                    double& pr_ch_a, double dt);

    void createEigenFromXYTheta(double x, double y, double theta, Eigen::Isometry3d &T);
};

#endif
