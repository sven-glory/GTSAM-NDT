#include <lodom.h>

int i = 0;
LODOM::LODOM(double ldist, double adist):
  kf_dist_linear_(ldist),
  kf_dist_angular_(adist),
  initialized_(false)
{

  initParams();

  Tnow.setIdentity();
  Tlast.setIdentity();
  Tkey.setIdentity();
  Tm.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;
  input_.min_reading = 2.0;
  input_.max_reading = 30.0;

  // Initialize output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;
  initParams();
  
}

LODOM::~LODOM()
{}

void LODOM::initParams()
{

    input_.max_angular_correction_deg = 45.0;
    input_.max_linear_correction = 0.50;
    input_.max_iterations = 10;
    input_.epsilon_xy = 0.000001;
    input_.epsilon_theta = 0.000001;
    input_.max_correspondence_dist = 0.3;
    input_.sigma = 0.010;
    input_.use_corr_tricks = 1;
    input_.restart = 0;
    input_.restart_threshold_mean_error = 0.01;
    input_.restart_dt = 1.0;
    input_.restart_dtheta = 0.1;
    input_.clustering_threshold = 0.25;
    input_.orientation_neighbourhood = 20;
    input_.use_point_to_line_distance = 1;
    input_.do_alpha_test = 0;
    input_.do_alpha_test_thresholdDeg = 20.0;
    input_.outliers_maxPerc = 0.90;
    input_.outliers_adaptive_order = 0.7;
    input_.outliers_adaptive_mult = 2.0;
    input_.do_visibility_test = 0;
    input_.outliers_remove_doubles = 1;
    input_.do_compute_covariance = 0;
    input_.debug_verify_tricks = 0;
    input_.use_ml_weights = 0;
    input_.use_sigma_weights = 0;
}

void LODOM::ScanToLDP(const std::vector<unsigned int> dis, LDP& ldp)
{
  unsigned int n = dis.size();
  ldp = ld_alloc_new(n);
  double ai = 2 * M_PI /n;

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = (double)dis[i]/1000;

    if (r > input_.min_reading && r < input_.max_reading)
    {
      // fill in laser scan data
      ldp->valid[i] = 1;
      ldp->readings[i] = r; 
      
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = -M_PI + i * ai;

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

// void LODOM::velCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
// {
//   boost::mutex::scoped_lock(mutex_);
//   latest_vel_msg_ = *twist_msg;

//   received_vel_ = true;
// }

void LODOM::processScan(LDP& curr_ldp_scan)
{
  prev_ldp_scan_->odometry[0] = 0.0;
  prev_ldp_scan_->odometry[1] = 0.0;
  prev_ldp_scan_->odometry[2] = 0.0;

  prev_ldp_scan_->estimate[0] = 0.0;
  prev_ldp_scan_->estimate[1] = 0.0;
  prev_ldp_scan_->estimate[2] = 0.0;

  prev_ldp_scan_->true_pose[0] = 0.0;
  prev_ldp_scan_->true_pose[1] = 0.0;
  prev_ldp_scan_->true_pose[2] = 0.0;

  input_.laser_ref  = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // **** estimated change since last scan

//   double dt = (time - last_icp_time_).toSec();
//   double pr_ch_x, pr_ch_y, pr_ch_a;
//   getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

    Tnow = Tlast;
    Tm = Tkey.inverse() * Tnow;

//   input_.first_guess[0] = pr_ch_l.getOrigin().getX();
//   input_.first_guess[1] = pr_ch_l.getOrigin().getY();
//   input_.first_guess[2] = tf::getYaw(pr_ch_l.getRotation());
  input_.first_guess[0] = Tm.translation()(0);
  input_.first_guess[1] = Tm.translation()(1);
  input_.first_guess[2] = Tm.rotation().eulerAngles(0,1,2)(2);
//  std::cout << "Temp: " << Temp.translation()(0) << " , " << Temp.translation()(1) << " , " <<  Temp.rotation().eulerAngles(0,1,2)(2) << std::endl;
 
  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  sm_icp(&input_, &output_);
  if (output_.valid)
  {

    createEigenFromXYTheta(output_.x[0], output_.x[1], output_.x[2], Tm);

    // std::cout << "Tkm: " << Temp.translation()(0) << " , " << Temp.translation()(1) << " , " <<  Temp.rotation().eulerAngles(0,1,2)(2) << std::endl;

    Tnow = Tkey * Tm;

    // std::cout << "Tnow: " << Tnow.translation()(0) << " , " << Tnow.translation()(1) << " , " <<  Tnow.rotation().eulerAngles(0,1,2)(2) << std::endl;

    if (newKeyframeNeeded(Tm))
    {
        ld_free(prev_ldp_scan_);
        prev_ldp_scan_ = curr_ldp_scan;
        Tkey = Tnow;
    }
    else
    {
        ld_free(curr_ldp_scan);
    }
    // std::cout << "Tlast: " << Tlast.translation()(0) << " , " << Tlast.translation()(1) << " , " <<  Tlast.rotation().eulerAngles(0,1,2)(2) << std::endl;
    // std::cout << "Tm: " << Tm.translation()(0) << " , " << Tm.translation()(1) << " , " <<  Tm.rotation().eulerAngles(0,1,2)(2) << std::endl;
    Tlast = Tnow;
  }
    
  else
  {
    std::cout << " falied " << std::endl;
  }
 
}

Eigen::Vector3d LODOM::lodomPG(const std::vector<unsigned int>& dis)
{
  // **** if first scan, cache the tf from base to the scanner

  if (!initialized_)
  {
    ScanToLDP(dis, prev_ldp_scan_);
    initialized_ = true;
    return Eigen::Vector3d(0, 0, 0);
  }

  LDP curr_ldp_scan;
  ScanToLDP(dis, curr_ldp_scan);
  processScan(curr_ldp_scan);
  Eigen::Vector3d result(Tnow.translation()(0), Tnow.translation()(1), Tnow.rotation().eulerAngles(0,1,2)(2));
  return result;
}

bool LODOM::newKeyframeNeeded(Eigen::Isometry3d &T)
{
  double dyaw = T.rotation().eulerAngles(0,1,2)(2);
  if (fabs(dyaw) > kf_dist_angular_) 
  {
    std::cout << "new KeyFrame set: " << ++i << std::endl;
    return true;
  }

  if (T.translation().norm() > kf_dist_linear_) 
  {
    std::cout << "new KeyFrame set: " << ++i << std::endl;
    return true;
  }

  return false;
}
// returns the predicted change in pose (in fixed frame)
// since the last time we did icp
// void LODOM::getPrediction(double& pr_ch_x, double& pr_ch_y,
//                                      double& pr_ch_a, double dt)
// {
//   boost::mutex::scoped_lock(mutex_);

//   // **** base case - no input available, use zero-motion model
//   pr_ch_x = 0.0;
//   pr_ch_y = 0.0;
//   pr_ch_a = 0.0;

//   // **** use velocity (for example from ab-filter)
//   if (use_vel_)
//   {
//     pr_ch_x = dt * latest_vel_msg_.linear.x;
//     pr_ch_y = dt * latest_vel_msg_.linear.y;
//     pr_ch_a = dt * latest_vel_msg_.angular.z;

//     if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
//     else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;
//   }

//   // **** use wheel odometry
//   if (use_odom_ && received_odom_)
//   {
//     pr_ch_x = latest_odom_msg_.pose.pose.position.x -
//               last_used_odom_msg_.pose.pose.position.x;

//     pr_ch_y = latest_odom_msg_.pose.pose.position.y -
//               last_used_odom_msg_.pose.pose.position.y;

//     pr_ch_a = tf::getYaw(latest_odom_msg_.pose.pose.orientation) -
//               tf::getYaw(last_used_odom_msg_.pose.pose.orientation);

//     // tf::Transform transform1;
//     // transform1.setOrigin( tf::Vector3(last_used_odom_msg_.pose.pose.position.x, last_used_odom_msg_.pose.pose.position.y, 0.0) );
//     // tf::Quaternion q1;
//     // q1.setRPY(0, 0, tf::getYaw(last_used_odom_msg_.pose.pose.orientation));
//     // transform1.setRotation(q1);

//     // tf::Transform transform2;
//     // transform2.setOrigin( tf::Vector3(latest_odom_msg_.pose.pose.position.x, latest_odom_msg_.pose.pose.position.y, 0.0) );
//     // tf::Quaternion q2;
//     // q2.setRPY(0, 0, tf::getYaw(latest_odom_msg_.pose.pose.orientation));
//     // transform2.setRotation(q2);

//     // tf::Transform delta = transform2 * transform1.inverse();

//     // pr_ch_x = delta.getOrigin().getX();
//     // pr_ch_y = delta.getOrigin().getY();
//     // pr_ch_a = tf::getYaw(delta.getRotation());

//     if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
//     else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

//     std::cerr << "predit dx : " << pr_ch_x 
//               << " , dy :" << pr_ch_y
//               << " , dyaw :" << pr_ch_a << std::endl;

//     last_used_odom_msg_ = latest_odom_msg_;
//   }

//   // **** use imu
//   if (use_imu_ && received_imu_)
//   {
//     pr_ch_a = tf::getYaw(latest_imu_msg_.orientation) -
//               tf::getYaw(last_used_imu_msg_.orientation);

//     if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
//     else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

//     last_used_imu_msg_ = latest_imu_msg_;
//   }
// }

void LODOM::createEigenFromXYTheta(
  double x, double y, double theta, Eigen::Isometry3d& t)
{
  t.setIdentity();
  Eigen::AngleAxisd rotation_vector ( theta, Eigen::Vector3d ( 0,0,1 ) );
  t.rotate (rotation_vector);
  t.pretranslate ( Eigen::Vector3d (x, y, 0)); 
}
