//#include <boost/foreach.hpp>

#include "ndt_mcl_localization.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool userInitialPose = false;
bool hasNewInitialPose = false;

double ipos_x = 0, ipos_y = 0, ipos_yaw = 0;
double ivar_x = 0, ivar_y = 0, ivar_yaw = 0;

double time_end;
std::string tf_odo_topic;
std::string tf_laser_link;
int NOP;

NDTMCL *ndtmcl;
// Laser sensor offset
float offx = 0;
float offy = 0;
float offa = 0;
static bool has_sensor_offset_set = false;
static bool isFirstLoad = true;
Eigen::Affine3d Told, Todo;

ndt_mcl_localization::ndt_mcl_localization()
{
	ndtmcl = NULL;
}

bool ndt_mcl_localization::initialize(perception_oru::NDTMap* ndmap_, double resolution)
{
	ndmap = ndmap_;

	double x = 0, y = 0, yaw = 0;
	double gx = 0, gy = 0, gyaw = 0;

	Eigen::Affine3d T = getAsAffine(x, y, yaw);
	Eigen::Affine3d Tgt = getAsAffine(gx, gy, gyaw);

	// 在此初始化粒子滤波器
	if (isFirstLoad || hasNewInitialPose)
	{
		fprintf(stderr, "Initializing to (%lf, %lf, %lf)\n", gx, gy, gyaw);

		ndtmcl = new NDTMCL(resolution, *ndmap, -0.5);
//		ndtmcl->forceSIR = true;

		// Initialize the particle filter
		ndtmcl->initializeFilter(gx, gy, gyaw, 0.5, 0.2, 2.0*M_PI/180.0, NOP);
		Told = T;
		Todo = Tgt;

		hasNewInitialPose = false;
	}

	return true;
}

bool ndt_mcl_localization::match(pcl::PointCloud<pcl::PointXYZ>& cloud,
	Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>& T)
{
	// Now we have the sensor origin and pointcloud -- Lets do MCL
	ndtmcl->updateAndPredict(Tmotion, cloud); ///<predicts, updates and resamples if necessary (ndt_mcl.hpp)
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void initialPoseReceived()
{
	ipos_x = 0;
	ipos_y = 0;
	ipos_yaw = 0;
	hasNewInitialPose = true;
}

Eigen::Affine3d getAsAffine(float x, float y, float yaw)
{
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Translation3d v(x, y, 0);
	Eigen::Affine3d T = v * m;

	return T;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Callback for laser scan messages
 */
void callback(/*const sensor_msgs::LaserScan::ConstPtr& scan*/)
{
	static int counter = 0;
	counter++;

//	static tf::TransformListener tf_listener;

	if (has_sensor_offset_set == false)
		return;

	double gx, gy, gyaw, x, y, yaw;

#if 0
	///Get state information
	tf::StampedTransform transform;

	tf::TransformBroadcaster w2o_broadcaster;
	geometry_msgs::Quaternion w2o_quat = tf::createQuaternionMsgFromYaw(0);
	geometry_msgs::TransformStamped w2o_trans;
	w2o_trans.header.stamp = scan->header.stamp;
	w2o_trans.header.frame_id = "world";
	w2o_trans.child_frame_id = "odom";

	w2o_trans.transform.translation.x = 0;
	w2o_trans.transform.translation.y = 0;
	w2o_trans.transform.translation.z = 0;
	w2o_trans.transform.rotation = w2o_quat;

	w2o_broadcaster.sendTransform(w2o_trans);

	tf_listener.waitForTransform("world", tf_odo_topic, scan->header.stamp, ros::Duration(0.95));
	tf_listener.lookupTransform("world", tf_odo_topic, scan->header.stamp, transform);
#endif

	yaw = 0;  //tf::getYaw(transform.getRotation());
	x = transform.getOrigin().x();
	y = transform.getOrigin().y();

	// Number of scans
	int N = (scan->angle_max - scan->angle_min) / scan->angle_increment;

	// Pose conversions

	Eigen::Affine3d T = getAsAffine(x, y, yaw);
	Eigen::Affine3d Tgt = getAsAffine(gx, gy, gyaw);

	// 如果获取首帧，在此设置初始姿态
	if (userInitialPose && hasNewInitialPose)
	{
		gx = ipos_x;
		gy = ipos_y;
		gyaw = ipos_yaw;
	}

	// 在此初始化粒子滤波器
	if (isFirstLoad || hasNewInitialPose)
	{
		fprintf(stderr, "Initializing to (%lf, %lf, %lf)\n", gx, gy, gyaw);

		// Initialize the particle filter
		ndtmcl->initializeFilter(gx, gy, gyaw, 0.5, 0.2, 2.0*M_PI / 180.0, NOP); //2.0*M_PI/180.0
		Told = T;
		Todo = Tgt;

		hasNewInitialPose = false;
	}

	// 计算姿态变化量Tmotion   Calculate the differential motion from the last frame
	Eigen::Affine3d Tmotion = Told.inverse() * T;

	// 累计里程计，用于对比显示
	Todo = Todo * Tmotion;

	if (!isFirstLoad)
	{
		// 从第二帧开始，对于每一较小的姿态变化，发布里程和扫描数据
		if ((Tmotion.translation().norm() < 0.005 && fabs(Tmotion.rotation().eulerAngles(0, 1, 2)[2]) < (0.2*M_PI / 180.0)))
		{
			Eigen::Vector3d dm = ndtmcl->getMean();
			Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances();

			// 在此发出里程及扫描数据消息
//			sendROSOdoMessage(dm, cov, scan);
			return;
		}
	}

	Told = T;

	// Calculate the laser pose with respect to the base
	float dy = offy;
	float dx = offx;
	float alpha = atan2(dy, dx);
	float L = sqrt(dx*dx + dy * dy);

	// Laser pose in base frame
	float lpx = L * cos(alpha);
	float lpy = L * sin(alpha);
	float lpa = offa;

	// Laser scan to PointCloud expressed in the base frame

	// 将激光扫描数据转化为以机器人为坐标系的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int j = 0;j < N; j++)
	{
		double r = scan->ranges[j];
		if (r >= scan->range_min && r<scan->range_max && r > 0.8)
		{
			double a = scan->angle_min + j * scan->angle_increment;

			pcl::PointXYZ pt;
			pt.x = r * cos(a + lpa) + lpx;
			pt.y = r * sin(a + lpa) + lpy;
			pt.z = 0.1 + 0.02 * (double)rand() / (double)RAND_MAX;

			cloud->push_back(pt);
		}
	}

	// Now we have the sensor origin and pointcloud -- Lets do MCL
	ndtmcl->updateAndPredict(Tmotion, *cloud); ///<predicts, updates and resamples if necessary (ndt_mcl.hpp)

//	std::cerr << "\n ndtmcl cost: " << mcl_end - mcl_begin << " s" << std::endl;

	Eigen::Vector3d dm = ndtmcl->getMean(); ///Maximum aposteriori pose
	Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances(); ///Pose covariance

	isFirstLoad = false;

//	sendROSOdoMessage(dm, cov, scan); ///Spit out the pose estimate
}

int main(int argc, char **argv) 
{
	bool use_sensor_pose = false;
	double sensor_pose_x = 0, sensor_pose_y = 0, sensor_pose_th = 0;

	double resolution = 0.2;
	bool forceSIR = false;
	NOP = 100;

	userInitialPose = false;

	ipos_x = 0;    // "initial_pose_x"
	ipos_y = 0;    // "initial_pose_y"
	ipos_yaw = 0;  // "initial_pose_yaw"

	if (userInitialPose == true) 
		hasNewInitialPose = true;

	perception_oru::NDTMap ndmap(new perception_oru::LazyGrid(resolution));

	ndmap.setMapSize(80.0, 80.0, 1.0);

	ndtmcl = new NDTMCL(resolution, ndmap, -0.5);
	if (forceSIR) 
		ndtmcl->forceSIR = true;

	fprintf(stderr, "*** FORCE SIR = %d****", forceSIR);

	offa = sensor_pose_th;
	offx = sensor_pose_x;
	offy = sensor_pose_y;


	has_sensor_offset_set = true;

	fprintf(stderr, "Sensor Pose = (%lf %lf %lf)\n", offx, offy, offa);

	return 0;
}
