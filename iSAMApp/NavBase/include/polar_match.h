/*
First of all you need to select the laser range finder you have by
setting "#define PM_LASER "
to PM_SICK_LMS200, PM_HOKUYO_URG_04LX. 
*/

#ifndef _POLAR_MATCH_
#define _POLAR_MATCH_

#include "Geometry.h"

#define M_PI              3.1415926f
#define SQ(x)             ((x)*(x))


//----------------- 激 光 扫 描 器 相 关 参 数------------

// STEP 1) Define a name for your laser range finder here (if it hasn't been defined yet):
// 第1步: 定义激光头的名称
#define PM_PSD_SCANNER      0
#define PM_HOKUYO_URG_04LX  1
#define PM_SICK_LMS200      2
#define PM_HOKUYO_UTM_30LX  3

// STEP 2) Set the type your laser range finder here:
// 第2步：选定程序中使用的激光头的 种类
//  #define PM_LASER PM_SICK_LMS200
//  #define PM_LASER PM_HOKUYO_URG_04LX
//  #define PM_LASER PM_PSD_SCANNER
   #define PM_LASER PM_HOKUYO_UTM_30LX

// STEP 3) Add your laser range finder's parameters here if it is a different model (use centimeters)
// 第三步：定义激光头的各种参数(单位:mm)

#if PM_LASER ==  PM_PSD_SCANNER
  #define PM_LASER_NAME       "PSD_Scanner" //< The name of the laser range finder. 
  #define PM_L_POINTS         200 //< Maximum number of points in a scan.
  #define PM_FOV              360 //< Field of view of the laser range finder in degrees.
  #define PM_MAX_RANGE        4000 //< [mm] Maximum valid laser range .
  #define PM_MIN_VALID_POINTS 100 //< Minimum number of valid points for scan matching.
  #define PM_SEARCH_WINDOW    50  //< Half window size which is searched for correct orientation.
  #define PM_CORRIDOR_THRESHOLD 25.0 //< Threshold for angle variation between points to determine if scan was taken of a corridor.
#elif PM_LASER ==  PM_HOKUYO_URG_04LX
  #define PM_LASER_NAME       "Hokuyo URG-04LX" //< The name of the laser range finder. 
  #define PM_L_POINTS         682 //< Maximum number of points in a scan.
  #define PM_FOV              240 //< Field of view of the laser range finder in degrees.
  #define PM_MAX_RANGE        5300 //< [mm] Maximum valid laser range.
  #define PM_MIN_VALID_POINTS 200 //< Minimum number of valid points for scan matching.
  #define PM_SEARCH_WINDOW    80  //< Half window size which is searched for correct orientation.
  #define PM_CORRIDOR_THRESHOLD 25.0 //< Threshold for angle variation between points to determine if scan was taken of a corridor.
#elif PM_LASER ==  PM_SICK_LMS200
  #define PM_LASER_NAME       "Sick LMS" //< The name of the laser range finder. 
  #define PM_L_POINTS         181  //< Maximum number of points in a scan.
  #define PM_FOV              180  //< Field of view of the laser range finder in degrees.
  #define PM_MAX_RANGE        10000 //< [mm] Maximum valid laser range.
  #define PM_MIN_VALID_POINTS 40   //< Minimum number of valid points for scan matching.
  #define PM_SEARCH_WINDOW    20   //< Half window size which is searched for correct orientation.
  #define PM_CORRIDOR_THRESHOLD 25.0 //< Threshold for angle variation between points to determine if scan was taken of a corridor.
#elif PM_LASER ==  PM_HOKUYO_UTM_30LX
  #define PM_LASER_NAME       "HOKUYO UTM-30LX" //< The name of the laser range finder. 
  #define PM_L_POINTS         1081  //< Maximum number of points in a scan.
  #define PM_FOV              270  //< Field of view of the laser range finder in degrees.
  #define PM_MAX_RANGE        7000  //< [mm] Maximum valid laser range. (3000 for this sensor.)
  #define PM_MIN_VALID_POINTS 300   //< Minimum number of valid points for scan matching.
  #define PM_SEARCH_WINDOW    200   //< Half window size which is searched for correct orientation.
  #define PM_CORRIDOR_THRESHOLD 25.0 //< Threshold for angle variation between points to determine if scan was taken of a corridor.
#endif

// STEP 4) Set the time registration delay (the time between your time stamps and the 
// time the scan was taken) and the distance of your laser from the odometry center 
// in the forward direction.
// 第4步：设置配准延时及激光头安装位置
#define PM_TIME_DELAY       0 //<[s] Time delay (time registration error) in the laser measurements. Use 0.02 for SLAMbot.
#define PM_LASER_Y          0 //<@brief [mm] Y coordinate of the laser on the robot.激光头在车体Y方向上的安装配位置(mm)
                              //<
                              //<Set to 0 for ground thruth, simulation, mapping_with_matching.
                              //<Set to 0 when in doubt and handle the coordinate transforms yourself. 
                              //<Set it to 31.3 for Slambot.
                              //<The Y axis points in the forward motion direction of a differential drive robot. 
#define PM_MIN_RANGE        100.0f //< [mm] Minimum valid laser range for the reprojected current scan.
#define PM_SEG_MAX_DIST     200.0 //< The distance between points to break a segment. 
#define PM_WEIGHTING_FACTOR 70*70 //< Parameter used for weighting associated points in the position calculation of PSM. Try setting it to 300*300 for laser odometry.
#define PM_CHANGE_WEIGHT_ITER 10 //< The number of iterations after which the weighting factor is reduced to weight down outliers.

#define PM_MAX_ERROR        1000  //< [mm] Maximum distance between associated points used in pose estimation. Try setting it to 300 for laser odometry.
#define PM_STOP_COND        0.4  //< If the pose change (|dx|+|dy|+|dth|) is smaller than this PSM scan matching stops.
#define PM_MAX_ITER         30   //< Maximum number of iterations for PSM.
#define PM_MAX_ITER_ICP     60   //< Maximum number of iterations for ICP
#define PM_STOP_COND_ICP    0.1  //< Stopping condition for ICP. The pose change has to be smaller than this.

#define PM_MIN_STD_XY          200.0 //<[mm] The minimum match result standard deviation in X or Y direction. Used in covariance estimation.
#define PM_MIN_STD_ORIENTATION 4.0  //<[degrees] The minimum standard deviation of the orientation match. Used in covariance estimation.
#define PM_MATCH_ERROR_OFFSET  50.0  //<[mm] Offset subtracted from average range residual when scaling the covariance matrix.

#define PM_ODO  -1 //< Show results with odometry only in mapping_with_matching().
#define PM_PSM   1 //< Polar scan matching - matching bearing association rule.
#define PM_ICP   3 //< Scan matching with iterative closest point association rule.

#define PM_TIME_FILE "results/iterations.txt" //< When generating results, timing is data is saved into this file from the matching algorithms.  i|time|ax|ay|ath[deg] is saved. Does not work in the moment.

// Description of range reading errors. Each range measurement may be tagged with one of these:
#define PM_RANGE     1  //< Measurement tag: range reading is longer than PM_MAX_RANGE.
#define PM_MOVING    2  //< Measurement tag: range reading corresponds to a moving point. Not implemented yet.
#define PM_MIXED     4  //< Measurement tag: range reading is a mixed pixel.
#define PM_OCCLUDED  8  //< Measurement tag: range reading is occluded.
#define PM_EMPTY     16 //< Measurement tag: no measurment (between 2 segments there is no interpolation!)

extern float   pm_fi[PM_L_POINTS];//< Contains precomputed range bearings.
extern float   pm_si[PM_L_POINTS];//< Contains the sinus of each bearing.
extern float   pm_co[PM_L_POINTS];//< Contains the cosinus of each bearing.
extern const float   PM_D2R; //< Conversion factor for converting degrees to radians.
extern const float   PM_R2D; //< Conversion factor for converting radians to degrees.

/** @brief Structure describing a laser scan.

The robot pose (rx,ry,th) describes the odometry center if PM_LASER_Y 
does not equal 0, or the laser center if PM_LASER_Y=0. <br>

In a laser scan, the middle laser bearing coincides with the laser coordinate frame's Y axis.

TODO: Consider using doubles for rx,ry in large environments.
*/
class CPmScan
{
public:
	CPosture m_pstRobot;          //<[mm] Robot posture
	float  r[PM_L_POINTS];        //<[mm] Laser range readings. 0 or negative ranges denote invalid readings.


	int      bad[PM_L_POINTS];    //< @brief Tag describing the validity of a range measurement.
										  //< 0 if OK; sources of invalidity - out of range reading;
										  //< reading belongs to moving object (not implemented); occlusion; mixed pixel.
	int      seg[PM_L_POINTS];    //< Describes which segment the range reading belongs to.

public:
	void pm_preprocessScan();
	void pm_median_filter();
	void pm_find_far_points();
	void pm_segment_scan();

	void pm_take_simulated_scan(const CPosture& pst);

	float pm_translation_estimation(const float *new_r, const int *new_bad, float C, float *dx, float *dy);
	float pm_orientation_search(const float *new_r, const int *new_bad);

	void Plot(CDC* pDC, float x, float y, float th, COLORREF crColor);
};

float norm_a(float a);

BOOL pm_init();

float pm_psm( const CPmScan *lsr,CPmScan *lsa);
float pm_icp( const CPmScan *lsr,CPmScan *lsa);

void pm_plotScanAt(const CPmScan *ls, float x,float y,float th,const char *col, double diameter = 2.0, bool connect_lines = false);
void pm_plotScan(CPmScan *ls, const char *col,double diameter = 2.0, bool connect_lines = false);

void pm_scan_project(const CPmScan *act, float *new_r, int *new_bad);

#endif
