#ifndef H_SCAN_MATCHING_LIB
#define H_SCAN_MATCHING_LIB

#include <gsl_eigen/gsl_eigen.h>

#include "CsmScan.h"
#include "math_utils.h"

struct sm_result 
{
	bool   valid;            // 1 if the result is valid
	double x[3];             // Scan matching result (x,y,theta)
	int    iterations;       // Number of iterations done
	int    nvalid;           // Number of valid correspondence in the end 
	float  valid_percent;    // 合格的匹配占合格扫描线的百分比
	double error;            // 点到线的平均误差距离
	double error2;           // 点到点的平均误差距离

	/** Fields used for covariance computation */
	gsl_matrix *cov_x_m;
	gsl_matrix *dx_dy1_m;
	gsl_matrix *dx_dy2_m;
};

class CCsmMatcher 
{
private:
	unsigned long m_ulStartTime;

private:
	/** First scan ("ref"erence scan) */
	CCsmScan* m_pRefScan;

	/** Second scan ("sens"or scan) */
	CCsmScan* m_pSensScan;

	/** Where to start */
 	double first_guess[3]; 

	/** Maximum angular displacement between scans (deg)*/
	double max_angular_correction_deg;
	/** Maximum translation between scans (m) */
	double max_linear_correction;

	/** When to stop */
	int max_iterations;
	/** A threshold for stopping. */
	double epsilon_xy;
	/** A threshold for stopping. */
	double epsilon_theta;
	
	/** Maximum distance for a correspondence to be valid */
	double max_correspondence_dist;

	/** Use smart tricks for finding correspondences. Only influences speed; not convergence. */
	bool use_corr_tricks;
	
	/** Restart if error under threshold (0 or 1)*/
	bool restart;

		/** Threshold for restarting */
		double restart_threshold_mean_error;
		/** Displacement for restarting */
		double restart_dt;
		/** Displacement for restarting */
		double restart_dtheta;
	

	/* Functions concerning discarding correspondences.
	   THESE ARE MAGIC NUMBERS -- and they need to be tuned. */

	/** Percentage of correspondences to consider: if 0.9,
	    always discard the top 10% of correspondences with more error */
	double outliers_maxPerc;

	/** Parameters describing a simple adaptive algorithm for discarding.
	    1) Order the errors.
		2) Choose the percentile according to outliers_adaptive_order.
		   (if it is 0.7, get the 70% percentile)
		3) Define an adaptive threshold multiplying outliers_adaptive_mult
		   with the value of the error at the chosen percentile.
		4) Discard correspondences over the threshold.
		
		This is useful to be conservative; yet remove the biggest errors.
	*/
		double outliers_adaptive_order; /* 0.7 */
		double outliers_adaptive_mult; /* 2 */

	/** Do not allow two different correspondences to share a point */
	bool outliers_remove_doubles; 


	
	/* Functions that compute and use point orientation for defining matches. */
		/** For now, a very simple max-distance clustering algorithm is used */
		double clustering_threshold;
		/** Number of neighbour rays used to estimate the orientation.*/
		int orientation_neighbourhood;
		/** Discard correspondences based on the angles */
		bool do_alpha_test;
		double do_alpha_test_thresholdDeg;
		
		
	/** I believe this trick is documented in one of the papers by Guttman (but I can't find
	    the reference). Or perhaps I was told by him directly. 
		
		If you already have a guess of the solution, you can compute the polar angle
		of the points of one scan in the new position. If the polar angle is not a monotone
		function of the readings index, it means that the surface is not visible in the 
		next position. If it is not visible, then we don't use it for matching.
		
		This is confusing without a picture! To understand what's going on, make a drawing
		in which a surface is not visible in one of the poses.
	
		Implemented in the function VisibilityTest().
	*/	
	bool do_visibility_test;

	/** If 1, use PlICP; if 0, use vanilla ICP. */
	bool use_point_to_line_distance;

	/** If 1, the field "true_alpha" is used to compute the incidence
	    beta, and the factor (1/cos^2(beta)) used to weight the impact
	    of each correspondence. This works fabolously if doing localization,
	    that is the first scan has no noise.
		If "true_alpha" is not available, it uses "alpha".
	*/
	int use_ml_weights;
	
	/* If 1, the field "readings_sigma" is used to weight the correspondence by 1/sigma^2 */
	int use_sigma_weights;
	
	/** Use the method in http://purl.org/censi/2006/icpcov to compute
	    the matching covariance. */
	bool do_compute_covariance;

	/** Checks that find_correspondences_tricks give the right answer */
	bool debug_verify_tricks;
	
	/** Pose of sensor with respect to robot: used for computing
	    the first estimate given the odometry. */
	double laser[3]; 

	/** Noise in the scan */
	double sigma;

	/** mark as invalid ( = don't use ) rays outside of this interval */
	double min_reading;
	double max_reading;
	
	unsigned long time_out;        // 算法超时退出时间(0-永不超时)

private:
	bool compatible(int i, int j);
	void find_correspondences();
	void find_correspondences_tricks();
	void debug_correspondences();

	/** This sets the stage.  */
	bool sm_icp(sm_result*res);

	/** This is the meat */
	bool icp_loop(const double* q0, double* x_new, double* total_error, double* error2, int* nvalid, int* iterations);
	int termination_criterion(const double*delta);
	void kill_outliers_trim(double*total_error, double* error2);
	void kill_outliers_double();

	/** This is the beef: computing in closed form the next estimate
	given the correspondences. */
	int compute_next_estimate(const double x_old[3], double x_new[3]);

public:
	CCsmMatcher()
	{
		max_angular_correction_deg = 90.0;   // Maximum angular displacement between scans
		max_linear_correction = 20.0;         // Maximum translation between scans (m)
		max_iterations = 50;                 // When we had enough
		epsilon_xy = 0.0001;                 // A threshold for stopping (m)
		epsilon_theta = 0.0001;              // A threshold for stopping (rad) = 0.005 deg
		max_correspondence_dist = 0.3;// 2.0;       // dubious parameter (m)
		sigma = 0.01;                        // Noise in the scan (m)
		use_corr_tricks = true;              // Use smart tricks for finding correspondences.
		restart = true;                      // Restart: Restart if error is over threshold
		restart_threshold_mean_error = 0.01; // Restart: Threshold for restarting. )
		restart_dt = 0.01;                   // Restart: displacement for restarting. (m)
		restart_dtheta = deg2rad(1.5);       // Restart: displacement for restarting. (rad)
		clustering_threshold = 0.05;         // Max distance for staying in the same clustering
		orientation_neighbourhood = 3;       // Number of neighbour rays used to estimate the orientation.
		use_point_to_line_distance = true;   // If 0, it's vanilla ICP.
		do_alpha_test = false;               // Discard correspondences based on the angles
		do_alpha_test_thresholdDeg = 20.0;   //
		outliers_maxPerc = 0.95;             // Outlier所占的比例不应超过此值
		outliers_adaptive_order = 0.7;
		outliers_adaptive_mult = 2.0;
		do_visibility_test = false;
		outliers_remove_doubles = true;      // no two points in m_pSensScan can have the same corr.);
		do_compute_covariance = false;       // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov .
		debug_verify_tricks = false;         // Checks that find_correspondences_tricks gives the right answer.
		laser[0] = 0.0;                      // laser.x (m)
		laser[1] = 0.0;                      // laser.y (m)
		laser[2] = 0.0;                      // laser.theta (rad)
		min_reading = 0.5;                   // Don't use readings less than min_reading (m)
		max_reading = 1000.0;                // Don't use readings longer than max_reading (m)
		use_ml_weights = 0;                  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.
		use_sigma_weights = 0;               // If 1, the field 'readings_sigma' in the second scan is used to weight the correspondence by 1/sigma^2
		time_out = 0;                        // 永不超时
	}

	bool Match(CCsmScan* pRefScan, CCsmScan* pSensScan, sm_result& result);
};

void csm_free_unused_memory();

#endif
