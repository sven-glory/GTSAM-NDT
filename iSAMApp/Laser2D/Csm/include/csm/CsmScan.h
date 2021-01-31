#ifndef H_LASER_DATA
#define H_LASER_DATA

//#ifndef WINDOWS
//#include <sys/time.h>
//#else
#include <time.h>
#include <windows.h>
//#endif

#include <vector>
#include <stdio.h>
#include <limits>
#include <gsl_eigen/gsl_eigen.h>
#include "restrict.h"
#include "CorrList.h"

#define CSM_SAMPLE_NUM               1

using namespace std;

class correspondence
{
public:
	bool valid;        // 1 if this correspondence is valid
	int j1;           // Closest point in the other scan
	int j2;           // Second closest point in the other scan
	enum { corr_pp = 0, corr_pl = 1} type;  // Type of correspondence (point to point, or point to line)
	double dist2_j1;  // Squared distance from p(i) to point j1

public:
	correspondence() 
	{
		Clear();
	}

	void Clear()
	{
		valid = false;
		j1 = -1;
		j2 = -1;
		dist2_j1 = std::numeric_limits<double>::quiet_NaN(); //GSL_NAN;
	}

	void Set(int _j1, int _j2)
	{
		valid = true;
		j1 = _j1;
		j2 = _j2;
	}

};

class point2d
{
public:
	double p[2];
	double rho, phi;

public:
	point2d() 
	{
		rho = phi = 0;
		p[0] = p[1] = 0;
	}

	double DistanceTo(const point2d& another)
	{
		double dx = another.p[0] - p[0];
		double dy = another.p[1] - p[1];
		return sqrt(dx*dx + dy*dy);
	}
};

class CCsmScanPoint
{
public:
   double theta;
   bool   valid;
   double readings;
   int    cluster;
   double alpha;
   double cov_alpha;
   int    alpha_valid;
   double readings_sigma;
   double true_alpha;
   correspondence  corr;
   point2d points;        // Cartesian representation
   point2d points_w;      // Cartesian representation, in "world" (laser_ref) coordinates Computed using ComputeWorldCoords()
   int up_bigger;
   int up_smaller;
   int down_bigger;
   int down_smaller;

public:
   CCsmScanPoint();

	void ComputeCartisian();
	double DistanceTo(const CCsmScanPoint& another);
};

class CScan;

///////////////////////////////////////////////////////////////////////////////
//   定义“CCsmScan”类。
class CCsmScan
{
private:
	double true_pose[3];
	double estimate[3];
	struct timeval tv;               // Timestamp

public:
	int    m_nRays;
	double min_theta;
	double max_theta;
	double odometry[3];

	CCsmScanPoint* m_pPoints;

private:

	/** -1 if not found */
	int NextValid(int i, int dir)
	{
		int j;
		for (j = i + dir; (j < m_nRays) && (j >= 0) && !ValidRay(j); j += dir);
		return ValidRay(j) ? j : -1;
	}

	void FindNeighbours(int i, int max_num, std::vector<int>& indexes, size_t*num_found);


public:
   CCsmScan(int _nrays);
	CCsmScan(const CScan& Scan);
	CCsmScan();

   ~CCsmScan();

	// 清除原有数据
	void Clear();

	/** This allocs the fields in the given structure. Use ld_alloc_new(), not this. */
	bool Alloc(int _nrays);

	/** Computes the "points_w" coordinates by roto-translating "points" */
	void ComputeWorldCoords(const double *pose);

	/** Fills the x,y fields in "points" by transforming (theta, reading) to cartesian */
	void ComputeCartesian();

	/** Fills the fields: *up_bigger, *up_smaller, *down_bigger, *down_smaller.*/
	void CreateJumpTables();

	/** Computes an hash of the correspondences */
	unsigned int CorrHash();

	/** Returns the number of valid correspondences. */
	int NumValidCorr();

	/** Do an extensive sanity check about the data contained in the structure. */
	bool ValidFields();

	/** A simple clustering algorithm. Sets the `cluster' field in the structure. */
	void SimpleClustering(double threshold);

	/** A cool orientation estimation algorithm. Needs cluster. */
	void ComputeOrientation(int size_neighbourhood, double sigma);

   void PossibleInterval(const double *p_i_w, double max_angular_correction_deg,
                          double max_linear_correction, int*from, int*to, int*start_cell);

	int ValidRay(int i)
	{
      return (i >= 0) && (i < m_nRays) && (m_pPoints[i].valid);
	}

	bool ValidAlpha(int i)
	{
		return (m_pPoints[i].alpha_valid != 0);
	}

	int CountValid();

	int CountInvalid();

	void SetNullCorr(int i)
	{
		m_pPoints[i].corr.Clear();
	}

	void SetCorr(int i, int j1, int j2) 
	{
		m_pPoints[i].corr.Set(j1, j2);
	}

	int NextValidUp(int i)
	{
		return NextValid(i, +1);
	}

	int NextValidDown(int i)
	{
		return NextValid(i, -1);
	}

	bool ValidCorr(int i) 
	{
		return m_pPoints[i].corr.valid;
	}

	// 扫描的角分辩率(线/弧度)
	double AngularReso();

	/** Marks a ray invalid if reading is outside range [min_reading, max_reading]. */
	void InvalidIfOutside(double min_reading, double max_reading);

	void VisibilityTest(const gsl_vector*x_old);

	bool LoadBinary(FILE*fp, int nCount);
};
#endif

