#include <stdafx.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <egsl/egsl_macros.h>
#include <vector>
#include "csm_all.h"
#include "Scan.h"

///////////////////////////////////////////////////////////////////////////////

CCsmScanPoint::CCsmScanPoint()
{
	valid = false;
	readings = std::numeric_limits<double>::quiet_NaN();
	readings_sigma = std::numeric_limits<double>::quiet_NaN();
	theta = std::numeric_limits<double>::quiet_NaN();

	cluster = -1;
	alpha = std::numeric_limits<double>::quiet_NaN();
	cov_alpha = std::numeric_limits<double>::quiet_NaN();
	alpha_valid = 0;

	true_alpha = std::numeric_limits<double>::quiet_NaN();

	up_bigger = 0;
	up_smaller = 0;
	down_bigger = 0;
	down_smaller = 0;

	corr.valid = false;
	corr.j1 = -1;
	corr.j2 = -1;

	points.p[0] = points.p[1] = points.rho = points.phi = std::numeric_limits<double>::quiet_NaN();
	points_w = points;
}

void CCsmScanPoint::ComputeCartisian()
{
	double x = cos(theta) * readings;
	double y = sin(theta) * readings;

	points.p[0] = x,
	points.p[1] = y;
	points.rho = std::numeric_limits<double>::quiet_NaN();
	points.phi = std::numeric_limits<double>::quiet_NaN();
}

double CCsmScanPoint::DistanceTo(const CCsmScanPoint& another)
{
	double dx = another.points.p[0] - points.p[0];
	double dy = another.points.p[1] - points.p[1];
	return sqrt(dx*dx + dy*dy);
}

///////////////////////////////////////////////////////////////////////////////

CCsmScan::CCsmScan(int _nrays)
{
	Alloc(_nrays);
}

CCsmScan::CCsmScan()
{
	m_nRays = 0;
	min_theta = 0;
	max_theta = 0;
	m_pPoints = NULL;
}

CCsmScan::CCsmScan(const CScan& Scan)
{
	m_nRays = 0;
	min_theta = 0;
	max_theta = 0;
	m_pPoints = NULL;

	double min_reading = 0;
	double max_reading = 30;

	int nCount = Scan.m_nCount / CSM_SAMPLE_NUM;
	Alloc(nCount);

	estimate[0] = Scan.m_poseScanner.x;
	estimate[1] = Scan.m_poseScanner.y;
	estimate[2] = Scan.m_poseScanner.fThita;

	odometry[0] = estimate[0];
	odometry[1] = estimate[1];
	odometry[2] = estimate[2];

	tv.tv_sec = 0;
	tv.tv_usec = 0;

	min_theta = Scan.m_fStartAng;
	max_theta = Scan.m_fEndAng;

	for (int i = 0; i < nCount; i++)
	{
		int j = i * CSM_SAMPLE_NUM;
		float reading = Scan.m_pPoints[j].r / 1000.0f;
		m_pPoints[i].valid = (reading > min_reading) && (reading < max_reading);
		m_pPoints[i].readings = m_pPoints[i].valid ? reading : NAN;
		m_pPoints[i].theta = min_theta + i * (max_theta - min_theta) / (m_nRays - 1);
	}

	if (!ValidFields())
		assert(false);
}

CCsmScan::~CCsmScan()
{
	Clear();
}

//
//   清除原有数据。
//
void CCsmScan::Clear()
{
	if (m_pPoints != NULL)
	{
		delete[]m_pPoints;
		m_pPoints = NULL;
	}
}

bool CCsmScan::Alloc(int _nrays)
{
	Clear();

	m_pPoints = new CCsmScanPoint[_nrays];
	if (m_pPoints == NULL)
		return false;

	m_nRays = _nrays;
	min_theta = std::numeric_limits<double>::quiet_NaN();
	max_theta = std::numeric_limits<double>::quiet_NaN();

	for (int i = 0; i < 3; i++)
		odometry[i] = estimate[i] = true_pose[i] = std::numeric_limits<double>::quiet_NaN();

	return true;
}

void CCsmScan::ComputeCartesian()
{
	for (int i = 0; i < m_nRays; i++)
		m_pPoints[i].ComputeCartisian();
}


void CCsmScan::ComputeWorldCoords(const double *pose)
{
	double pose_x = pose[0];
	double pose_y = pose[1];
	double pose_theta = pose[2];
	double cos_theta = cos(pose_theta); 
	double sin_theta = sin(pose_theta);

	for (int i = 0; i < m_nRays; i++)
	{
		CCsmScanPoint& p = m_pPoints[i];

		if (!ValidRay(i))
			continue;

		double x0 = p.points.p[0],
				 y0 = p.points.p[1];
		
		if (is_nan(x0) || is_nan(y0))
			assert(false);
		
		p.points_w.p[0] = cos_theta * x0 - sin_theta * y0 + pose_x;
		p.points_w.p[1] = sin_theta * x0 + cos_theta * y0 + pose_y;
		/* polar coordinates */
	}
	
	for (int i = 0; i < m_nRays; i++)
	{
		CCsmScanPoint& p = m_pPoints[i];

		double x = p.points_w.p[0];
		double y = p.points_w.p[1];
		p.points_w.rho = sqrt( x*x+y*y);
		p.points_w.phi = atan2(y, x);
	}
}

int CCsmScan::NumValidCorr()
{
	int num = 0;
	for (int i = 0; i < m_nRays; i++)
	{
		if (m_pPoints[i].corr.valid)
			num++;
	}

	return num;
}

//
//   对扫描结构数据字段进行正确性验证。
//
//   Do an extensive sanity check about the data contained in the structure.
//
bool CCsmScan::ValidFields()
{
	int min_nrays = 10;
	int max_nrays = 10000;

	if (m_nRays < min_nrays || m_nRays > max_nrays)
		return false;

	if (is_nan(min_theta) || is_nan(max_theta))
		return false;

	double min_fov = deg2rad(20.0);
	double max_fov = 2.01 * M_PI;
	double fov = max_theta - min_theta;

	if (fov < min_fov || fov > max_fov)
		return false;

	if (fabs(min_theta - m_pPoints[0].theta) > 1e-8)
		return false;

	if (fabs(max_theta - m_pPoints[m_nRays - 1].theta) > 1e-8)
		return false;

	/* Check that there are valid rays */
	double min_reading = 0;
	double max_reading = 100;

	int i; for (i = 0;i < m_nRays;i++)
	{
		CCsmScanPoint& p = m_pPoints[i];
		double th = p.theta;
		if (p.valid)
		{
			double r = p.readings;
			if (is_nan(r) || is_nan(th))
				return false;

			if (!(min_reading < r && r < max_reading))
				return false;
		}
		else
		{
			/* ray not valid, but checking theta anyway */
			if (is_nan(th))
				return false;

			if (p.cluster != -1)
				return false;
		}

		if (p.cluster < -1)
			return false;

		if (!is_nan(p.readings_sigma) && p.readings_sigma < 0)
			return false;
	}

	int num_valid = CountValid();
	int num_invalid = CountInvalid();

	/* Checks that there is at least 10% valid rays */
	if (num_valid < m_nRays * 0.10)
		return false;

	return true;
}

int CCsmScan::CountValid()
{
	int num = 0;
	if (m_pPoints == NULL)
		return 0;

	for (int i = 0; i < m_nRays; i++)
	{
		if (m_pPoints[i].valid == true)
			num++;
	}

	return num;
}

int CCsmScan::CountInvalid()
{
	int num = 0;
	if (m_pPoints == NULL)
		return 0;

	for (int i = 0; i < m_nRays; i++)
	{
		if (m_pPoints[i].valid == false)
			num++;
	}

	return num;
}

/** Computes an hash of the correspondences */
unsigned int CCsmScan::CorrHash()
{
	unsigned int hash = 0;
	unsigned int i = 0;

	for (i = 0; i < (unsigned)m_nRays; i++)
	{
		CCsmScanPoint& p = m_pPoints[i];

		int str = ValidCorr((int)i) ? (p.corr.j1 + 1000 * p.corr.j2) : -1;
		hash ^= ((i & 1) == 0) ? ((hash << 7) ^ (str) ^ (hash >> 3)) :
			(~((hash << 11) ^ (str) ^ (hash >> 5)));
	}

	return (hash & 0x7FFFFFFF);
}

/* indexes: an array of size "max_num*2" */
void CCsmScan::FindNeighbours(int i, int max_num, std::vector<int>& indexes, size_t* num_found) 
{
	*num_found = 0;

	int up = i;
	while ((up + 1 <= i + max_num) && (up + 1 < m_nRays) && ValidRay(up + 1)
		&& (m_pPoints[up + 1].cluster == m_pPoints[i].cluster))
	{
		up += 1;
		indexes[(*num_found)++] = up;
	}

	int down = i;
	while ((down >= i - max_num) && (down - 1 >= 0) && ValidRay(down - 1) &&
		(m_pPoints[down - 1].cluster == m_pPoints[i].cluster))
	{
		down -= 1;
		indexes[(*num_found)++] = down;
	}
}

/** expects cartesian valid */
void CCsmScan::VisibilityTest(const gsl_vector* u)
{
	double* theta_from_u = new double[m_nRays];

	int j;
	for (j = 0; j < m_nRays; j++)
	{
		if (!ValidRay(j))
			continue;

		theta_from_u[j] =
			atan2(gvg(u, 1) - m_pPoints[j].points.p[1],
				gvg(u, 0) - m_pPoints[j].points.p[0]);
	}

	int invalid = 0;
	for (j = 1; j < m_nRays; j++)
	{
		if (!ValidRay(j) || !ValidRay(j - 1))
			continue;

		if (theta_from_u[j] < theta_from_u[j - 1])
		{
			m_pPoints[j].valid = false;
			invalid++;
		}
	}

	delete[] theta_from_u;
}

//
//   扫描的角分辩率(线/弧度)。

double CCsmScan::AngularReso()
{
	return m_nRays / (max_theta - min_theta);
}

//
//   判断扫描数据是否超界(对超界的数据扫描线标记"valid = false")。
//
void CCsmScan::InvalidIfOutside(double min_reading, double max_reading)
{
	for (int i = 0; i < m_nRays; i++)
	{
		if (!ValidRay(i))
			continue;

		double r = m_pPoints[i].readings;
		if (r <= min_reading || r > max_reading)
			m_pPoints[i].valid = false;
	}
}

/** A very cool algorithm for finding the orientation */

void filter_orientation(double theta0, double rho0, size_t n,
	const std::vector<double>& thetas, const std::vector<double>& rhos, double *alpha, double*cov0_alpha) {

	egsl_push();
	/* Y = L x + R epsilon */
	val Y = zeros(n, 1);
	val L = ones(n, 1);
	val R = zeros(n, n + 1);

	size_t i; for (i = 0;i < n;i++) {
		*egsl_atmp(Y, i, 0) = (rhos[i] - rho0) / (thetas[i] - theta0);
		*egsl_atmp(R, i, 0) = -1 / (thetas[i] - theta0);
		*egsl_atmp(R, i, i + 1) = +1 / (thetas[i] - theta0);
	}

	val eRinv = inv(m(R, tr(R)));
	val vcov_f1 = inv(m3(tr(L), eRinv, L));
	val vf1 = m4(vcov_f1, tr(L), eRinv, Y);

	double cov_f1 = *egsl_atmp(vcov_f1, 0, 0);
	double f1 = *egsl_atmp(vf1, 0, 0);

	*alpha = theta0 - atan(f1 / rho0);

	if (cos(*alpha)*cos(theta0) + sin(*alpha)*sin(theta0) > 0)
		*alpha = *alpha + M_PI;

	double dalpha_df1 = rho0 / (square(rho0) + square(f1));
	double dalpha_drho = -f1 / (square(rho0) + square(f1));

	*cov0_alpha = square(dalpha_df1) * cov_f1 + square(dalpha_drho);

#ifndef WINDOWS
	if (std::isnan(*alpha)) {
#else
	if (_isnan(*alpha)) {
#endif
		egsl_print("Y", Y);
		egsl_print("L", L);
		egsl_print("R", R);
		egsl_print("eRinv", eRinv);
		egsl_print("vcov_f1", vcov_f1);

		printf("   f1 = %f cov =%f \n", f1, cov_f1);
		printf("   f1/rho = %f \n", f1 / rho0);
		printf("   atan = %f \n", atan(f1 / rho0));
		printf("   theta0= %f \n", theta0);
	}

	egsl_pop();
	/*
	//	printf("dalpha_df1 = %f dalpha_drho = %f\n",dalpha_df1,dalpha_drho);
	//	printf("f1 = %f covf1 = %f alpha = %f cov_alpha = %f\n ",f1,cov_f1,*alpha,*cov0_alpha);
	//	printf("sotto = %f\n ",(square(rho0) + square(f1)));

	//	printf("   alpha = %f sigma= %f°\n", *alpha, rad2deg(0.01*sqrt(*cov0_alpha)));

		printf("l= ");
		gsl_matrix_fprintf(stdout, l, "%f");
		printf("\ny= ");
		gsl_matrix_fprintf(stdout, y, "%f");
		printf("\nr= ");
		gsl_matrix_fprintf(stdout, r, "%f");
		printf("\ninv(r*r)= ");
		gsl_matrix_fprintf(stdout, Rinv, "%f");
		printf("\nf1 = %lf ",f1);
		printf("\ncov_f1 = %lf ",cov_f1);
	*/
	}



/** Requires the "cluster" field to be set */
void CCsmScan::ComputeOrientation(int size_neighbourhood, double sigma)
{
	int i;
	for(i=0;i<m_nRays;i++)
	{
		CCsmScanPoint& p = m_pPoints[i];

		if(!ValidRay(i) || (p.cluster == -1))
		{
			p.alpha = std::numeric_limits<double>::quiet_NaN(); //GSL_NAN;
			p.cov_alpha = std::numeric_limits<double>::quiet_NaN(); //GSL_NAN;
			p.alpha_valid = 0;
			continue;
		}
		
		//int neighbours[size_neighbourhood*2];
		std::vector<int> neighbours(size_neighbourhood*2, 0);
		size_t num_neighbours;
		FindNeighbours(i, size_neighbourhood, neighbours, &num_neighbours);

		if(0==num_neighbours) 
		{
			p.alpha = std::numeric_limits<double>::quiet_NaN(); //GSL_NAN;
			p.cov_alpha = std::numeric_limits<double>::quiet_NaN(); //GSL_NAN;
			p.alpha_valid = 0;
			continue;
		}

/*		printf("orientation for i=%d:\n",i); */
		std::vector<double> thetas(num_neighbours, 0.0);
		std::vector<double> readings(num_neighbours, 0.0);
		size_t a=0; 
		for (a = 0; a < num_neighbours; a++)
		{
			thetas[a] = m_pPoints[neighbours[a]].theta;
			readings[a] = m_pPoints[neighbours[a]].readings;
			/* printf(" j = %d theta = %f rho = %f\n", neighbours[a], thetas[a],readings[a]); */
		}
		
		double _alpha=42, cov0_alpha=32;
		filter_orientation(p.theta, p.readings,num_neighbours,
			thetas,readings,&_alpha,&cov0_alpha);
#ifndef WINDOWS
		if(std::isnan(_alpha))
		{
#else
		if(_isnan(_alpha))
		{
#endif
			p.alpha = std::numeric_limits<double>::quiet_NaN();
			p.cov_alpha = std::numeric_limits<double>::quiet_NaN();
			p.alpha_valid = 0;
		}
		else
		{
			p.alpha = _alpha;
			p.cov_alpha = cov0_alpha * square(sigma);
			p.alpha_valid = 1;
		}
		/* printf("---------- i = %d alpha = %f sigma=%f cov_alpha = %f\n", i, alpha, cov_alpha[i]);*/
	}
}

void CCsmScan::CreateJumpTables()
{
	int i;
	for (i = 0; i < m_nRays; i++)
	{
		CCsmScanPoint& p = m_pPoints[i];
		int j = i + 1;

		while (j < m_nRays && m_pPoints[j].valid && m_pPoints[j].readings <= p.readings)
			j++;
		p.up_bigger = j - i;

		j = i + 1;
		while (j < m_nRays && m_pPoints[j].valid && m_pPoints[j].readings >= p.readings)
			j++;
		p.up_smaller = j - i;

		j = i - 1;
		while (j >= 0 && m_pPoints[j].valid && m_pPoints[j].readings >= p.readings)
			j--;
		p.down_smaller = j - i;

		j = i - 1;
		while (j >= 0 && m_pPoints[j].valid && m_pPoints[j].readings <= p.readings)
			j--;
		p.down_bigger = j - i;
	}
}

void CCsmScan::PossibleInterval(const double *p_i_w, double max_angular_correction_deg,
	double max_linear_correction, int* from, int* to,
	int* start_cell)
{
	double angle_res = (max_theta - min_theta) / m_nRays;

	/* Delta for the angle */
	double delta = fabs(deg2rad(max_angular_correction_deg)) +
		fabs(atan(max_linear_correction / norm_d(p_i_w)));

	/* Dimension of the cell range */
	int range = (int)ceil(delta / angle_res);

	/* To be turned into an interval of cells */
	double start_theta = atan2(p_i_w[1], p_i_w[0]);

	/* Make sure that start_theta is in the interval [min_theta,max_theta].
		For example, -1 is not in [0, 2pi] */
	if (start_theta < min_theta)
		start_theta += 2 * M_PI;

	if (start_theta > max_theta)
		start_theta -= 2 * M_PI;

	*start_cell = (int)((start_theta - min_theta) / (max_theta - min_theta) * m_nRays);

	*from = minmax(0, m_nRays - 1, *start_cell - range);
	*to = minmax(0, m_nRays - 1, *start_cell + range);

	if (0)
		printf("from: %d to: %d delta: %f start_theta: %f min/max theta: [%f,%f] range: %d start_cell: %d\n",
			*from, *to,
			delta, start_theta, min_theta, max_theta, range, *start_cell);
}

/*
	A very very simple clustering algorithm.
	Try threshold = 5*sigma
*/

void CCsmScan::SimpleClustering(double threshold)
{
	int _cluster = -1;
	double last_reading = 0; /* I have to initialize it
	  explicitely or else gcc complains it might be uninitialized.
	  Stupid compiler, it cannot even solve the halting problem. */

	int i;
	for (i = 0; i < m_nRays; i++)
	{
		CCsmScanPoint& p = m_pPoints[i];

		/* Skip if not valid */
		if (!ValidRay(i))
		{
			p.cluster = -1;
			continue;
		}
		/* If this is the first valid point, assign cluster #0 */
		if (-1 == _cluster)
			_cluster = 0;
		else
			/* Else, start a new cluster if distance is above threshold */
			if (fabs(last_reading - p.readings) > threshold)
				_cluster++;

		p.cluster = _cluster;
		last_reading = p.readings;
	}

	/* TODO: set to -1 the one-point clusters */
}

bool CCsmScan::LoadBinary(FILE*fp, int nCount)
{
	double fov = 2 * M_PI;
	double min_reading = 0;
	double max_reading = 30;


	if (!Alloc(nCount))
		return false;

	float f[3];
	if (fread(f, sizeof(float), 3, fp) != 3)
		return false;

	estimate[0] = f[0];
	estimate[1] = f[1];
	estimate[2] = f[2];

	odometry[0] = f[0];
	odometry[1] = f[1];
	odometry[2] = f[2];

	tv.tv_sec = 0;
	tv.tv_usec = 0;

	min_theta = -fov / 2;
	max_theta = +fov / 2;

	// 依次读取各个扫描线数据
	for (int i = 0; i < nCount; i++)
	{
		float reading;
		int nIntensity;

		if (fread(&reading, sizeof(float), 1, fp) != 1)
			return NULL;

		if (fread(&nIntensity, sizeof(int), 1, fp) != 1)
			return NULL;

		reading /= 1000.0f;

		m_pPoints[i].valid = (reading > min_reading) && (reading < max_reading);
		m_pPoints[i].readings = m_pPoints[i].valid ? reading : NAN;
		m_pPoints[i].theta = min_theta + i * (max_theta - min_theta) / (m_nRays - 1);
	}

	return true;
}
