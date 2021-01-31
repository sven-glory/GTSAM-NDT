#include <math.h>
#include <string.h>

#include <gsl_eigen/gsl_eigen.h>
#include <egsl/egsl_macros.h>
#include <gpc/gpc.h>
#include "../csm/csm_all.h"
#include "icp.h"
#include "time_patch.h"
#include <vector>

///////////////////////////////////////////////////////////////////////////////
//                                ICP                                        //
///////////////////////////////////////////////////////////////////////////////
bool CCsmMatcher::sm_icp(sm_result* res)
{
	res->valid = false;

	// 对两个扫描的数据字段进行基本的正确性判断
	if (!m_pRefScan->ValidFields() || !m_pSensScan->ValidFields())
		return false;

	// 对两个扫描集合中的点进行合格性判断，对扫描数据超界的情况进行“不合格”标记
	m_pRefScan->InvalidIfOutside(min_reading, max_reading);
	m_pSensScan->InvalidIfOutside(min_reading, max_reading);

	egsl_push_named("sm_icp");

	// 如必要，生成跳转表
	if (use_corr_tricks || debug_verify_tricks)
		m_pRefScan->CreateJumpTables();

	// 对所有点生成笛卡尔坐标
	m_pRefScan->ComputeCartesian();
	m_pSensScan->ComputeCartesian();

	// 如果需要的话，进行角度测试
	if (do_alpha_test)
	{
		m_pRefScan->SimpleClustering(clustering_threshold);
		m_pRefScan->ComputeOrientation(orientation_neighbourhood, sigma);
		m_pSensScan->SimpleClustering(clustering_threshold);
		m_pSensScan->ComputeOrientation(orientation_neighbourhood, sigma);
	}

	gsl_vector * x_new = gsl_vector_alloc(3);
	gsl_vector * x_old = vector_from_array(3, first_guess);

	// 如果需要的话，进行可视性测试
	if (do_visibility_test)
	{
		m_pRefScan->VisibilityTest(x_old);

		gsl_vector * minus_x_old = gsl_vector_alloc(3);
		ominus(x_old, minus_x_old);
		m_pSensScan->VisibilityTest(minus_x_old);
		gsl_vector_free(minus_x_old);
	}

	double error, error2;
	int iterations, nvalid;

	// 进行ICP循环迭代
	if (!icp_loop(x_old->data(), x_new->data(), &error, &error2, &nvalid, &iterations))
	{
		res->valid = false;
		res->iterations = iterations;
		res->nvalid = 0;
	}

	// ICP成功
	else
	{
		int restarted = 0;
		double best_error = error;
		gsl_vector * best_x = gsl_vector_alloc(3);
		gsl_vector_memcpy(best_x, x_new);

		if (restart && (error / nvalid)>(restart_threshold_mean_error))
		{
			restarted = 1;
			double dt = restart_dt;
			double dth = restart_dtheta;

			double perturb[6][3] =
			{
				{ dt,0,0 },{ -dt,0,0 },
				{ 0,dt,0 },{ 0,-dt,0 },
				{ 0,0,dth },{ 0,0,-dth }
			};

			for (int a = 0; a < 6; a++)
			{
				CCsmMatcher my_params = *this;
				gsl_vector * start = gsl_vector_alloc(3);
				gvs(start, 0, gvg(x_new, 0) + perturb[a][0]);
				gvs(start, 1, gvg(x_new, 1) + perturb[a][1]);
				gvs(start, 2, gvg(x_new, 2) + perturb[a][2]);

				gsl_vector * x_a = gsl_vector_alloc(3);

				double my_error;
				int my_valid, my_iterations;

				if (!my_params.icp_loop(start->data(), x_a->data(), &my_error, &error2, &my_valid, &my_iterations))
					break;

				iterations += my_iterations;

				if (my_error < best_error)
				{
					gsl_vector_memcpy(best_x, x_a);
					best_error = my_error;
				}

				gsl_vector_free(x_a); 
				gsl_vector_free(start);
			}
		}


		// 最终，记录ICP成功的结果
		res->valid = true;
		vector_to_array(best_x, res->x);

		// recompute correspondences in case of restarts
		if (restarted)
		{
			m_pSensScan->ComputeWorldCoords(res->x);
			if (use_corr_tricks)
				find_correspondences_tricks();
			else
				find_correspondences();
		}

		if (do_compute_covariance) 
		{
			val cov0_x, dx_dy1, dx_dy2;
			compute_covariance_exact(m_pRefScan, m_pSensScan, best_x, &cov0_x, &dx_dy1, &dx_dy2);

			val cov_x = sc(square(sigma), cov0_x);

			res->cov_x_m = egsl_v2gslm(cov_x);
			res->dx_dy1_m = egsl_v2gslm(dx_dy1);
			res->dx_dy2_m = egsl_v2gslm(dx_dy2);
		}

		res->error = best_error / nvalid;
		res->error2 = error2;
		res->iterations = iterations;
		res->nvalid = nvalid;
		res->valid_percent = (float)nvalid / (float)m_pSensScan->CountValid();
		gsl_vector_free(best_x);
	}

	gsl_vector_free(x_new);
	gsl_vector_free(x_old);

	egsl_pop_named("sm_icp");

	return res->valid;
}

void csm_free_unused_memory()
{
	egsl_free_unused_memory();
}

///////////////////////////////////////////////////////////////////////////////
//                                Corr Tricks                                //
///////////////////////////////////////////////////////////////////////////////

/** This is very close (but *less* than) to sin(x), for
	 x in (0, PI/2). It's a 5 degree taylor expansion. */
INLINE double mysin(double x)
{
	const double a = -1.0 / 6.0;
	const double b = +1.0 / 120.0;
	double x2 = x * x;
	return x * (.99 + x2 * (a + b * x2));
}


#define DEBUG_SEARCH(a) ;

extern int distance_counter;

INLINE double local_distance_squared_d(const double* a, const double* b)
{
	distance_counter++;
	double x = a[0] - b[0];
	double y = a[1] - b[1];
	return x * x + y * y;
}

#define SQUARE(a) ((a)*(a))

/* Assumes that points_w is filled.  */
void CCsmMatcher::find_correspondences_tricks()
{
	/* Handy constant */
	double C1 = m_pRefScan->AngularReso();
	double max_correspondence_dist2 = square(max_correspondence_dist);

	/* Last match */
	int last_best = -1;
	//	const point2d * restrict points_w = m_pSensScan->points_w;

	for (int i = 0; i < m_pSensScan->m_nRays; i++)
	{
		// 对于不合格的扫描线，直接置为“无对应点”
		if (!m_pSensScan->ValidRay(i))
		{
			m_pSensScan->SetNullCorr(i);
			continue;
		}

		// 获取该点在m_pRefScan中的位置
		CCsmScanPoint& pt = m_pSensScan->m_pPoints[i];
		const double *p_i_w = pt.points_w.p;       // 迪卡尔坐标
		double p_i_w_nrm2 = pt.points_w.rho;       // 极径
		double p_i_w_phi = pt.points_w.phi;        // 极角

		/** Search domain for j1 */
		int from = 0;
		int to = m_pRefScan->m_nRays - 1;

		int start_cell = (int)((p_i_w_phi - m_pRefScan->min_theta) * C1);

		/** Current best match */
		int j1 = -1;

		/** and his distance */
		double best_dist = 42;

		/** If last match was succesful, then start at that index + 1 */
		int we_start_at;
		if (last_best == -1)
			we_start_at = start_cell;
		else
			we_start_at = last_best + 1;

		if (we_start_at > to)
			we_start_at = to;

		if (we_start_at < from) 
			we_start_at = from;

		int up = we_start_at + 1;
		int down = we_start_at;
		double last_dist_up = 0; /* first is down */
		double last_dist_down = -1;

		int up_stopped = 0;
		int down_stopped = 0;

		DEBUG_SEARCH(printf("i=%d p_i_w = %f %f\n", i, p_i_w[0], p_i_w, [1]));
		DEBUG_SEARCH(printf("i=%d [from %d down %d mid %d up %d to %d]\n",
			i, from, down, start_cell, up, to));

		while ((!up_stopped) || (!down_stopped))
		{
			int now_up = up_stopped ? 0 :
				down_stopped ? 1 : last_dist_up < last_dist_down;

			DEBUG_SEARCH(printf("|"));

			/* Now two symmetric chunks of code, the now_up and the !now_up */
			if (now_up) 
			{
				DEBUG_SEARCH(printf("up %d ", up));

				/* If we have crossed the "to" boundary we stop searching
					on the "up" direction. */
				if (up > to)
				{
					up_stopped = 1; continue;
				}

				/* Just ignore invalid rays */
				if (!m_pRefScan->m_pPoints[up].valid)
				{
					++up;
					continue;
				}

				/* This is the distance from p_i_w to the "up" point*/
				last_dist_up = local_distance_squared_d(p_i_w, m_pRefScan->m_pPoints[up].points.p);

				/* If it is less than the best point, it is our new j1 */
				if ((last_dist_up < best_dist) || (j1 == -1))
				{
					j1 = up;
					best_dist = last_dist_up;
				}

				/* If we are moving away from start_cell */
				if (up > start_cell)
				{
					double delta_theta = (m_pRefScan->m_pPoints[up].theta - p_i_w_phi);

					/* We can compute a bound for early stopping. Currently
						our best point has distance best_dist; we can compute
						min_dist_up, which is the minimum distance that can have
						points for j>up (see figure)*/
					double min_dist_up = p_i_w_nrm2 *
						((delta_theta > M_PI*0.5) ? 1 : mysin(delta_theta));

					/* If going up we can't make better than best_dist, then
						 we stop searching in the "up" direction */
					if (SQUARE(min_dist_up) > best_dist)
					{
						up_stopped = 1; continue;
					}

					/* If we are moving away, then we can implement the jump tables
						optimizations. */
					up +=
						/* If p_i_w is shorter than "up" */
						(m_pRefScan->m_pPoints[up].readings < p_i_w_nrm2)
						?
						/* We can jump to a bigger point */
						m_pRefScan->m_pPoints[up].up_bigger
						/* Or else we jump to a smaller point */
						: m_pRefScan->m_pPoints[up].up_smaller;

				}
				else
					/* If we are moving towards "start_cell", we can't do any
						ot the previous optimizations and we just move to the next point */
					++up;
			}

			/* This is the specular part of the previous chunk of code. */
			if (!now_up)
			{
				DEBUG_SEARCH(printf("down %d ", down));
				if (down < from)
				{
					down_stopped = 1;
					continue;
				}

				if (!m_pRefScan->m_pPoints[down].valid)
				{
					--down;
					continue;
				}

				last_dist_down = local_distance_squared_d(p_i_w, m_pRefScan->m_pPoints[down].points.p);
				if ((last_dist_down < best_dist) || (j1 == -1))
				{
					j1 = down;
					best_dist = last_dist_down;
				}

				if (down < start_cell)
				{
					double delta_theta = (p_i_w_phi - m_pRefScan->m_pPoints[down].theta);
					double min_dist_down = p_i_w_nrm2 *
						((delta_theta > M_PI*0.5) ? 1 : mysin(delta_theta));

					if (SQUARE(min_dist_down) > best_dist)
					{
						down_stopped = 1; continue;
					}
					down += (m_pRefScan->m_pPoints[down].readings < p_i_w_nrm2) ?
						m_pRefScan->m_pPoints[down].down_bigger : m_pRefScan->m_pPoints[down].down_smaller;
				}

				else --down;
			}
		}

		DEBUG_SEARCH(printf("i=%d j1=%d dist=%f\n", i, j1, best_dist));

		/* If no point matched. */
		if ((-1 == j1) || (best_dist > max_correspondence_dist2))
		{
			m_pSensScan->SetNullCorr(i);
			continue;
		}

		/* We ignore matching the first or the last point in the scan */
		if (0 == j1 || j1 == (m_pRefScan->m_nRays - 1)) 
		{
			/* no match */
			m_pSensScan->SetNullCorr(i);
			continue;
		}

		/* Now we want to find j2, the second best match. */
		int j2;

		/* We find the next valid point, up and down */
		int j2up = m_pRefScan->NextValidUp(j1);
		int j2down = m_pRefScan->NextValidDown(j1);

		/* And then (very boring) we use the nearest */
		if ((j2up == -1) && (j2down == -1))
		{
			m_pSensScan->SetNullCorr(i);
			continue;
		}

		if (j2up == -1)
		{
			j2 = j2down;
		}
		else if (j2down == -1)
		{
			j2 = j2up;
		}
		else
		{
			double dist_up = local_distance_squared_d(p_i_w, m_pRefScan->m_pPoints[j2up].points.p);
			double dist_down = local_distance_squared_d(p_i_w, m_pRefScan->m_pPoints[j2down].points.p);
			j2 = dist_up < dist_down ? j2up : j2down;
		}

		last_best = j1;

		pt.corr.valid = true;
		pt.corr.j1 = j1;
		pt.corr.j2 = j2;
		pt.corr.dist2_j1 = best_dist;
		pt.corr.type =
			use_point_to_line_distance ? correspondence::corr_pl : correspondence::corr_pp;
	}
}

///////////////////////////////////////////////////////////////////////////////
//                                Corr Dumb                                  //
///////////////////////////////////////////////////////////////////////////////

bool CCsmMatcher::compatible(int i, int j)
{
	if (!do_alpha_test)
		return true;

	double theta0 = 0; /* FIXME */
	if ((m_pSensScan->m_pPoints[i].alpha_valid == 0) ||
		(m_pRefScan->m_pPoints[j].alpha_valid == 0))
		return true;

	double alpha_i = m_pSensScan->m_pPoints[i].alpha;
	double alpha_j = m_pRefScan->m_pPoints[j].alpha;
	double tolerance = deg2rad(do_alpha_test_thresholdDeg);

	/** FIXME remove alpha test */
	double theta = angleDiff(alpha_j, alpha_i);
	if (fabs(angleDiff(theta, theta0)) > tolerance + deg2rad(max_angular_correction_deg))
		return false;
	else
		return true;
}

void CCsmMatcher::find_correspondences()
{
	for (int i = 0; i < m_pSensScan->m_nRays; i++)
	{
		if (!m_pSensScan->ValidRay(i))
		{
			/*			sm_debug("dumb: i %d is invalid \n", i);*/
			m_pSensScan->SetNullCorr(i);
			continue;
		}

		double *p_i_w = m_pSensScan->m_pPoints[i].points_w.p;

		int j1 = -1;
		double best_dist = 10000;

		int from; int to; int start_cell;
		m_pRefScan->PossibleInterval(p_i_w, max_angular_correction_deg,
			max_linear_correction, &from, &to, &start_cell);

		for (int j = from; j <= to; j++)
		{
			if (!m_pRefScan->ValidRay(j))
				continue;

			double dist = distance_squared_d(p_i_w, m_pRefScan->m_pPoints[j].points.p);
			if (dist > square(max_correspondence_dist))
				continue;

			if ((-1 == j1) || (dist < best_dist))
			{
				if (compatible(i, j))
				{
					j1 = j;
					best_dist = dist;
				}
			}
		}

		if (j1 == -1)
		{/* no match */
			m_pSensScan->SetNullCorr(i);
			continue;
		}

		/* Do not match with extrema*/
		if (j1 == 0 || (j1 == (m_pRefScan->m_nRays - 1)))
		{
			/* no match */
			m_pSensScan->SetNullCorr(i);
			continue;
		}

		int j2;
		int j2up = m_pRefScan->NextValidUp(j1);
		int j2down = m_pRefScan->NextValidDown(j1);
		if ((j2up == -1) && (j2down == -1))
		{
			m_pSensScan->SetNullCorr(i);
			continue;
		}

		if (j2up == -1)
		{
			j2 = j2down;
		}
		else if (j2down == -1)
		{
			j2 = j2up;
		}
		else
		{
			double dist_up = distance_squared_d(p_i_w, m_pRefScan->m_pPoints[j2up].points.p);
			double dist_down = distance_squared_d(p_i_w, m_pRefScan->m_pPoints[j2down].points.p);
			j2 = dist_up < dist_down ? j2up : j2down;
		}

		m_pSensScan->SetCorr(i, j1, j2);
		m_pSensScan->m_pPoints[i].corr.dist2_j1 = best_dist;

		m_pSensScan->m_pPoints[i].corr.type =
			use_point_to_line_distance ? correspondence::corr_pl : correspondence::corr_pp;
	}
}

///////////////////////////////////////////////////////////////////////////////
//                                Covariance                                 //
///////////////////////////////////////////////////////////////////////////////


val compute_C_k(val p_j1, val p_j2);
val dC_drho(val p1, val p2);


void compute_covariance_exact(CCsmScan* pRefScan, CCsmScan* pSensScan,
	const gsl_vector*x, val *cov0_x, val *dx_dy1, val *dx_dy2)
{
	egsl_push_named("compute_covariance_exact");

	val d2J_dxdy1 = zeros(3, (size_t)pRefScan->m_nRays);
	val d2J_dxdy2 = zeros(3, (size_t)pSensScan->m_nRays);

	/* the three pieces of d2J_dx2 */
	val d2J_dt2 = zeros(2, 2);
	val d2J_dt_dtheta = zeros(2, 1);
	val d2J_dtheta2 = zeros(1, 1);

	double theta = x->data()[2];
	val t = egsl_vFa(2, x->data());

	for (int i = 0; i < pSensScan->m_nRays; i++)
	{
		if (!pSensScan->ValidCorr(i))
			continue;

		egsl_push_named("compute_covariance_exact iteration");

		int j1 = pSensScan->m_pPoints[i].corr.j1;
		int j2 = pSensScan->m_pPoints[i].corr.j2;

		val p_i = egsl_vFa(2, pSensScan->m_pPoints[i].points.p);
		val p_j1 = egsl_vFa(2, pRefScan->m_pPoints[j1].points.p);
		val p_j2 = egsl_vFa(2, pRefScan->m_pPoints[j2].points.p);

		/* v1 := rot(theta+M_PI/2)*p_i */
		val v1 = m(rot(theta + M_PI / 2), p_i);

		/* v2 := (rot(theta)*p_i+t-p_j1) */
		val v2 = sum3(m(rot(theta), p_i), t, minus(p_j1));

		/* v3 := rot(theta)*v_i */
		val v3 = vers(theta + pSensScan->m_pPoints[i].theta);

		/* v4 := rot(theta+PI/2)*v_i; */
		val v4 = vers(theta + pSensScan->m_pPoints[i].theta + M_PI / 2);

		val C_k = compute_C_k(p_j1, p_j2);

		val d2J_dt2_k = sc(2.0, C_k);
		val d2J_dt_dtheta_k = sc(2.0, m(C_k, v1));

		val v_new = m(rot(theta + M_PI), p_i);
		val d2J_dtheta2_k = sc(2.0, sum(m3(tr(v2), C_k, v_new), m3(tr(v1), C_k, v1)));
		add_to(d2J_dt2, d2J_dt2_k);
		add_to(d2J_dt_dtheta, d2J_dt_dtheta_k);
		add_to(d2J_dtheta2, d2J_dtheta2_k);

		/* for measurement rho_i  in the second scan */
		val d2Jk_dtdrho_i = sc(2.0, m(C_k, v3));
		val d2Jk_dtheta_drho_i = sc(2.0, sum(m3(tr(v2), C_k, v4), m3(tr(v3), C_k, v1)));
		add_to_col(d2J_dxdy2, (size_t)i, comp_col(d2Jk_dtdrho_i, d2Jk_dtheta_drho_i));

		/* for measurements rho_j1, rho_j2 in the first scan */

		val dC_drho_j1 = dC_drho(p_j1, p_j2);
		val dC_drho_j2 = dC_drho(p_j2, p_j1);


		val v_j1 = vers(pRefScan->m_pPoints[j1].theta);

		val d2Jk_dt_drho_j1 = sum(sc(-2.0, m(C_k, v_j1)), sc(2.0, m(dC_drho_j1, v2)));
		val d2Jk_dtheta_drho_j1 = sum(sc(-2.0, m3(tr(v_j1), C_k, v1)), m3(tr(v2), dC_drho_j1, v1));
		add_to_col(d2J_dxdy1, (size_t)j1, comp_col(d2Jk_dt_drho_j1, d2Jk_dtheta_drho_j1));

		/* for measurement rho_j2*/
		val d2Jk_dt_drho_j2 = sc(2.0, m(dC_drho_j2, v2));
		val d2Jk_dtheta_drho_j2 = sc(2.0, m3(tr(v2), dC_drho_j2, v1));
		add_to_col(d2J_dxdy1, (size_t)j2, comp_col(d2Jk_dt_drho_j2, d2Jk_dtheta_drho_j2));

		egsl_pop_named("compute_covariance_exact iteration");
	}

	/* composes matrix  d2J_dx2  from the pieces*/
	val d2J_dx2 = comp_col(comp_row(d2J_dt2, d2J_dt_dtheta),
		comp_row(tr(d2J_dt_dtheta), d2J_dtheta2));

	val edx_dy1 = sc(-1.0, m(inv(d2J_dx2), d2J_dxdy1));
	val edx_dy2 = sc(-1.0, m(inv(d2J_dx2), d2J_dxdy2));

	val ecov0_x = sum(m(edx_dy1, tr(edx_dy1)), m(edx_dy2, tr(edx_dy2)));

	/* With the egsl_promote we save the matrix in the previous
		context */
	*cov0_x = egsl_promote(ecov0_x);
	*dx_dy1 = egsl_promote(edx_dy1);
	*dx_dy2 = egsl_promote(edx_dy2);

	egsl_pop_named("compute_covariance_exact");
	/* now edx_dy1 is not valid anymore, but *dx_dy1 is. */
}

val compute_C_k(val p_j1, val p_j2)
{
	val d = sub(p_j1, p_j2);
	double alpha = M_PI / 2 + atan2(atv(d, 1), atv(d, 0));
	double c = cos(alpha); double s = sin(alpha);
	double m[2 * 2] =
	{
		c*c, c*s,
		c*s, s*s
	};
	return egsl_vFda(2, 2, m);
}


val dC_drho(val p1, val p2)
{
	double eps = 0.001;

	val C_k = compute_C_k(p1, p2);
	val p1b = sum(p1, sc(eps / egsl_norm(p1), p1));
	val C_k_eps = compute_C_k(p1b, p2);
	return sc(1 / eps, sub(C_k_eps, C_k));
}


///////////////////////////////////////////////////////////////////////////////
//                                Outliers                                   //
///////////////////////////////////////////////////////////////////////////////

void quicksort(std::vector<double>& array, int begin, int end);

// 如果多于一个在m_pSensScan中的点对应到了m_pRefScan中的同一个点，那么仅留下最近的那个。
void CCsmMatcher::kill_outliers_double()
{
	double threshold = 3; /* TODO: add as configurable */

	std::vector<double> dist2_i(m_pSensScan->m_nRays, 0.0);
	std::vector<double> dist2_j(m_pRefScan->m_nRays, 0.0);

	for (int j = 0;j < m_pRefScan->m_nRays;j++)
		dist2_j[j] = 1000000;

	int i;
	for (i = 0; i < m_pSensScan->m_nRays; i++)
	{
		if (!m_pSensScan->ValidCorr(i))
			continue;

		int j1 = m_pSensScan->m_pPoints[i].corr.j1;
		dist2_i[i] = m_pSensScan->m_pPoints[i].corr.dist2_j1;
		dist2_j[j1] = (std::min)(dist2_j[j1], dist2_i[i]);
	}

	int nkilled = 0;
	for (i = 0; i < m_pSensScan->m_nRays; i++)
	{
		if (!m_pSensScan->ValidCorr(i))
			continue;

		int j1 = m_pSensScan->m_pPoints[i].corr.j1;
		if (dist2_i[i] > (threshold * threshold) * dist2_j[j1])
		{
			m_pSensScan->m_pPoints[i].corr.valid = false;
			nkilled++;
		}
	}
}

/**
	Trims the corrispondences using an adaptive algorithm

	Assumes cartesian coordinates computed. (points and points_w)

	So, to disable this:
		outliers_maxPerc = 1
		outliers_adaptive_order = 1

*/
void CCsmMatcher::kill_outliers_trim(double* total_error, double* error2)
{
	/* dist2, indexed by k, contains the error for the k-th correspondence */
	std::vector<double> dist2(m_pSensScan->m_nRays, 0.0);
	std::vector<double> dist(m_pSensScan->m_nRays, 0.0);

	// 依次考察m_pSensScan中的点
	int k = 0;
	for (int i = 0; i < m_pSensScan->m_nRays; i++)
	{
		// 对于那些没有合格对应点的点，距离dist置为NaN
		if (!m_pSensScan->ValidCorr(i))
		{
			dist[i] = std::numeric_limits<double>::quiet_NaN();
			continue;
		}

		double *p_i_w = m_pSensScan->m_pPoints[i].points_w.p;
		int j1 = m_pSensScan->m_pPoints[i].corr.j1;
		int j2 = m_pSensScan->m_pPoints[i].corr.j2;

		/* Compute the distance to the corresponding segment */
		// 计算该点到对应线段的距离
		dist[i] = dist_to_segment_d(m_pRefScan->m_pPoints[j1].points.p, m_pRefScan->m_pPoints[j2].points.p, p_i_w);
		dist2[k] = dist[i];
		k++;
	}

	/* two errors limits are defined: */
		/* In any case, we don't want more than outliers_maxPerc% */
	// 定义了两种误差限：超过outliers_maxPerc%部分的点都应被丢弃掉

	// 计算应留下多少点
	int order = (int)floor(k * outliers_maxPerc);
	order = (std::max)(0, (std::min)(order, k - 1));

	// 将所有对应的距离误差升序排列
	quicksort(dist2, 0, k - 1);
	double error_limit1 = dist2[order];

	/* Then we take a order statics (o*K) */
	/* And we say that the error must be less than alpha*dist(o*K) */
	int order2 = (int)floor(k * outliers_adaptive_order);
	order2 = (std::max)(0, (std::min)(order2, k - 1));

	double error_limit2 = outliers_adaptive_mult * dist2[order2];
	double error_limit = (std::min)(error_limit1, error_limit2);

	*total_error = 0;
	int nvalid = 0;
	for (int i = 0;i < m_pSensScan->m_nRays; i++)
	{
		if (!m_pSensScan->ValidCorr(i))
			continue;

		if (dist[i] > error_limit)
		{
			m_pSensScan->m_pPoints[i].corr.valid = false;
			m_pSensScan->m_pPoints[i].corr.j1 = -1;
			m_pSensScan->m_pPoints[i].corr.j2 = -1;
		}
		else
		{
			nvalid++;
			*total_error += dist[i];
		}
	}

	// 下面计算“点到点”之间的误差距离(上面计算的是“点到线”之间的误差距离)
	*error2 = 0;
	for (int i = 0; i < m_pSensScan->m_nRays; i++)
	{
		if (!m_pSensScan->ValidCorr(i))
			continue;

		// 计算对应点
		point2d& pt1 = m_pSensScan->m_pPoints[i].points_w;
		int j1 = m_pSensScan->m_pPoints[i].corr.j1;
		
		assert(j1 >= 0 && j1 < m_pRefScan->m_nRays);

		point2d& pt2 = m_pRefScan->m_pPoints[j1].points;
		
		// 累加匹配点间距离
		double d = pt1.DistanceTo(pt2);
		*error2 += d;
	}

	*error2 /= nvalid;
}


void swap_double(double*a, double*b)
{
	double t = *a; 
	*a = *b; 
	*b = t;
}

/** Code taken from Wikipedia */
void quicksort(std::vector<double>& array, int begin, int end)
{
	if (end > begin)
	{
		double pivot = array[begin];
		int l = begin + 1;
		int r = end + 1;

		while (l < r)
		{
			if (array[l] < pivot)
			{
				l++;
			}
			else
			{
				r--;
				swap_double(&(array[l]), &(array[r]));
			}
		}

		l--;
		swap_double(&(array[begin]), &(array[l]));

		if (l > begin)
			quicksort(array, begin, l);
		if (end > r)
			quicksort(array, r, end);
	}
}


///////////////////////////////////////////////////////////////////////////////
//                             ICP Loop                                      //
///////////////////////////////////////////////////////////////////////////////


#define ICP_FAIL_PERCENT              0.05

bool CCsmMatcher::icp_loop(const double* q0, double* x_new, double* total_error, double* error2,
	int* valid, int* iterations)
{
	if (any_nan(q0, 3))
		return false;

	double x_old[3], delta[3], delta_old[3] = { 0,0,0 };
	copy_d(q0, 3, x_old);

	std::vector<unsigned int> hashes(max_iterations, 0);
	int iteration;

	bool all_is_okay = true;

	for (iteration = 0; iteration < max_iterations; iteration++)
	{
#ifdef _MSC_VER
		if (time_out != 0)
		{
			if (GetTickCount() - m_ulStartTime > 1000)
				return false;
		}
#endif

		egsl_push_named("icp_loop iteration");

		// 将m_SensScan里的点变换到m_pRefScan的坐标系内
		m_pSensScan->ComputeWorldCoords(x_old);

		// 找出对应点(简单方法/聪明方法)
		if (use_corr_tricks)
			find_correspondences_tricks();
		else
			find_correspondences();

		// 如果需要的话，核对find_correspondences_tricks()和find_correspondences()计算的结果是否一样
		if (debug_verify_tricks)
			debug_correspondences();

		// 计算m_pSensScan集合中对应 点的数量
		int num_corr = m_pSensScan->NumValidCorr();
		double fail_perc = ICP_FAIL_PERCENT;

		// 如果对应点太少，退出
		if (num_corr < fail_perc * m_pSensScan->m_nRays)
		{ 
			all_is_okay = false;
			egsl_pop_named("icp_loop iteration"); /* loop context */
			break;
		}

		// 去掉那些怀疑是outliers的点
		if (outliers_remove_doubles)
			kill_outliers_double();

		// 重新计算m_pSensScan集合中对应点的数量
		int num_corr2 = m_pSensScan->NumValidCorr();

		double error = 0;

		/* Trim correspondences */
		kill_outliers_trim(&error, error2);
		int num_corr_after = m_pSensScan->NumValidCorr();

		*total_error = error;
		*valid = num_corr_after;

		// 如果对应点太少，退出
		if (num_corr_after < fail_perc * m_pSensScan->m_nRays)
		{
			all_is_okay = false;
			egsl_pop_named("icp_loop iteration"); /* loop context */
			break;
		}

		/* Compute next estimate based on the correspondences */
		if (!compute_next_estimate(x_old, x_new))
		{
			all_is_okay = false;
			egsl_pop_named("icp_loop iteration");
			break;
		}

		pose_diff_d(x_new, x_old, delta);

		/** Checks for oscillations */
		hashes[iteration] = m_pSensScan->CorrHash();

		/** PLICP terminates in a finite number of steps! */
		if (use_point_to_line_distance)
		{
			bool loop_detected = false;
			for (int a = iteration - 1; a >= 0; a--)
			{
				if (hashes[a] == hashes[iteration])
				{
					loop_detected = true;
					break;
				}
			}

			if (loop_detected)
			{
				egsl_pop_named("icp_loop iteration");
				break;
			}
		}

		/* This termination criterium is useless when using
		the point-to-line-distance; however, we put it here because
		one can choose to use the point-to-point distance. */
		if (termination_criterion(delta))
		{
			egsl_pop_named("icp_loop iteration");
			break;
		}

		copy_d(x_new, 3, x_old);
		copy_d(delta, 3, delta_old);


		egsl_pop_named("icp_loop iteration");
	}

	*iterations = iteration + 1;

	return all_is_okay;
}

int CCsmMatcher::termination_criterion(const double*delta)
{
	double a = norm_d(delta);
	double b = fabs(delta[2]);
	return (a < epsilon_xy) && (b < epsilon_theta);
}

/** This is the beef: computing in closed form the next estimate
given the correspondences. */
int CCsmMatcher::compute_next_estimate(const double x_old[3], double x_new[3])
{
	//struct gpc_corr c[m_pSensScan->m_nRays];
	struct gpc_corr dummy;
	std::vector<gpc_corr> c(m_pSensScan->m_nRays, dummy);

	int i; int k = 0;
	for (i = 0; i < m_pSensScan->m_nRays; i++)
	{
		if (!m_pSensScan->m_pPoints[i].valid)
			continue;

		if (!m_pSensScan->ValidCorr(i))
			continue;

		int j1 = m_pSensScan->m_pPoints[i].corr.j1;
		int j2 = m_pSensScan->m_pPoints[i].corr.j2;

		c[k].valid = true;

		if (m_pSensScan->m_pPoints[i].corr.type == correspondence::corr_pl)
		{
			c[k].p[0] = m_pSensScan->m_pPoints[i].points.p[0];
			c[k].p[1] = m_pSensScan->m_pPoints[i].points.p[1];
			c[k].q[0] = m_pRefScan->m_pPoints[j1].points.p[0];
			c[k].q[1] = m_pRefScan->m_pPoints[j1].points.p[1];

			/** TODO: here we could use the estimated alpha */
			double diff[2];
			diff[0] = m_pRefScan->m_pPoints[j1].points.p[0] - m_pRefScan->m_pPoints[j2].points.p[0];
			diff[1] = m_pRefScan->m_pPoints[j1].points.p[1] - m_pRefScan->m_pPoints[j2].points.p[1];
			double one_on_norm = 1 / sqrt(diff[0] * diff[0] + diff[1] * diff[1]);
			double normal[2];
			normal[0] = +diff[1] * one_on_norm;
			normal[1] = -diff[0] * one_on_norm;

			double cos_alpha = normal[0];
			double sin_alpha = normal[1];

			c[k].C[0][0] = cos_alpha * cos_alpha;
			c[k].C[1][0] =
				c[k].C[0][1] = cos_alpha * sin_alpha;
			c[k].C[1][1] = sin_alpha * sin_alpha;
		}
		else
		{
			c[k].p[0] = m_pSensScan->m_pPoints[i].points.p[0];
			c[k].p[1] = m_pSensScan->m_pPoints[i].points.p[1];

			projection_on_segment_d(
				m_pRefScan->m_pPoints[j1].points.p,
				m_pRefScan->m_pPoints[j2].points.p,
				m_pSensScan->m_pPoints[i].points_w.p,
				c[k].q);

			/* Identity matrix */
			c[k].C[0][0] = 1;
			c[k].C[1][0] = 0;
			c[k].C[0][1] = 0;
			c[k].C[1][1] = 1;
		}

		double factor = 1;

		/* Scale the correspondence weight by a factor concerning the
		information in this reading. */
		if (use_ml_weights)
		{
			int have_alpha = 0;
			double alpha = 0;

			if (!is_nan(m_pRefScan->m_pPoints[j1].true_alpha))
			{
				alpha = m_pRefScan->m_pPoints[j1].true_alpha;
				have_alpha = 1;
			}
			else if (m_pRefScan->m_pPoints[j1].alpha_valid)
			{
				alpha = m_pRefScan->m_pPoints[j1].alpha;
				have_alpha = 1;
			}
			else have_alpha = 0;

			if (have_alpha)
			{
				double pose_theta = x_old[2];
				/** Incidence of the ray
				Note that alpha is relative to the first scan (not the world)
				and that pose_theta is the angle of the second scan with
				respect to the first, hence it's ok. */
				double beta = alpha - (pose_theta + m_pSensScan->m_pPoints[i].theta);
				factor = 1 / square(cos(beta));
			}
			else
			{
				static int warned_before = 0;
				if (!warned_before)
				{
					/*ld_write_as_json(m_pRefScan, stderr);*/
					warned_before = 1;
				}
			}
		}

		/* Weight the points by the sigma in m_pSensScan */
		if (use_sigma_weights)
		{
			if (!is_nan(m_pSensScan->m_pPoints[i].readings_sigma))
			{
				factor *= 1 / square(m_pSensScan->m_pPoints[i].readings_sigma);
			}
			else
			{
				static int warned_before = 0;
			}
		}

		c[k].C[0][0] *= factor;
		c[k].C[1][0] *= factor;
		c[k].C[0][1] *= factor;
		c[k].C[1][1] *= factor;

		k++;
	}

	/* TODO: use prior for odometry */
	double std = 0.11;
	const double inv_cov_x0[9] =
	{
		1 / (std*std), 0, 0,
		0, 1 / (std*std), 0,
		0, 0, 0
	};

	int ok = gpc_solve(k, c, 0, inv_cov_x0, x_new);
	if (!ok)
		return 0;

	double old_error = gpc_total_error(c, k, x_old);
	double new_error = gpc_total_error(c, k, x_new);

	return 1;
}

///////////////////////////////////////////////////////////////////////////////
//                                Debug                                      //
///////////////////////////////////////////////////////////////////////////////

void CCsmMatcher::debug_correspondences()
{
	/** Do the test */
	find_correspondences_tricks();
	long hash1 = m_pSensScan->CorrHash();
	find_correspondences();
	long hash2 = m_pSensScan->CorrHash();

	if (hash1 != hash2)
	{
		if (1)
			exit(-1);
	}
}


/////////////////////


bool CCsmMatcher::Match(CCsmScan* pRefScan, CCsmScan* pSensScan, sm_result& result)
{
	m_pRefScan = pRefScan;
	m_pSensScan = pSensScan;

	double odometry[3];
	pose_diff_d(m_pSensScan->odometry, m_pRefScan->odometry, odometry);
	double ominus_laser[3], temp[3];
	ominus_d(laser, ominus_laser);
	oplus_d(ominus_laser, odometry, temp);
	oplus_d(temp, laser, first_guess);

#ifdef _MSC_VER
	m_ulStartTime = GetTickCount();
#endif

	double d1 = getDoubleTime();

	/* Do the actual work */
	sm_icp(&result);

	double d = getDoubleTime() - d1;

	if (result.valid)
	{
//		pose_diff_d(result.x, odometry, result.z);
		// 在此验算平均匹配误差值

	}
	return result.valid;
}
