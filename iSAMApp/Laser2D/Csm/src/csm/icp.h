#ifndef _H_ICP_
#define _H_ICP_

#include <csm/csm_all.h>

void compute_covariance_exact(CCsmScan* laser_ref, CCsmScan* laser_sens, 
	const gsl_vector* x, val* cov0_x, val* dx_dy1, val* dx_dy2);

void swap_double(double* a, double* b);

#endif
