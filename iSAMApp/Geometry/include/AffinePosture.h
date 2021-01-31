#ifndef __AffinePosture
#define __AffinePosture

#include "Geometry.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>

Eigen::Affine3d PostureToAffine(double x, double y, double yaw);
Eigen::Affine3d PostureToAffine(const CPosture& pst);
CPosture AffineToPosture(const Eigen::Affine3d& affine);

#endif
