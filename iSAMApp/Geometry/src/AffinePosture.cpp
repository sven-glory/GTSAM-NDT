#include "stdafx.h"
#include "AffinePosture.h"

///////////////////////////////////////////////////////////////////////////////

Eigen::Affine3d PostureToAffine(double x, double y, double yaw)
{
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
		 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
		 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

	Eigen::Translation3d v(x, y, 0);
	Eigen::Affine3d T = v * m;

	return T;
}

Eigen::Affine3d PostureToAffine(const CPosture& pst)
{
	return PostureToAffine(pst.x, pst.y, pst.fThita);
}

CPosture AffineToPosture(const Eigen::Affine3d& affine)
{
	CPosture pst;
	pst.x = (float)affine.translation().x();
	pst.y = (float)affine.translation().y();
	pst.fThita = (float)affine.rotation().eulerAngles(0, 1, 2)(2);
	return pst;
}
