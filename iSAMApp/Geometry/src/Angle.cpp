//                            - ANGLE.CPP -
//
//   Implementation of the geometric conception "angle".
//
//   Author: Zhang Lei
//   Date:   2001. 9. 7
//

#include "stdafx.h"

#include <math.h>
#include "Geometry.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

float CAngle::m_fReso = PI/180;     // Default resolution is 1 degree.

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CAngle".

//
//   CAngle: The constructor.
//
CAngle::CAngle(float fValue, int nMode)
{
	// Convert into radian if neccessary
	m_fRad = (nMode == IN_RADIAN) ? fValue : ToRadian(fValue);

	// Normalize the angle
	m_fRad = NormAngle(m_fRad);
}

//
//   根据两个点构造角。
//
CAngle::CAngle(const CPnt& pt1, const CPnt& pt2)
{
	// Convert into radian if neccessary
	m_fRad = (float)atan2(pt2.y - pt1.y, pt2.x - pt1.x);

	// Normalize the angle
	m_fRad = NormAngle(m_fRad);
}

//
//    Degree: Get the angle value in "degree".
//
float CAngle::Degree()
{
	return ToDegree(NormAngle());
}

//
//    Degree: Get the angle value in "degree".
//
float CAngle::Degree2()
{
	return ToDegree(NormAngle2());
}

//
//    Quadrant: Return the angle's Quadrant. (1/2/3/4).
//
int CAngle::Quadrant()
{
	if (m_fRad < PI/2)                    // Quadrant 1
		return 1;
	else if (m_fRad < PI)                 // Quadrant 2
		return 2;
	else if (m_fRad < 3*PI/2)             // Quadrant 3
		return 3;
	else                                  // Quadrant 4
		return 4;
}

//
//   ToRadian: Convert a degree angle into a radian value.
//
float CAngle::ToRadian(float fDeg)
{
	return (PI/180) * fDeg;
}

//
//   ToDegree: Convert a radian angle into a degree value.
//
float CAngle::ToDegree(float fRad)
{
	return (180/PI) * fRad;
}

//
//    NormAngle: Normalize an radian angle into the range [0, 2*PI).
//
float CAngle::NormAngle(float fRad)
{
	// Scale the angle into [0, +)
	while(fRad < 0)
		fRad += 2*PI;

	// Scale the angle into [0, 2*PI)
	while (fRad >= 2*PI)
		fRad -= 2*PI;

	return fRad;
}

//
//    NormAngle: Normalize an radian angle into the range [-PI, PI).
//
float CAngle::NormAngle2(float fRad)
{
	// Scale the angle into [0, +)
	while(fRad < -PI)
		fRad += 2*PI;

	// Scale the angle into [0, 2*PI)
	while (fRad >= PI)
		fRad -= 2*PI;

	return fRad;
}

//
//    NormAngle: Normalize the angle into the range [0, 2 * PI).
//
float CAngle::NormAngle()
{
	float fRad = m_fRad;

   // Scale the angle into [0, +)
   while(fRad < 0)
      fRad += 2*PI;

   // Scale the angle into [0, 2 * PI)
   while (fRad >= 2 * PI)
      fRad -= 2*PI;

   return fRad;
}

//
//    NormAngle: Normalize the angle into the range [-PI, PI).
//
float CAngle::NormAngle2()
{
	float fRad = m_fRad;

	// Scale the angle into [-PI, +)
	while(fRad < -PI)
		fRad += 2*PI;

	// Scale the angle into [-PI, PI)
	while (fRad >= PI)
		fRad -= 2*PI;

	return fRad;
}

//
//   SetAngleReso: Set the resolution (in radian) of angle comparison.
//
float CAngle::SetReso(float fReso)
{
	// Save old resolution
	float fTemp = m_fReso;

	// Set new resolution
	m_fReso = fReso;

	// Return the old reso value
	return fTemp;               
}

//
//   将角旋转一个角度。
//
void CAngle::Rotate(float fAng)
{
	m_fRad = NormAngle(m_fRad + fAng);
}

//
//   Operator "-": Return the negation of the angle.
//
CAngle CAngle::operator -() const
{
	return CAngle(-m_fRad);
}

//
//   Operator "!": Return the reverse-directioned angle.
//
CAngle CAngle::operator !() const
{
	return CAngle(m_fRad + PI);
}

//
//   Operator "+": Return the sum of 2 angles.
//
CAngle CAngle::operator +(const CAngle& Ang) const
{
	return CAngle(m_fRad + Ang.m_fRad);
}

//
//   Operator "-": Return the difference of 2 angles.
//
CAngle CAngle::operator -(const CAngle& Ang) const
{
	return CAngle(m_fRad - Ang.m_fRad);
}

//
//   Operator "+=": Increment of angle.
//
void CAngle::operator +=(const CAngle& Ang)
{
	m_fRad = NormAngle(m_fRad + Ang.m_fRad);
}

//
//   Operator "-=": Decrement of angle.
//
void CAngle::operator -=(const CAngle& Ang)
{
	m_fRad = NormAngle(m_fRad - Ang.m_fRad);
}

//
//   Operator "==": Test if the 2 given angles are equal.
//
//   NOTE: 
//      If the difference of the 2 angles is less than the "resolution",
//   the 2 angles will be regarded as equal.
//
bool CAngle::operator ==(const CAngle& Ang) const
{
	float fTemp = (float)fabs(m_fRad - Ang.m_fRad);
	return (fTemp < m_fReso || 2*PI - fTemp < m_fReso);
}

//
//   Operator "!=": Test if the 2 given angles are not equal.
//    
bool CAngle::operator !=(const CAngle& Ang) const
{
	return !(*this == Ang);
}

//
//   判断两个角是否近似相等。
//
bool CAngle::ApproxEqualTo(const CAngle& Ang, float fMaxDiffRad) const
{
	if (fMaxDiffRad == 0)
		fMaxDiffRad = m_fReso;

	float fTemp = (float)fabs(m_fRad - Ang.m_fRad);
	return (fTemp < fMaxDiffRad || 2*PI - fTemp < fMaxDiffRad);
}

//
//   Operator ">": Test if the first angle is bigger than the second one.
//
bool CAngle::operator >(const CAngle& Ang) const
{
	return ((m_fRad > Ang.m_fRad) && (*this != Ang));
}

//
//    Operator "<": Test if the first angle is smaller than the second one.
//
bool CAngle::operator <(const CAngle& Ang) const
{
	return ((m_fRad < Ang.m_fRad) && (*this != Ang));
}

//
//   Operator ">=": Test if the first angle is bigger than or equal to
//   the second one.
//
bool CAngle::operator >=(const CAngle& Ang) const
{
	return ((m_fRad > Ang.m_fRad) || (*this == Ang));
}

//
//   Operator "<=": Test if the first angle is smaller than or equal to
//   the second one.
//
bool CAngle::operator <=(const CAngle& Ang) const
{
	return ((m_fRad < Ang.m_fRad) || (*this == Ang));
}

//
//   Operator "=": Set the angle value.
//
void CAngle::operator =(float fRad)
{
	m_fRad = NormAngle(fRad);
}

//
//   Operator "+": Return the sum of 2 angles.
//
CAngle CAngle::operator +(float fAngle) const
{
	return CAngle(m_fRad + fAngle);
}

//
//   Operator "-": Return the difference of 2 angles.
//
CAngle CAngle::operator -(float fAngle) const
{
	return CAngle(m_fRad - fAngle);
}

//
//   Operator "+=": Increment of angle.
//
void CAngle::operator +=(float fAngle)
{
	m_fRad = NormAngle(m_fRad + fAngle);
}

//
//   Operator "-=": Decrement of angle.
//
void CAngle::operator -=(float fAngle)
{
	m_fRad = NormAngle(m_fRad - fAngle);
}

//
//   Operator "==": Check whether the angle is equal to the specified radian value.
//
bool CAngle::operator ==(float fRad) const
{
	float fTemp = (float)fabs(m_fRad - NormAngle(fRad));
	return (fTemp < m_fReso || 2*PI - fTemp < m_fReso);
}

bool CAngle::operator !=(float fAngle) const
{
	return !(*this == fAngle);
}

bool CAngle::operator >(float fAngle) const
{
	fAngle = NormAngle(fAngle);
	return ((m_fRad > fAngle) && (*this != fAngle));
}

bool CAngle::operator <(float fAngle) const
{
	fAngle = NormAngle(fAngle);
	return ((m_fRad < fAngle) && (*this != fAngle));
}

bool CAngle::operator >=(float fAngle) const
{
	fAngle = NormAngle(fAngle);
	return ((m_fRad > fAngle) || (*this == fAngle));
}

bool CAngle::operator <=(float fAngle) const
{
	fAngle = NormAngle(fAngle);
	return ((m_fRad < fAngle) || (*this == fAngle));
}

//
//   计算本角与另外一个角的差(只返回正值)。如果两个角为跨周期的角，
//   只考虑它们最小的差值。
//
float CAngle::GetDifference(const CAngle& another) const
{
	float ang1 = NormAngle(m_fRad);
	float ang2 = NormAngle(another.m_fRad);

	if (ang1 > ang2)
	{
		if (ang1 - ang2 > PI)
			return 2*PI + ang2 - ang1;
		else
			return ang1 - ang2;
	}
	else if (ang1 < ang2)
	{
		if (ang2 - ang1 > PI)
			return 2*PI + ang1 - ang2;
		else
			return ang2 - ang1;
	}
	else
		return 0;
}

//
//   判断本角是否在从ang1到ang2(逆时针旋转)的范围之内。
//
bool CAngle::InRange(const CAngle& ang1, const CAngle& ang2) const
{
	CAngle angDiff2 = (CAngle)ang2 - ang1;
	CAngle angDiff1 = *this - ang1;
	return angDiff2.m_fRad > angDiff1.m_fRad;
}

//////////////////////////////////////////////////////////////////////////////
//   The followings are some overloaded functions: sin(), cos(), tan()

//
//   sin: Overloaded function of sin().
//
float sin(const CAngle& Ang)
{
	return (float)sin((double)Ang.m_fRad);
}

//
//   cos: Overloaded function of cos().
//
float cos(const CAngle& Ang)
{
	return (float)cos((double)Ang.m_fRad);
}

//
//   tan: Overloaded function of tan().
//
float tan(const CAngle& Ang)
{
	return (float)tan((double)Ang.m_fRad);
}

//
//   Get the absolute value of an angle. (the absolute value of -30 degree is 30 degree)
//
CAngle abs(const CAngle& Ang)
{
	CAngle ang = Ang;
	if (ang.m_fRad > PI)
		ang.m_fRad = 2*PI - ang.m_fRad;

	return ang;
}

//
//   求两个角的差的绝对值，并把结果归一化到(0, PI)中。
//
float AngleDiff(float angle1, float angle2)
{
	float angle = CAngle::NormAngle(angle1 - angle2);

	if (angle > PI)
		angle = PI * 2 - angle;

	return angle;
}

float NormAngle2(float fRad)
{
	// Scale the angle into [-PI, +)
	while(fRad < -PI)
		fRad += 2*PI;

	// Scale the angle into [-PI, PI)
	while (fRad >= PI)
		fRad -= 2*PI;

	return fRad;
}
