//                            - GEOMETRY.H -
//
//    Defines the following basic geometric concepts:
//      . CPnt
//      . CAngle
//      . CPosture
//      . CPostureDev
//      . CMoveDir
//      . CTurnDir
//      . CTransform
//      . CLine
//      . CArc
//      . CSpp
//      . CSpline
//      . CScp
//
//    Author: Zhang Lei
//    Date:   2001. 9. 7
//

#ifndef __Geometry
#define __Geometry

#include <math.h>
#include <stdio.h>
#include "ZTypes.h"
//#include "Tools.h"
//#include <afxwin.h>
//#include <Filter.h>

// Defines PI
#if !defined PI
#define PI                     (3.14159265f)
#endif

#define TO_DEGREE(x)           (x/PI*180.0f)
#define TO_RADIAN(x)           (x/180.0f*PI)

#ifdef _MFC_VER
class CScreenReference;
#endif

class CPosture;
class CFrame;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPnt".
class /*DllExport*/ CPnt
{
public:
	unsigned int id;        // ID号
	float x;                // 平面直角坐标X位置
	float y;                // 平面直角坐标Y位置
	float a;                // 极坐标角度(单位：弧度)
	float r;                // 极坐标距离

public:
	// The constructor
	CPnt(float fx, float fy, unsigned int _id = 0)
	{
		x = fx;
		y = fy;
		a = 0;
		r = 0;
		id = _id;
	}

	// Default constructor
	CPnt()
	{
		x = 0;
		y = 0;
		a = 0;
		r = 0;
		id = 0;
	}

	// 设置点的坐标
	void Set(float _x, float _y)
	{
		x = _x;
		y = _y;
	}

	// 设置点的极坐标
	void SetPolar(float fAngle, float fRadius, int _id)
	{
		id = _id;
		r = fRadius;
		a = fAngle;
		x = (float)(fRadius * cos(fAngle));
		y = (float)(fRadius * sin(fAngle));
	}

	// 取得对象的引用
	CPnt& GetPntObject() { return *this; }

	// 将点移动指定的距离
	void Move(float dx, float dy);

	// 将点绕指定的中心点进行旋转
	virtual void Rotate(float fAng, float fCx = 0, float fCy = 0);

	// 将点绕指定的中心点进行旋转
	virtual void Rotate(float fAng, const CPnt& ptCenter);

	// 重载 "=="
	bool operator ==(const CPnt& pt) const;

	// 重载  "!="
	bool operator !=(const CPnt& pt) const;

	// 重载 "+="
	void operator += (const CPnt& pt);

	// 重载 "-="
	void operator -= (const CPnt& pt);

	// 重载 "+"
	CPnt operator +(const CPnt& pt) const;

	// 重载 "-"
	CPnt operator -(const CPnt& pt) const;

	// 比较两个点的“大小”(依据“左-下”原则)
	bool operator < (const CPnt& pt) const;

	// 比较两个点的“大小”(依据“左-下”原则)
	bool operator > (const CPnt& pt) const;

	// 计算该点到另一点的直线距离
	float DistanceTo(const CPnt& pt) const;

	// 计算两个点之间的距离的平方
	float Distance2To(const CPnt& pt2) const;

	// 判断两个点是否可以近似地认为是一个点(相距非常近)
	bool IsEqualTo(const CPnt& pt2, float limit) const;

	// 根据迪卡尔坐标计算出点的极坐标
	void UpdatePolar();
	void UpdatePolar(const CPosture& pstRefFrame);

	// 根据极坐标计算出点的迪卡尔坐标
	void UpdateCartisian();

	// 计算该点到另一点的角度距离
	float AngleDistanceTo(const CPnt& pt) const;

	// 从文件装入点数据
	bool Load(FILE* fp);

	// 将点数据写入文件
	bool Save(FILE* fp);

	// 进行坐标正变换
	virtual void Transform(const CFrame& frame);

	// 进行坐标逆变换
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER

	CPnt(const CPoint& point)
	{
		x = (float)point.x;
		y = (float)point.y;
		a = 0;
		r = 0;
		id = 0;
	}

	// “拷备”构造函数
	void operator = (const CPoint& point);

	// 重载 "=="
	bool operator == (const CPoint& point) const;

	// 重载  "!="
	bool operator != (const CPoint& point) const;

	void Dump();

	// 在屏幕上绘制该点
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize = 1, int nLineWidth = 0);
#endif
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CAngle".
enum ANGLE_MODE { IN_RADIAN, IN_DEGREE };

class /*DllExport*/ CAngle
{
public:
	static float m_fReso;        // Resolution for angle computation

public:
	float m_fRad;                // The angle value in radian

public:
	// Convert degree into radian
	static float ToRadian(float deg);

	// Convert radian into degree
	static float ToDegree(float rad);

	// Normalize an angle
	static float NormAngle(float rad);

	static float NormAngle2(float rad);

	// Set new resolution for angle computation
	static float SetReso(float fReso);

public:
	// The constructors
	CAngle(float val, int mode = IN_RADIAN);

	// 根据两个点构造角
	CAngle(const CPnt& pt1, const CPnt& pt2);

	// The default constructor
	CAngle() { m_fRad = 0; }

	// Get the angle value in "degree" in range [0, 360)
	float Degree();

	// Get the angle value in "degree" in range [-180, 180)
	float Degree2();

	// The quadrant of angle: 1/2/3/4
	int Quadrant();

	// 将角旋转一个角度
	void Rotate(float fAng);

	// Normalize an angle to [0, 2*PI]
	float NormAngle();

	// Normalize an angle to [-PI, PI]
	float NormAngle2();

	// Overloaded operators: "!", "+", "-", "+=", "-=", "==", ">", "<", "="
	CAngle operator -() const;
	CAngle operator !() const;
	CAngle operator +(const CAngle& Ang) const;
	CAngle operator -(const CAngle& Ang) const;
	void operator +=(const CAngle& Ang);
	void operator -=(const CAngle& Ang);
	bool operator ==(const CAngle& Ang) const;
	bool operator !=(const CAngle& Ang) const;
	bool operator >(const CAngle& Ang) const;
	bool operator <(const CAngle& Ang) const;
	bool operator >=(const CAngle& Ang) const;
	bool operator <=(const CAngle& Ang) const;

	void operator =(float fRad);
	CAngle operator +(float fRad) const;
	CAngle operator -(float fRad) const;
	void operator +=(float fRad);
	void operator -=(float fRad);
	bool operator ==(float fRad) const;
	bool operator !=(float fRad) const;
	bool operator >(float fRad) const;
	bool operator <(float fRad) const;
	bool operator >=(float fRad) const;
	bool operator <=(float fRad) const;

	bool ApproxEqualTo(const CAngle& ang, float fMaxDiffRad = 0) const;

	// 计算本角与另外一个角的差(只返回正值)
	float GetDifference(const CAngle& another) const;

	bool InRange(const CAngle& ang1, const CAngle& ang2) const;
};

/*DllExport*/ float sin(const CAngle& Ang);
/*DllExport*/ float cos(const CAngle& Ang);
/*DllExport*/ float tan(const CAngle& Ang);
/*DllExport*/ CAngle abs(const CAngle& Ang);
/*DllExport*/ float AngleDiff(float angle1, float angle2);
/*DllExport*/ float NormAngle2(float fRad);
//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CRegion".
//
class /*DllExport*/ CRegion
{
public:
	float fLeft;
	float fTop;
	float fRight;
	float fBottom;

public:
	// The constructor
	CRegion(float _fLeft, float _fTop, float _fRight, float _fBottom)
	{
		fLeft = _fLeft;
		fTop = _fTop;
		fRight = _fRight;          // fRight >= fLeft
		fBottom = _fBottom;        // fBottom <= fTop
	}

	void Create(float _fLeft, float _fTop, float _fRight, float _fBottom)
	{
		fLeft = _fLeft;
		fTop = _fTop;
		fRight = _fRight;          // fRight >= fLeft
		fBottom = _fBottom;        // fBottom <= fTop
	}

	// Default constructor
	CRegion() 
	{
		fLeft = fTop = fRight = fBottom = 0.0f;
	}

	// Get the width of the region
	float Width() {return (fRight - fLeft);}

	// Get the height of the region
	float Height() {return (fTop - fBottom);}

	// 取得中心点坐标
	CPnt GetCenterPoint() 
	{
		CPnt pt;
		pt.x = (fLeft + fRight) / 2;
		pt.y = (fTop + fBottom) / 2;
		
		return pt;
	}

	// Whether the region contains the specified point
	bool Contain(CPnt& pt)
	{
		return (pt.x >= fLeft && pt.x <= fRight &&
				  pt.y >= fBottom && pt.y <= fTop);
	}
	
	// Whether the region contains the specified point (form#2)
	bool Contain(float fX, float fY)
	{
		return (fX >= fLeft && fX <= fRight &&
				  fY >= fBottom && fY <= fTop);
	}
};
class /*DllExport*/ CPolyRegion
{
public:
	CPnt* m_pVertex;
	int       m_nCount;

public:
	CPolyRegion(int nCount = 0, CPnt* pPnt = NULL);
	~CPolyRegion();

	// 判断指定的点是否包含在此多边形区域中
	bool Contain(const CPnt& pt) const;

	// 判断此区域是否包含另外一个指定的区域
	bool Contain(const CPolyRegion& PolyRgn) const;

	bool OverlapWith(const CPolyRegion& PolyRgn) const;

#ifdef _MFC_VER
	// 在屏幕上绘制此直线
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth = 1, int nPointSize = 1, bool bBigVertex = false);
#endif
};

class vector_velocity;
class CLine;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPosture".
//
class /*DllExport*/ CPosture : public CPnt
{
public:
	float fThita;      // The direction angle

public:
	// The constructors
	CPosture(float fX, float fY, float fAngle)
	{
		x = fX;
		y = fY;
		fThita = fAngle;
	}

	CPosture(float fX, float fY, const CAngle& Angle)
	{
		x = fX;
		y = fY;
		fThita = Angle.m_fRad;
	}

	CPosture(const CPnt& pt, const CAngle& Angle)
	{
		x = pt.x;
		y = pt.y;
		fThita = Angle.m_fRad;
	}

	// Default constructor
	CPosture()
	{
		x = y = 0;
		fThita = 0;
	}

	void Create(float _x, float _y, float _thita)
	{
		x = _x;
		y = _y;
		fThita = _thita;
	}

	void Create(const CPnt& pt, const CAngle& ang)
	{
		x = pt.x;
		y = pt.y;
		fThita = ang.m_fRad;
	}

	void SetPnt(const CPnt& pt)
	{
		x = pt.x;
		y = pt.y;
	}

	void SetPnt(float fX, float fY)
	{
		x = fX;
		y = fY;
	}

	void SetAngle(const CAngle& ang)
	{
		fThita = ang.m_fRad;
	}

	void SetAngle(float fAngle)
	{
		fThita = CAngle::NormAngle(fAngle);
	}

	void SetPosture(const CPosture& pst)
	{
		GetPostureObject() = pst;
	}

	void SetPosture(const CPnt& pt, const CAngle& ang)
	{
		SetPnt(pt);
		SetAngle(ang);
	}

	void ReverseAngle()
	{
		fThita = CAngle::NormAngle(fThita + PI);
	}

	void SetPosture(float fX, float fY, float fAngle)
	{
		x = fX;
		y = fY;
		fThita = CAngle::NormAngle(fAngle);
	}

	CPosture& GetPostureObject() { return *this; }

	CAngle GetAngle() const
	{
		CAngle ang(fThita);
		return ang;
	}

	// 将点绕指定的中心点进行旋转
	virtual void Rotate(float fAng, float fCx = 0, float fCy = 0);

	// 将姿态绕指定的中心点进行旋转
	virtual void Rotate(float fAng, CPnt ptCenter);

	void operator += (const CPosture& pstAnother)
	{
		x += pstAnother.x;
		y += pstAnother.y;
		fThita += pstAnother.fThita;
		fThita = CAngle::NormAngle(fThita);
	}

	void operator -= (const CPosture& pstAnother)
	{
		x -= pstAnother.x;
		y -= pstAnother.y;
		fThita -= pstAnother.fThita;
		fThita = CAngle::NormAngle(fThita);
	}

	// 在当前姿态的基础上，根据给定的速度向量，推算出一定时段后的新姿态
	CPosture Deduce(const vector_velocity& vel, float interval);

	// 计算姿态角到一条射线的夹角
	CAngle AngleToLine(const CLine& ln) const;

	void RotatePos(float angle, float cx, float cy);

	// 进行坐标正变换
	virtual void Transform(const CFrame& frame);

	void TransformToGlobal(const CPosture *p1, CPosture *result);

	void TransformToLocal(const CPosture *p1, CPosture *result);

	// 进行坐标逆变换
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER
	// 在屏幕上绘制此姿态
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nCircleRadiusMm, int nArrowLenMm, int nWidth = 1);
#endif
};

CPosture operator + (const CPosture& pst1, const CPosture& pst2);
CPosture operator - (const CPosture& pst1, const CPosture& pst2);

// 根据两次姿态及时间间隔估算物体在这两个姿态间过渡时的速度向量
vector_velocity EstimateVel(CPosture pst1, CPosture pst2, float interval);

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CVelocity".
class /*DllExport*/ CVelocity
{
public:
	float fLinear;
	float fAngular;

public:
	// The constructor
	CVelocity(float fLin, float fAng)
	{
		fLinear = fLin;
		fAngular = fAng;
	}

	// Default constructor
	CVelocity() {}
};

class vector_velocity
{
public:
	float vel_x;
	float vel_y;
	float vel_angle;

public:
	vector_velocity(float _vx = 0, float _vy = 0, float _va = 0)
	{
		vel_x = _vx;
		vel_y = _vy;
		vel_angle = _va;
	}

	void SetZero()
	{
		vel_x = vel_y = vel_angle = 0;
	}
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CMoveDir".
enum MoveDirTag { FORWARD, BACKWARD, LEFTWARD = 0, RIGHTWARD = 1 };

class /*DllExport*/ CMoveDir
{
public:
	MoveDirTag m_tagMoveDir;      // The tag for the move direction

public:
	// The constructor
	CMoveDir(MoveDirTag tagMoveDir) { m_tagMoveDir = tagMoveDir; }

	// Default constructor
	CMoveDir() { m_tagMoveDir = FORWARD; }

	// Copy constructor
	void operator = (const CMoveDir& MoveDir);

	// Assignment of move direction
	void operator = (MoveDirTag tagMoveDir);

	// Test if 2 objects are identical
	bool operator == (const CMoveDir& MoveDir) const;

	// Test if 2 objects are not identical
	bool operator != (const CMoveDir& MoveDir) const;

	// Test if the object is of the specified turn direction
	bool operator == (MoveDirTag tagMoveDir) const;

	// Test if the object is not of the specified turn direction
	bool operator != (MoveDirTag tagMoveDir) const;

	// Get the opposite turn direction
	CMoveDir operator !();
};


//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CTurnDir".
enum TurnDirTag { COUNTER_CLOCKWISE, CLOCKWISE };

class /*DllExport*/ CTurnDir
{
public:
	TurnDirTag m_tagTurnDir;      // The tag for the turn direction

public:
	// The constructor
	CTurnDir(TurnDirTag tagTurnDir) { m_tagTurnDir = tagTurnDir; }

	// Default constructor
	CTurnDir() { m_tagTurnDir = COUNTER_CLOCKWISE; }

	// Copy constructor
	void operator =(CTurnDir TurnDir);

	// Assignment of turn direction
	void operator =(TurnDirTag tagTurnDir);

	// Test if 2 objects are identical
	bool operator == (const CTurnDir& TurnDir) const;

	// Test if 2 objects are not identical
	bool operator != (const CTurnDir& TurnDir) const;

	// Test if the object is of the specified turn direction
	bool operator == (TurnDirTag tagTurnDir) const;

	// Test if the object is not of the specified turn direction
	bool operator != (TurnDirTag tagTurnDir) const;

	// Get the opposite turn direction
	CTurnDir operator !();
};

class CLine;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CTransform".

class /*DllExport*/ CTransform : public CPosture
{
public:
	CTransform(const CPosture& pstLocal);

	// The constructor
	CTransform(const CPnt& ptOrigin, const CAngle& angSlant);

	// Default constructor
	CTransform() {}

	// Init the origin and slant angle of the local frame
	void Init(const CPosture& pstLocal);

	void Create(float _x, float _y, float _angle)
	{
		CPosture::Create(_x, _y, _angle);
	}

	void Create(CLine& line1, CLine& line2);

	// A transformation from the local frame to the world frame
	CPnt GetWorldPoint(const CPnt& ptLocal) const;

	// A reverse transformation from the world frame to the local frame
	CPnt GetLocalPoint(const CPnt& ptWorld) const;

	// A transformation from the local frame to the world frame
	CPosture GetWorldPosture(const CPosture& pstLocal) const;

	// A reverse transformation from the world frame to the local frame
	CPosture GetLocalPosture(const CPosture& pstWorld) const;

	// 将一段直线从局部坐标系到变换到世界坐标系中
	CLine GetWorldLine(const CLine& lnLocal) const;

	// 将一段直线从世界坐标系到变换到局部坐标系中
	CLine GetLocalLine(const CLine& lnWorld) const;
};

//
//   抽象直线: ax + by + c = 0。
//
class /*DllExport*/ CLineBase
{
public:
	int   m_nId;   // ID号
	float a;       // Normal X
	float b;       // Normal Y
	float c;       // 到原点的距离

public:
	CLineBase(float _a, float _b, float _c)
	{
		m_nId = 0;
		a = _a;
		b = _b;
		c = _c;
	}

	// 点斜式构建
	CLineBase(const CPnt& pt, float fAng);

	CLineBase()
	{
		m_nId = 0;
		a = b = c = 0;
	}

	// 直接参数生成
	void DirectCreate(float _a, float _b, float _c)
	{
		m_nId = 0;
		a = _a;
		b = _b;
		c = _c;
	}

	// 点斜式构建直线
	void Create(float x, float y, float fAng);

	// 通过对点云的线性回归求得最优拟合线
	bool CreateFitLine(const CPnt* sp, int num);

	CLineBase& GetLineBaseObject() { return *this; }

	// 计算该直线到指定点的距离
	float DistanceToPoint(const CPnt& pt) const;

	// 求得直线外一点到此直线的投影
	CPnt GetProjectPoint(const CPnt& pt) const;

	// 求得直线外一直线到此直线的投影
	CLine GetProjectLine(const CLine& Line) const;

	// 取得直线的倾斜角(弧度)
	float StdSlantAngleRad() const;
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CLine".
class /*DllExport*/ CLine : public CLineBase
{
public:
	CPnt   m_ptStart;        // The start point
	CPnt   m_ptEnd;          // The end point
	CAngle     m_angSlant;       // Slant angle
	float      m_fTotalLen;      // The length of the line

public:
	// 第一种构造函数
	CLine(const CPnt& ptStart, const CPnt& ptEnd);

	// 第二种构造函数
	CLine(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen);

	// 第三种构造函数
	CLine(const CPosture& pstStart, float fTotalLen);

	// 第四种构造函数
	CLine(const CLine& Line2);

	// 缺省构造函数
	CLine() { m_nId = 0; }

	// 取得自身的引用(主要用于继承)
	CLine& GetLineObject() { return *this; }

	// 根据两点生成线段
	bool Create(const CPnt& ptStart, const CPnt& ptEnd);

	// 根据起始点、倾角和长度生成线段
	bool Create(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen);

	// 计算完整直线段的所有参数(倾角、长度)
	void ComputeParam();

	// 返回起始点
	CPnt& GetStartPoint() { return m_ptStart; }

	// 返回终止点
	CPnt& GetEndPoint() { return m_ptEnd; }

	// 反转线段的方向(将起点、终点对调)
	void Reverse();

	// 改变线段的长度(沿两端延长/缩短)
	bool Resize(float fDist1, float fDist2);

	// 取得直线段的长度
	float Length() const { return m_fTotalLen; }

	// 取得直线段长度的平方
	float Length2() const;

	// 轨迹点函数
	CPnt TrajFun(float fCurLen) const;

	// 取得直线的倾斜角
	CAngle SlantAngle() const { return m_angSlant; }

	// 取得线段的倾斜角。
	float SlantAngleRad() const;

	// 返回线段的中点
	CPnt GetMidpoint() const;

	// 判断一个点是否在此直线(段)上
	bool ContainPoint(const CPnt& pt, bool bExtend = false) const;

	// 判断两条直线是否平行
	bool IsParallelTo(const CLine& Line, float fMaxAngDiff = 0) const;

	// 判断两条直线是否垂直
	bool IsVerticalTo(const CLine& line, float fMaxAngDiff = 0) const;

	// 判断两条直线是否共线
	bool IsColinearWith(const CLine& Line, float fMaxAngDiff, float fMaxDistDiff) const;

	// 取得两条直线的交点
	bool IntersectLineAt(const CLine& Line, CPnt& pt, float& fDist) const;

	//
	//   Intersects two lines.  Returns true if intersection point exists,
	//   false otherwise.  (*px, *py) will hold intersection point,
	//   *onSegment[12] is true if point is on line segment[12].
	//   px, py, onSegment[12] might be NULL.
	//
	bool Intersect(const CLine& line2, float *px, float *py, bool *onSegment1, bool *onSegment2, float fSmallGate = 1e-6) const;

	// 计算该与另一直线的角度差
	CAngle AngleToLine(const CLine& line2) const;

	// 计算该直线与另一条无向直线的角度差(两个方向中较小的)
	CAngle AngleToUndirectionalLine(const CLine& line2) const;

	// 计算该直线到指定点的距离
	float DistanceToPoint(const CPnt& pt) const;

	// 计算该直线到指定点的距离
	float DistanceToPoint(bool bIsSegment, const CPnt& pt, float* pLambda = NULL, CPnt* pFootPoint = NULL) const;

	// 判断一个给定的点是否“触碰”到该直线
	int PointHit(const CPnt& pt, float fDistGate);

	// 将直线绕指定的中心点进行旋转
	virtual void Rotate(float fAng, float fCx = 0, float fCy = 0);

	// 将直线绕指定的中心点进行旋转
	virtual void Rotate(float fAng, CPnt ptCenter);

	// 变换到局部坐标系
	CLine TransformToLocal(CTransform& trans);

	// 变换到全局坐标系
	CLine TransformToGlobal(CTransform& trans);

	// 取得直线段的斜率、Y轴截距和(当直线垂直于X轴时的)X轴截距
	int GetParam(float* k, float* b, float* c) const;

	// 计算直线的两个端点中哪个距离指定的点pt更近，并在fDist中返回此近距离
	int FindNearPoint(const CPnt& pt, float* pDist = NULL) const;

	// 计算本线段到另一线段的投影
	bool GetProjection(const CLine& another, CLine& lineProj, float& fProjDist, float& fProjLen) const;

	// 进行坐标正变换
	virtual void Transform(const CFrame& frame);

	// 进行坐标逆变换
	virtual void InvTransform(const CFrame& frame);

	bool Merge(CLine& Line2, float fMaxAngDiff, float fMaxDistDiff, bool bExtMode = true);

#ifdef _MFC_VER
	// 在屏幕上绘制此直线
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth = 1, int nPointSize = 1, bool bBigVertex = false);
#endif
};

///////////////////////////////////////////////////////////////////////////////
//   定义“矩形类”。
class CRectangle
{
private:
	bool m_bInit;                 // 区域是否已经初始化
	CPnt m_ptLeftTop;             // 矩形左上角位置
	CPnt m_ptRightBottom;         // 矩形右下角位置

public:
	CRectangle(const CPnt& ptLeftTop, const CPnt& ptRightBottom);
	CRectangle(float fLeft, float fTop, float fRight, float fBottom);
	CRectangle();

	// 生成矩形
	bool Create(const CPnt& ptLeftTop, const CPnt& ptRightBottom);

	// 生成矩形
	bool Create(float fLeft, float fTop, float fRight, float fBottom);

	// 清除区域
	void Clear();

	// 取得左上角点的位置
	CPnt GetLeftTopPoint() const { return m_ptLeftTop; }

	// 取得右下角点的位置
	CPnt GetRightBottomPoint() const { return m_ptRightBottom; }

	// 取得中心点的位置
	CPnt GetCenterPoint() const;

	// 判断一个点是否处于矩形以内
	bool Contain(float x, float y) const;

	// 判断一个点是否处于矩形以内
	bool Contain(const CPnt& pt) const;

	// 判断一线段是否处于矩形以内
	bool Contain(const CLine& ln) const;

	// 判断另一个矩形是否处于该矩形以内
	bool Contain(const CRectangle& r) const;

	// 取得最左点X坐标
	float Left() const { return m_ptLeftTop.x; }

	// 取得最右点X坐标
	float Right() const { return m_ptRightBottom.x; }

	// 取得最上点Y坐标
	float Top() const { return m_ptLeftTop.y; }

	// 取得最下点Y坐标
	float Bottom() const { return m_ptRightBottom.y; }

	// 取得矩形区域的宽度
	float Width() const;

	// 取得矩形区域的高度
	float Height() const;

	// 调整矩形区域大小以便容纳给定的点
	void operator += (const CPnt& pt);

	// 调整矩形区域大小以便容纳给定的直线
	void operator += (const CLine& line);

	// 调整矩形区域大小以便容纳给定的矩形区域
	void operator += (const CRectangle& rect);

	// 从文件装入矩形数据
	bool Load(FILE* fp);

	// 将矩形数据写入文件
	bool Save(FILE* fp);

#ifdef _MSC_VER
	// 在屏幕上绘制此矩形
	//void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth);
#endif
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CCircle".
class /*DllExport*/ CCircle
{
public:
	CPnt m_ptCenter;        // The center of the curve
	float    m_fRadius;         // Radius of the replaced arc
	CPnt m_pt;              // The trajectory point

public:
	// The constructor
	CCircle(const CPnt& ptCenter, float fRadius);

	// Default constructor
	CCircle() {}

	// 生成此圆
	void Create(const CPnt& ptCenter, float fRadius);

	// Get the radius of its similar arc
	float Radius() const { return m_fRadius; }

	// 取得周长
	float Perimeter() const { return 2 * PI * m_fRadius; }

	// 取得此圆与一条起点在圆外的直线的第一个交点
	bool IntersectLineAt(const CLine& Line, CPnt& ptNear, float& fDist);

	// 以此圆截取一条直线，并将截取到的新直线保存到NewLine中
	bool CutLine(const CLine& Line, CLine& NewLine) const;

	// 判断一个点是否在圆内(或圆上)
	bool Contain(const CPnt& pt) const;

#ifdef _MFC_VER
	// 在屏幕上绘制此圆
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth);
#endif
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CArc".
class /*DllExport*/ CArc
{
public:
	CPnt     m_ptCenter;        // The center of the curve
	CPnt     m_ptStart;         // Start point
	CPnt     m_ptEnd;           // End point
	CTurnDir m_TurnDir;         // Turn direction
	float    m_fTurnAngle;      // Curve's turn angle
	float    m_fCurRadius;      // Radius at the current point
	float    m_fRadius;         // Radius of the replaced arc
	CPnt     m_pt;              // The trajectory point
	CAngle   m_angTangent;      // The tangent angle
	float    m_fCurvature;      // The curvature

	CAngle   m_angStart;        // Slant angle of the start radius
	CTransform m_Transform;     // Coordinates transformation object

public:
	// The constructor
	CArc(const CPnt& ptCenter, const CPnt& ptStart, const CPnt& ptEnd, CTurnDir TurnDir =
		COUNTER_CLOCKWISE);

	// Default constructor
	CArc() {}

	// Get the total turn angle of the curve 
	float TurnAngle() const { return m_fTurnAngle; }

	// Get the radius of its similar arc
	float Radius() const { return m_fRadius; }

	// Get the current radius
	virtual float CurRadius() const { return m_fCurRadius; }

	// Set the current turn angle to specified a trajectory point
	virtual void SetCurAngle(float fPhi);

	// The trajectory generation function
	virtual CPnt TrajFun() { return m_pt; }

	// The tangent angle generation function
	virtual CAngle TangentFun() { return m_angTangent; }

	// The curvature generation function
	virtual float CurvatureFun() { return m_fCurvature; }
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CSpp".
class /*DllExport*/ CSpp : public CArc
{
public:
	// The constructor
	CSpp(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir =
		COUNTER_CLOCKWISE);

	// Default constructor
	CSpp() {}

	// Set the current turn angle to specified a trajectory point
	virtual void SetCurAngle(float fPhi);
};


//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CSpline".
class /*DllExport*/ CSpline : public CSpp
{
public:
	float m_fRb;               // Radius of the center arc segment
	float m_fBeita;

private:
	void Solve1(float fPhi);
	void Solve2(float fPhi);

public:
	// The constructor
	CSpline(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir =
		COUNTER_CLOCKWISE);

	// Default constructor
	CSpline() {}

	// Set the current turn angle to specified a trajectory point
	virtual void SetCurAngle(float fPhi);
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CScp".
class /*DllExport*/ CScp
{
public:
	CPnt   m_ptStart;            // Start point
	CPnt   m_ptEnd;              // End point
	CAngle m_angLane;            // Slant angle of the lanes
	CPnt   m_pt;                 // Trajectory point
	CPnt   m_ptLocal;            // Local trajectory point
	CAngle m_angTangent;         // Tangent angle with respect to world frame
	CAngle m_angTangent0;        // Tangent angle with respect to the lane
	float  m_fCurvature;         // Curvature

	float  m_fXe;                // Xe
	float  m_fYe;                // Ye
	CAngle m_angShift;           // Curve's shifting angle (to the lane)
	CTransform m_Transform;      // Coordinate transformation object

public:
	// The constructor
	CScp(CPnt& ptStart, CPnt& ptEnd, CAngle& angLane);

	// Default constructor
	CScp() {}

	// Caculate the X distance of the curve
	float GetX();

	// Set the current X distance to specified a trajectory point
	void SetCurX(float fX);

	// The trajectory generation function
	CPnt TrajFun();

	// The tangent angle generation function
	CAngle TangentFun(bool bWorldFrame = true);

	// The curvature generation function
	float CurvatureFun();

	// Get the curve's shifting angle with respect to the lane
	CAngle& ShiftAngle();

	float NewtonRoot(float fXk, float fY);

	float ScpFun(float fXk, float fY);
	float ScpFun_(float fXk);

	// Find the X coordinate of the reference point
	bool FindRefX(CPnt& pt, float& fRefX, float &fErrX);

	// Find the error in Y direction
	bool FindErrY(CPnt& pt, float& fErrY);
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CNewCurve".

class /*DllExport*/ CNewCurve : public CArc
{
private:
	float X0, Y0;
	float X1, Y1;
	float X2, Y2;
	float X3, Y3;

	float A1, A2;
	float B1, B2, B3;
	float C1, C2, C3;
	float D1, D2, D3;
	float E1, E2;
	float F1, F2, F3, F4, F5, F6;

public:
	float Ds, DThita;

public:
	// The constructor
	CNewCurve(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir);

	// Default constructor
	CNewCurve() {}

	// Set the current turn angle to specified a trajectory point
	void SetCurAngle(float fPhi);
};
#endif
