//                            - GEOMETRY.H -
//
//    Defines the following basic geometric concepts:
//      . CPoint2d
//      . CAngle
//      . CRegion
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
#include "ZTypes.h"
#include "Tools.h"

#ifdef _MSC_VER
class CScreenReference;
#endif
class CDC;
//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPoint2d".
class /*DllExport*/ CPoint2d
{
public:
	unsigned int id;        // ID号
	float x;                // 平面直角坐标X位置
	float y;                // 平面直角坐标Y位置
	float a;                // 极坐标角度(单位：弧度)
	float r;                // 极坐标距离

public:
	// The constructor
	CPoint2d(float fx, float fy, unsigned int _id = 0)
	{
		x = fx;
		y = fy;
		id = _id;
	}

	// Default constructor
	CPoint2d() 
	{
		x = 0;
		y = 0;
		id = 0;
	}

	CPoint2d(CPoint& point)
	{
		x = (float)point.x;
		y = (float)point.y;
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

	// “拷备”构造函数
	void operator = (const CPoint& point);

	// 取得对象的引用
	CPoint2d& GetPoint2dObject() {return *this;}

	// 将点移动指定的距离
	void Move(float dx, float dy);

	// 将点绕指定的中心点进行旋转
	virtual void Rotate(float fAng, float fCx = 0, float fCy = 0);

	// 将点绕指定的中心点进行旋转
	virtual void Rotate(float fAng, CPoint2d ptCenter);

	// 重载 "=="
	BOOL operator ==(const CPoint2d& pt);

	// 重载  "!="
	BOOL operator !=(const CPoint2d& pt);

	// 重载 "=="
	BOOL operator ==(CPoint& point);

	// 重载  "!="
	BOOL operator !=(CPoint& point);

	// 计算该点到另一点的直线距离
	float DistanceTo(const CPoint2d& pt);

	// 计算两个点之间的距离的平方
	float Distance2To(const CPoint2d& pt2);

	// 判断两个点是否可以近似地认为是一个点(相距非常近)
	bool IsEqualTo(const CPoint2d& pt2, float limit);

	// 根据迪卡尔坐标计算出点的极坐标
	void UpdatePolar();

	// 根据极坐标计算出点的迪卡尔坐标
	void UpdateCartisian();

	// 计算该点到另一点的角度距离
	float AngleDistanceTo(const CPoint2d& pt);

	// 从文件装入点数据
	bool Load(FILE* fp);

	// 将点数据写入文件
	bool Save(FILE* fp);

#ifdef _MSC_VER
	void Dump();

	// 在屏幕上绘制该点
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize = 1);
#endif
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CAngle".
enum ANGLE_MODE {IN_RADIAN, IN_DEGREE};

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
	CAngle(CPoint2d& pt1, CPoint2d& pt2);

	// The default constructor
	CAngle() {}

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
	CAngle operator -();
	CAngle operator !();
	CAngle operator +(const CAngle& Ang);
	CAngle operator -(const CAngle& Ang);
	void operator +=(const CAngle& Ang);
	void operator -=(const CAngle& Ang);
	BOOL operator ==(const CAngle& Ang);
	BOOL operator !=(const CAngle& Ang);
	BOOL operator >(const CAngle& Ang);
	BOOL operator <(const CAngle& Ang);
	BOOL operator >=(const CAngle& Ang);
	BOOL operator <=(const CAngle& Ang);
	
	void operator =(float fRad);
	CAngle operator +(float fRad);
	CAngle operator -(float fRad);
	void operator +=(float fRad);
	void operator -=(float fRad);
	BOOL operator ==(float fRad);
	BOOL operator !=(float fRad);
	BOOL operator >(float fRad);
	BOOL operator <(float fRad);
	BOOL operator >=(float fRad);
	BOOL operator <=(float fRad);

	BOOL ApproxEqualTo(CAngle& ang, float fMaxDiffRad = 0);

	// 计算本角与另外一个角的差(只返回正值)
	float GetDifference(const CAngle& another);

	bool InRange(CAngle& ang1, CAngle& ang2);
};

/*DllExport*/ float sin(const CAngle& Ang);
/*DllExport*/ float cos(const CAngle& Ang);
/*DllExport*/ float tan(const CAngle& Ang);
/*DllExport*/ CAngle abs(const CAngle& Ang);
/*DllExport*/ float AngleDiff(float angle1, float angle2);

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
	CPoint2d GetCenterPoint() 
	{
		CPoint2d pt;
		pt.x = (fLeft + fRight) / 2;
		pt.y = (fTop + fBottom) / 2;
		
		return pt;
	}

	// Whether the region contains the specified point
	BOOL Contain(CPoint2d& pt)
	{
		return (pt.x >= fLeft && pt.x <= fRight &&
				  pt.y >= fBottom && pt.y <= fTop);
	}
	
	// Whether the region contains the specified point (form#2)
	BOOL Contain(float fX, float fY)
	{
		return (fX >= fLeft && fX <= fRight &&
				  fY >= fBottom && fY <= fTop);
	}
};

class /*DllExport*/ CPolyRegion
{
public:
	CPoint2d* m_pVertex;
	int       m_nCount;

public:
	CPolyRegion(int nCount = 0, CPoint2d* pPnt = NULL);
	~CPolyRegion();

	// 判断指定的点是否包含在此多边形区域中
	BOOL Contain(CPoint2d& pt);

	// 判断此区域是否包含另外一个指定的区域
	BOOL Contain(CPolyRegion& PolyRgn);

	BOOL OverlapWith(CPolyRegion& PolyRgn);

#ifdef _MSC_VER
	// 在屏幕上绘制此直线
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth = 1, int nPointSize = 1, BOOL bBigVertex = FALSE);
#endif
};

class vector_velocity;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPosture".
//
class /*DllExport*/ CPosture : public CPoint2d
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
	
	CPosture(float fX, float fY, CAngle& Angle)
	{
		x = fX;
		y = fY;
		fThita = Angle.m_fRad;
	}
	
	CPosture(CPoint2d& pt, CAngle& Angle)
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

	void SetPnt(CPoint2d& pt)
	{
		x = pt.x;
		y = pt.y;
	}
	
	void SetPnt(float fX, float fY)
	{
		x = fX;
		y = fY;
	}
	
	void SetAngle(CAngle& ang)
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

	void SetPosture(CPoint2d& pt, CAngle& ang)
	{
		SetPnt(pt);
		SetAngle(ang);
	}
	
	void SetPosture(float fX, float fY, float fAngle)
	{
		x = fX;
		y = fY;
		fThita = CAngle::NormAngle(fAngle);
	}

	CPosture& GetPostureObject() {return *this;}

	CAngle GetAngle()
	{
		return CAngle(fThita);
	}

	// 将点绕指定的中心点进行旋转
	virtual void Rotate(float fAng, float fCx = 0, float fCy = 0);

	// 将姿态绕指定的中心点进行旋转
	virtual void Rotate(float fAng, CPoint2d ptCenter);
	
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
	CPosture Deduce(vector_velocity& vel, float interval);

	void TransformToLocal(const CPosture* p1, CPosture *result);
	void TransformToGlobal(const CPosture* p1, CPosture* result);
	void RotatePos(float angle, float cx, float cy);
};

CPosture operator + (CPosture& pst1, CPosture& pst2);
CPosture operator - (CPosture& pst1, CPosture& pst2);

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
enum MoveDirTag {FORWARD, BACKWARD, LEFTWARD = 0, RIGHTWARD = 1};

class /*DllExport*/ CMoveDir
{
public:
	MoveDirTag m_tagMoveDir;      // The tag for the move direction

public:
	// The constructor
	CMoveDir(MoveDirTag tagMoveDir) {m_tagMoveDir = tagMoveDir;}

	// Default constructor
	CMoveDir() {m_tagMoveDir = FORWARD;}

	// Copy constructor
	void operator =(CMoveDir& MoveDir);

	// Assignment of move direction
	void operator =(MoveDirTag tagMoveDir);

	// Test if 2 objects are identical
	BOOL operator ==(CMoveDir& MoveDir);

	// Test if 2 objects are not identical
	BOOL operator !=(CMoveDir& MoveDir);

	// Test if the object is of the specified turn direction
	BOOL operator ==(MoveDirTag tagMoveDir);

	// Test if the object is not of the specified turn direction
	BOOL operator !=(MoveDirTag tagMoveDir);

	// Get the opposite turn direction
	CMoveDir operator !();
};


//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CTurnDir".
enum TurnDirTag {COUNTER_CLOCKWISE, CLOCKWISE};

class /*DllExport*/ CTurnDir
{
public:
	TurnDirTag m_tagTurnDir;      // The tag for the turn direction

public:
	// The constructor
	CTurnDir(TurnDirTag tagTurnDir) {m_tagTurnDir = tagTurnDir;}

	// Default constructor
	CTurnDir() {m_tagTurnDir = COUNTER_CLOCKWISE;}

	// Copy constructor
	void operator =(CTurnDir TurnDir);

	// Assignment of turn direction
	void operator =(TurnDirTag tagTurnDir);

	// Test if 2 objects are identical
	BOOL operator ==(CTurnDir& TurnDir);

	// Test if 2 objects are not identical
	BOOL operator !=(CTurnDir& TurnDir);

	// Test if the object is of the specified turn direction
	BOOL operator ==(TurnDirTag tagTurnDir);

	// Test if the object is not of the specified turn direction
	BOOL operator !=(TurnDirTag tagTurnDir);

	// Get the opposite turn direction
	CTurnDir operator !();
};

class CLine;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CTransform".

class /*DllExport*/ CTransform : public CPosture
{
public:
	// The constructor
	CTransform(CPoint2d& ptOrigin, CAngle& angSlant);

	// Default constructor
	CTransform() {}
    
    // Init the origin and slant angle of the local frame
	void Init(CPosture& pstLocal);
	
	void Create(float _x, float _y, float _angle)
	{
		CPosture::Create(_x, _y, _angle);
	}

	void Create(CLine& line1, CLine& line2);

	// A transformation from the local frame to the world frame
	CPoint2d GetWorldPoint(CPoint2d& ptLocal);

	// A reverse transformation from the world frame to the local frame
	CPoint2d GetLocalPoint(CPoint2d& ptWorld);

	// A transformation from the local frame to the world frame
	CPosture GetWorldPosture(CPosture& pstLocal);

	// A reverse transformation from the world frame to the local frame
	CPosture GetLocalPosture(CPosture& pstWorld);
};


//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CLine".
class /*DllExport*/ CLine
{
public:
	int        m_nId;            // ID号
	CPoint2d   m_ptStart;        // The start point
	CPoint2d   m_ptEnd;          // The end point
	CPoint2d   m_pt;             // Working point for trajectory generation
	CAngle     m_angSlant;       // Slant angle
	float      m_fTotalLen;      // The length of the line

public:
	// Constructor form #1
	CLine(CPoint2d& ptStart, CPoint2d& ptEnd);

	// Constructor form #2
	CLine(CPoint2d& ptStart, CAngle& angSlant, float fTotalLen);

	// Constructor form #3
	CLine(const CLine& Line2);

	// The default constructor
	CLine() {}

	// 取得自身的引用(主要用于继承)
	CLine& GetLineObject() {return *this;}

	// 根据两点生成线段
	void Create(CPoint2d& ptStart, CPoint2d& ptEnd);

	// 根据一点和直线的倾斜角生成线段
	void Create(CPoint2d& ptStart, CAngle& angSlant, float fTotalLen);

	// 计算完整直线段的所有参数(倾角、长度)
	void ComputeParam();

	// 反转线段的方向(将起点、终点对调)
	void Reverse();

	// 改变线段的长度(沿两端延长/缩短)
	void Resize(float fDist1, float fDist2);

	// Get the size of the line
	float Length();

	// 取得线段长度的平方
	float Length2();

	// The trajectory generation function
	CPoint2d& TrajFun(float fCurLen);

	// Get the line's slant angle
	CAngle& SlantAngle();

	// 取得线段的倾斜角。
	float GetAngle() const;

	// 返回线段的中点
	CPoint2d GetCenterPoint();

	// The curvature generation function
	float CurvatureFun();

	// 判断一个点是否在此直线(段)上
	BOOL ContainPoint(CPoint2d& pt, BOOL bExtend = FALSE);

	// 判断两条直线是否平行
	BOOL IsParallelTo(CLine& Line, float fMaxAngDiff = 0);

	// 判断两条直线是否垂直
	bool IsVerticalTo(CLine& line, float fMaxAngDiff = 0);

	// 判断两条直线是否共线
	BOOL IsColinearWith(CLine& Line, float fMaxAngDiff, float fMaxDistDiff);

	// 取得两条直线的交点
	BOOL IntersectLineAt(CLine& Line, CPoint2d& pt, float& fDist);

	//
	//   Intersects two lines.  Returns TRUE if intersection point exists,
	//   FALSE otherwise.  (*px, *py) will hold intersection point,
	//   *onSegment[12] is TRUE if point is on line segment[12].
	//   px, py, onSegment[12] might be NULL.
	//
	BOOL Intersect(CLine& line2, float *px, float *py, BOOL *onSegment1, BOOL *onSegment2);

	// 将此线段与另外一条线段合并
	BOOL Merge(CLine& Line2, float fMaxAngDiff, float fMaxDistDiff, bool bExtMode = true);

	// 计算该与另一直线的角度差
	CAngle AngleToLine(CLine& line2);

	// 计算该直线与另一条无向直线的角度差(两个方向中较小的)
	CAngle AngleToUndirectionalLine(CLine& line2);

	// 计算该直线到指定点的距离
	float DistanceToPoint(BOOL bIsSegment, const CPoint2d& pt, float* pLambda = NULL, CPoint2d* pFootPoint = NULL);

	// 将直线绕指定的中心点进行旋转
	virtual void Rotate(float fAng, float fCx = 0, float fCy = 0);

	// 将直线绕指定的中心点进行旋转
	virtual void Rotate(float fAng, CPoint2d ptCenter);

	// 取得直线段的斜率、Y轴截距和(当直线垂直于X轴时的)X轴截距
	int GetParam(float* k, float* b, float* c);

	// 计算直线的两个端点中哪个距离指定的点pt更近，并在fDist中返回此近距离
	int FindNearPoint(CPoint2d& pt, float* pDist = NULL);

#ifdef _MSC_VER
	// 在屏幕上绘制此直线
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth = 1, int nPointSize = 1, BOOL bBigVertex = FALSE);
#endif
};


//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CCircle".
class /*DllExport*/ CCircle
{
public:
	CPoint2d m_ptCenter;        // The center of the curve
	float    m_fRadius;         // Radius of the replaced arc
	CPoint2d m_pt;              // The trajectory point

public:
	// The constructor
	CCircle(CPoint2d& ptCenter, float fRadius);

	// Default constructor
	CCircle() {}

	// 生成此圆
	void Create(CPoint2d& ptCenter, float fRadius);

	// Get the radius of its similar arc
	float Radius() {return m_fRadius;}

	// 取得周长
	float Perimeter() {return 2 * PI * m_fRadius;}

	// 取得此圆与一条起点在圆外的直线的第一个交点
	BOOL IntersectLineAt(CLine& Line, CPoint2d& ptNear, float& fDist);

	// 以此圆截取一条直线，并将截取到的新直线保存到NewLine中
	bool CutLine(CLine& Line, CLine& NewLine);

	// 判断一个点是否在圆内(或圆上)
	bool Contain(const CPoint2d& pt);

#ifdef _MSC_VER
	// 在屏幕上绘制此圆
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth);
#endif
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CArc".
class /*DllExport*/ CArc
{
public:
	CPoint2d     m_ptCenter;        // The center of the curve
	CPoint2d     m_ptStart;         // Start point
	CPoint2d     m_ptEnd;           // End point
	CTurnDir m_TurnDir;         // Turn direction
	float    m_fTurnAngle;      // Curve's turn angle
	float    m_fCurRadius;      // Radius at the current point
	float    m_fRadius;         // Radius of the replaced arc
	CPoint2d     m_pt;              // The trajectory point
	CAngle   m_angTangent;      // The tangent angle
	float    m_fCurvature;      // The curvature

	CAngle   m_angStart;        // Slant angle of the start radius
	CTransform m_Transform;     // Coordinates transformation object

public:
	// The constructor
	CArc(CPoint2d& ptCenter, CPoint2d& ptStart, CPoint2d& ptEnd, CTurnDir TurnDir =
		  COUNTER_CLOCKWISE);

	// Default constructor
	CArc() {}

	// Get the total turn angle of the curve 
	float TurnAngle() {return m_fTurnAngle;}

	// Get the radius of its similar arc
	float Radius() {return m_fRadius;}

	// Get the current radius
	virtual float CurRadius() {return m_fCurRadius;}

	// Set the current turn angle to specified a trajectory point
	virtual void SetCurAngle(float fPhi);

	// The trajectory generation function
	virtual CPoint2d& TrajFun() {return m_pt;}

	// The tangent angle generation function
	virtual CAngle& TangentFun() {return m_angTangent;}

	// The curvature generation function
	virtual float CurvatureFun() {return m_fCurvature;}
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CSpp".
class /*DllExport*/ CSpp : public CArc
{
public:
	// The constructor
	CSpp(CPoint2d& ptCenter, CPoint2d& ptStart, CPoint2d& ptEnd, CTurnDir TurnDir =
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
	CSpline(CPoint2d& ptCenter, CPoint2d& ptStart, CPoint2d& ptEnd, CTurnDir TurnDir =
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
	CPoint2d   m_ptStart;            // Start point
	CPoint2d   m_ptEnd;              // End point
	CAngle m_angLane;            // Slant angle of the lanes
	CPoint2d   m_pt;                 // Trajectory point
	CPoint2d   m_ptLocal;            // Local trajectory point
	CAngle m_angTangent;         // Tangent angle with respect to world frame
	CAngle m_angTangent0;        // Tangent angle with respect to the lane
	float  m_fCurvature;         // Curvature

	float  m_fXe;                // Xe
	float  m_fYe;                // Ye
	CAngle m_angShift;           // Curve's shifting angle (to the lane)
	CTransform m_Transform;      // Coordinate transformation object

public:
	// The constructor
	CScp(CPoint2d& ptStart, CPoint2d& ptEnd, CAngle& angLane);

	// Default constructor
	CScp() {}

	// Caculate the X distance of the curve
	float GetX();

	// Set the current X distance to specified a trajectory point
	void SetCurX(float fX);

	// The trajectory generation function
	CPoint2d& TrajFun();

	// The tangent angle generation function
	CAngle& TangentFun(BOOL bWorldFrame = TRUE);

	// The curvature generation function
	float CurvatureFun();

	// Get the curve's shifting angle with respect to the lane
	CAngle& ShiftAngle();

	float NewtonRoot(float fXk, float fY);

	float ScpFun(float fXk, float fY);
	float ScpFun_(float fXk);

	// Find the X coordinate of the reference point
	BOOL FindRefX(CPoint2d& pt, float& fRefX, float &fErrX);

	// Find the error in Y direction
	BOOL FindErrY(CPoint2d& pt, float& fErrY);
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
	CNewCurve(CPoint2d& ptCenter, CPoint2d& ptStart, CPoint2d& ptEnd, CTurnDir TurnDir);

	// Default constructor
	CNewCurve() {}

	// Set the current turn angle to specified a trajectory point
	void SetCurAngle(float fPhi);
};
#endif
