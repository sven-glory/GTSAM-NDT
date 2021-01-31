#ifndef __CLineFeature
#define __CLineFeature

#include "Geometry.h"
#include "Feature.h"
#include <afxwin.h>

///////////////////////////////////////////////////////////////////////////////
//   定义“特征”基类。

class CLineFeature : public CFeature, public CLine
{
public:
    float m_fNormalX;
	 float m_fNormalY;          // normal vector
    float m_fDistOrigin;       // 到原点的距离

    long  m_lStart;
	 long  m_lEnd;              // point numbers in scan
    float m_fSigma2;           // 匹配误差

	bool  m_bSelected;         // 标明此线段是否被选中(用于屏幕编辑处理)

public:
	CLineFeature();

	virtual bool Load(FILE* file);
	virtual bool Save(FILE* file);

	// 生成线段
	void Create(CPnt& ptStart, CPnt& ptEnd);

	// 生成线段
	void Create(CLine& ln);
	
	// 反转线段的方向(将起点、终点对调)
	void Reverse();
	
	// 将该直线段投影到另一直线段，得到的投影线段存入lnResult
	bool ProjectToLine(CLineFeature& ln, CLineFeature& lnResult);


	// 将特征进行平移
	virtual void Move(float fX, float fY);

	// 将特征进行旋转
	virtual void Rotate(CAngle ang, CPnt ptCenter);

	// 选中/取消选中此线段
	void Select(bool bOn) {m_bSelected = bOn;}

#ifdef _MSC_VER
	// (在屏幕窗口上)测试指定的点是否在线段上
	//bool HitTest(CScreenReference& ScrnRef, CPoint point);

	// 在屏幕上显示此直线特征
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize);
#endif

};
#endif
