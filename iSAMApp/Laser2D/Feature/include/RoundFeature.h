#ifndef __CRoundFeature
#define __CRoundFeature

#include "PointFeature.h"

class CScan;

///////////////////////////////////////////////////////////////////////////////
//   定义“圆柱特征”类。
class CRoundFeature : public CPointFeature
{
private:
	float m_fRadius;                // 柱状特征的半径

public:
	CRoundFeature();

	// 生成一个复本
	virtual CPointFeature* Duplicate() const;

	// 设置柱状特征的参数
	void SetParam(float fRadius);

	// 针对给定的点云，在规定的角度范围内，检测点云中是否含有该柱状物，并返回它的中心位置
	virtual bool Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter);

	// 从文本文件中装入点特征的参数
	virtual int LoadText(FILE* fp);

	// 将点特征的参数保存到文本文件中
	virtual int SaveText(FILE* fp);

	// 从二进制文件中装入点特征的参数
	virtual int LoadBinary(FILE* fp);

	// 将点特征的参数保存到二进制文件中
	virtual int SaveBinary(FILE* fp);

#ifdef _MSC_VER
	// 在屏幕上绘制此点特征
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize);
#endif
};
#endif

