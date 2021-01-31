#pragma once

#include "Geometry.h"

//////////////////////////////////////////////////////////////////////////////
//   定义二维的坐标系类型"CFrame"。

class CFrame : public CPosture
{
public:
	CFrame(const CPosture& pstLocal);
	CFrame(const CPnt& ptOrigin, const CAngle& angSlant);
	CFrame(float _x, float _y, float _t);
	CFrame() {}

	// 后续生成坐标系参数
	void Create(const CPosture& pstLocal);

	// 后续生成坐标系参数
	void Create(float _x, float _y, float _t)
	{
		CPosture::Create(_x, _y, _t);
	}

	//   定义“*=”运算
	CFrame& operator *= (const CFrame& another);

	//   定义“*”运算
	CFrame operator * (const CFrame& another);
};
