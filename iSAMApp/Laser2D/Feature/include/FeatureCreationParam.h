#pragma once

#include "ReflFeatureCreateParam.h"
#include "LineFeatureCreateParam.h"

///////////////////////////////////////////////////////////////////////////////
//   特征提取参数。
class CFeatureCreationParam
{
public:
	CReflectorCreationParam   m_RefParam;
	CLineFeatureCreationParam m_LineParam;

	CFeatureCreationParam() {}

	// 从二进制文件装入参数
	bool Load(FILE* fp)
	{
		if (!m_RefParam.Load(fp))
			return false;

		if (!m_LineParam.Load(fp))
			return false;
		
		return true;
	}

	// 将参数保存到二进制文件
	bool Save(FILE* fp)
	{
		if (!m_RefParam.Save(fp))
			return false;

		if (!m_LineParam.Save(fp))
			return false;

		return true;
	}
};