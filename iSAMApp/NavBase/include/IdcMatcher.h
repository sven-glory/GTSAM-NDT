#ifndef __CIdcMatcher
#define __CIdcMatcher

#include "matrix.h"

//
//   基于IDC算法的匹配器。
//
class CIdcMatcher
{
private:
	CScan*         m_pRefScan;              // 指向“参考扫描”的指针
	CScan*         m_pCurScan;              // 指向“当前扫描”的指针

public:
	// 构造函数
	CIdcMatcher();

	// 析构函数
	~CIdcMatcher();

	// 进行匹配运算
	int Match(CScan* pRefScan, CScan* pCurScan, PoseGauss* pPose, float* pError);
};
#endif
