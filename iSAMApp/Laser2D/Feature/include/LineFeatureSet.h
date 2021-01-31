#ifndef __CLineFeatureSet
#define __CLineFeatureSet

#include "ScanPoint.h"
#include "LineFeature.h"
#include "PointFeatureSet.h"
#include "LineFeatureCreateParam.h"

#include <vector>

using namespace std;

class CScan;

//
//   CLineFeatureSet描述扫描中的直线段。
//
class CLineFeatureSet : public vector<CLineFeature*>
{
private:
	CRectangle m_rect;

private:
	void UpdateCoveringRect();

	void SplitPoints(CScanPoint *sp, long lStart, long lEnd);

	// 将那些共线且相连的线段合并
	void LineScanMergeLines(CScan *scan, long *lineNum);

	// 删除那些扫描角不佳的直线特征
	void RemoveBadLines(const CPosture& pstScanner, const CScan& scan);

public:
	CLineFeatureCreationParam m_Param;    // 直线生成参数
	CPosture      m_pstScanner;           // 激光头参考姿态

private:
	bool PointTooCloseToSomeCorner(CPnt& pt, vector<CPnt>& ptCorners);

	// 对所有特征进行共线调理
	bool ColinearRectify();

	// 根据直线特征类型分配空间
	CLineFeature* NewLineFeature(int nSubType);

public:
	// 构造函数
	CLineFeatureSet(int nNum = 0);

	// “拷贝”构造函数
	CLineFeatureSet(const CLineFeatureSet& Obj, bool bFilterDisabled = false);

	~CLineFeatureSet();

	// 重载“=”操作符
	void operator = (const CLineFeatureSet& Obj);

	// 设置直线性特征生成参数
	void SetCreationParam(CLineFeatureCreationParam* pParam);

	// 根据所提供的扫描到的直线数组生成直线特征集合
	bool CreateFromLocalLines(int nNum, CLineFeature* pLineData);

	// 设置扫检测到这些直线特征时的激光头姿态，以便计算各条直线特征的观测方向
	void SetDetectPosture(const CPosture& pstDetect);

	// 从一个扫描集中抽取其有所有直线段
	bool CreateFromScan(const CScan& scan);

	// 根据当前姿态、最大扫描半径和直线模型来生成直线特征集合
	bool CreateFromWorldLines(CPosture& pst, float fMaxRange, int nNumOfLines, CLine* pLine);

	// 清除所有的直线扫描(CLineFeatureSet)
	void Clear();

	// 取得直线特征的数量
	int GetCount() const {return (int)size();}

	// 取得指定的直线特征
	CLineFeature& GetLineFeature(int nIdx) { return *at(nIdx); }

	// 分配内存并复制当前对象内容
	CLineFeatureSet *Duplicate();

	// 将此另一个直线段集并入此直线段集中
	bool Merge(const CLineFeatureSet& LineScan);

	// 增加一个直线特征
	bool Add(const CLineFeature& LineFeature);

	// 通过合并共线的线段来简化此直线段集合
	bool Simplify(float fMaxGapBetweenLines);

	// 在集合中找到所有共线的特征，并进行分组记录
	int FindColinearGroups(float fMaxAngDiff, float fMaxDistDiff);

	// 针对共线的特征，通过多段线的描述方式进行合并
	bool ColinearSimplify(float fMaxAngDiff, float fMaxDistDiff);

	// 生成优化共线特征后的直线特征组
	CLineFeatureSet* Optimize();

	// 删除指定的线段
	bool DeleteAt(int nIdx);

	// returns field of view of linescan in rad.
	float FieldOfView(CScan *scan);

	// returns total length of all line segments
	float TotalLength();

	// 去掉所有长度短于minLineLength的直线
	void LengthFilter(float minLineLength);

	// 判断直线扫描集是否包含指定的点
	bool ContainScanPoint(const CScanPoint& sp);

	// 移除位于指定区域内的线段
	void RemoveWithin(const CRectangle& r);

	// 将特征进行平移
	virtual void Move(float fX, float fY);

	// 将特征进行旋转
	virtual void Rotate(CAngle ang, CPnt ptCenter);

#if 0
	// 变换到局部坐标系
	void TransformToLocal(CTransform& trans, CLineFeatureSet& setLocal);

	// 变换到全局坐标系
	void TransformToGlobal(CTransform& trans, CLineFeatureSet& setGlobal);
#endif

	void Select(int nIdx, bool bOn);

	// 从文本文件装入直线特征集合
	virtual int LoadText(FILE* fp);

	// 将直线特征集合存到文本文件
	virtual int SaveText(FILE* fp);

	// 从二进制文件装入直线特征集合
	virtual int LoadBinary(FILE* fp);

	// 将直线特征集合存到二进制文件
	virtual int SaveBinary(FILE* fp);

	// 取得最左点X坐标
	float LeftMost() { return m_rect.Left(); }

	// 取得最右点X坐标
	float RightMost() { return m_rect.Right(); }

	// 取得最上点Y坐标
	float TopMost() { return m_rect.Top(); }

	// 取得最下点Y坐标
	float BottomMost() { return m_rect.Bottom(); }

	// 取得整个直线集合的外阔尺寸
	// 取得覆盖区域
	CRectangle GetCoveringRect() const { return m_rect; }

	// 根据直线特征集合生成角点集合
	int CreateCornerPoints(vector<CPointFeature>& ptCorners);

	// 将直线集合转换到局部坐标系中，转换后的原点姿态落到指定的姿态处
	void Transform(CPosture& pstLocal);

	// 将直线集合转换到世界坐标系中，转换后原来的原点姿态需要到达指定的姿态
	void InvTransform(CPosture& pstOrigin);

	// 以给定的点为中心，以指定的范围为半径，截取出一个特征子集
	bool GetSubset(CPnt& ptCenter, float fRange, CLineFeatureSet& Subset);

	// 针对当前的直线特征集合，分离出“较好的直线特征集合”和“角点特征集合”
	bool SeperateFeatures(CLineFeatureSet& GoodLineFeatureSet, CPointFeatureSet& CornerFeatureSet);

	// 从二进制文件中装入用户编辑数据
	bool LoadUserData(FILE* fp);

	// 将用户编辑数据保存到二进制文件中
	bool SaveUserData(FILE* fp);

	// 从另外一个LineFeatureSet复制其用户使能设置
	bool CopyUserData(const CLineFeatureSet& another);

	// 进行坐标正变换
	virtual void Transform(const CFrame& frame);

	// 进行坐标逆变换
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER
	void Dump();

	// 绘制直线特征集合
	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nPointSize = 1,
		bool bShowActiveSide = false, bool bShowId = false, bool bShowRefPoint = false, 
		int nShowDisabled = DISABLED_FEATURE_UNDERTONE);

	// 判断一个给定点是否触碰到某一条直线特征(用于屏幕上直线触碰判断)
	int PointHit(const CPnt& pt, float fDistGate);
#endif
};
#endif
