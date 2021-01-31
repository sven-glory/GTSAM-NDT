#ifndef __CPointFeatureSet
#define __CPointFeatureSet

#include <stdio.h>
#include <vector>
#include "PointFeature.h"
#include "FeatureCreationParam.h"

#ifdef QT_VERSION
#include <QColor>
#endif

using namespace std;

class CPointMatchList;
class CScan;

//////////////////////////////////////////////////////////////////////////////
//   定义“点特征集合”类。
class CPointFeatureSet : public vector <CPointFeature*>
{
private:
	CRectangle m_rect;
	float** m_pDistCache;     // 用于存储点与点之间距离的数据缓冲区

private:
	// 根据所提供的特征类型分配空间
	CPointFeature* NewPointFeature(int nSubType);

	// 更新边界值
	void UpdateCoveringRect();

	// 清除内部的各点之间的距离值
	void ClearDistanceCache();

public:
	CPointFeatureSet();
	~CPointFeatureSet();

	// “拷贝”构造函数
	CPointFeatureSet(const CPointFeatureSet& another);

	// 赋值
	CPointFeatureSet& operator = (const CPointFeatureSet& another);

	// 清除集合
	virtual void Clear();

	// 取得集合内点的数量
	int GetCount() { return (int)size(); }

	// 根据所有点的极坐标计算出它们的迪卡尔坐标
	void UpdateCartisian();

	// 向集合内添加一个新点
	CPointFeatureSet& operator += (CPointFeature* pNewFeature);

	// 向集合内添加一个新点
	CPointFeatureSet& operator += (const CPointFeature& NewFeature);

	// 将另一个集合并入本集合中
	CPointFeatureSet& operator += (const CPointFeatureSet& another);

	// 删除指定序号的点
	bool DeleteAt(int nIdx);

	// 将点特征集合由世界坐标系变换到局部坐标系
	void Transform(CPosture& pstLocal);

	// 将点特征集合由局部坐标系变换到世界坐标系
	void InvTransform(CPosture& pstOrigin);

	// 以给定的点为中心，以指定的范围为半径，截取出一个特征子集
	bool GetSubset(const CPnt& ptCenter, float fRange, CPointFeatureSet& Subset);

	// 更改指定点的位置
	bool ModidfyPointPos(int nId, CPnt& pt);

	// 通过在pstScanner处对给定的点集进行虚拟扫描采样，进而生成局部扫描点集
	int CreateFromSimuScan(const CPointFeatureSet& Model, const CPosture& pstScanner, 
		float fMaxScanDist, bool bAddNoise = false);

	// 从给定的扫描点云生成反光板特征
	bool CreateFromScan(const CScan& Scan, CReflectorCreationParam* pParam);

	// 取得覆盖区域
	CRectangle GetCoveringRect() const { return m_rect; }

	// 为各点之间的距离分配存储空间
	bool CreateDistanceCache();

	// 取得i, j两点之间的距离
	float PointDistance(int i, int j);

	// 对所有的点特征按极角从小到大的顺序进行排序
	void SortByPolarAngle();

	// 对重叠点进行合并
	void MergeOverlappedPoints(float fDistGate = 0.2f);

	// 从文件中装入特征集合数据
	virtual int LoadText(FILE* fp);

	// 将特征集合数据保存到文件中
	virtual int SaveText(FILE* fp);

	// 从二进制文件中装入特征集合数据
	virtual int LoadBinary(FILE* fp);

	// 将特征集合数据保存到二进制文件中
	virtual int SaveBinary(FILE* fp);

	// 从二进制文件中装入用户编辑数据
	bool LoadUserData(FILE* fp);

	// 将用户编辑数据保存到二进制文件中
	bool SaveUserData(FILE* fp);

	// 从另外一个PointFeatureSet复制其用户使能设置
	bool CopyUserData(const CPointFeatureSet& anther);

	// 进行坐标正变换
	virtual void Transform(const CFrame& frame);

	// 进行坐标逆变换
	virtual void InvTransform(const CFrame& frame);

	// 判断指定的世界点是否触碰到本特征集合中的某个点特征。
	int PointHit(const CPnt& pt, float fDistGate);

#ifdef _MFC_VER
	void Dump();

	// 在屏幕上绘制此点特征集合
	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, COLORREF crSelected, int nLineWidth = 0, 
		bool bShowId = false);

#elif defined QT_VERSION
	// 在屏幕上绘制此点特征集合
	void Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, QColor crSelected, , int nLineWidth = 0,
		bool bShowId = false);
#endif
};
#endif

