#include "stdafx.h"
#include <algorithm>
#include "PointFeatureSet.h"
#include "FlatReflectorFeature.h"
#include "ShortLineFeature.h"
#include "Scan.h"
#include "assert.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define NOISE_RATIO        0.02f

//
//   对给定的极径数据施加噪声，得到带有随机误差的极径数据。
//
void AddNoise(float fDistIn, float& fDistOut, float fNoiseRatio)
{
	float fNoise = fDistIn * fNoiseRatio;
	int nAmp = (int)(fNoise);
	int nNoise = (nAmp == 0) ? 0 : rand() % nAmp;
	nNoise -= nAmp / 2;
	fDistOut = fDistIn + nNoise;
}

///////////////////////////////////////////////////////////////////////////////

CPointFeatureSet::CPointFeatureSet()
{
	reserve(50);
	clear();
	m_rect.Clear();
	m_pDistCache = NULL;
}

CPointFeatureSet::~CPointFeatureSet()
{
	Clear();
}

//
//   “拷贝”构造函数。
//
CPointFeatureSet::CPointFeatureSet(const CPointFeatureSet& another)
{
	m_pDistCache = NULL;
	Clear();

	for (int i = 0; i < (int)another.size(); i++)
	{
		// 根据点特征的类型分配新空间
		CPointFeature* pNewFeature = another[i]->Duplicate();
		if (pNewFeature == NULL)
			assert(false);

		push_back(pNewFeature);
	}

	CreateDistanceCache();
}

//
//   实现赋值运算。
//
CPointFeatureSet& CPointFeatureSet::operator = (const CPointFeatureSet& another)
{
	Clear();

	for (int i = 0; i < (int)another.size(); i++)
	{
		// 根据点特征的类型分配新空间
		CPointFeature* pNewFeature = another[i]->Duplicate();
		if (pNewFeature == NULL)
			assert(false);

		push_back(pNewFeature);
	}

	CreateDistanceCache();
	return *this;
}

//
//   清除集合。
//
void CPointFeatureSet::Clear()
{
	ClearDistanceCache();

	for (int i = 0; i < (int)size(); i++)
		delete at(i);

	clear();
	m_rect.Clear();
}

//
//   根据所提供的特征类型分配空间。
//
CPointFeature* CPointFeatureSet::NewPointFeature(int nSubType)
{
	// 根据类型分配空间
	switch (nSubType)
	{
	case GENERIC_POINT_FEATURE:              // 一般点特征
		return new CPointFeature;

	case FLAT_REFLECTOR_FEATURE:             // 反光板特征
		return new CFlatReflectorFeature;

	case SHORT_LINE_FEATURE:                 // 短直线特征
		return new CShortLineFeature;

	default:
		return NULL;
	}
}

//
//   向集合内添加一个新点。
//
CPointFeatureSet& CPointFeatureSet::operator += (CPointFeature* pNewFeature)
{
	ClearDistanceCache();

	push_back(pNewFeature);
	m_rect += *pNewFeature;

	return *this;
}

//
//   向集合内添加一个新点。
//
CPointFeatureSet& CPointFeatureSet::operator += (const CPointFeature& NewFeature)
{
	*this += NewFeature.Duplicate();
	return *this;
}

//
//   将另一个集合并入本集合中。
//
CPointFeatureSet& CPointFeatureSet::operator += (const CPointFeatureSet& another)
{
	for (int i = 0; i < (int)another.size(); i++)
	{
		CPointFeature* pFeature = another[i]->Duplicate();
		*this += pFeature;
	}

	return *this;
}

//
//   删除指定序号的点。
//
bool CPointFeatureSet::DeleteAt(int nIdx)
{
	if (nIdx < 0 && nIdx >= (int)size())
		return false;

	ClearDistanceCache();

	delete at(nIdx);
	erase(begin() + nIdx);

	UpdateCoveringRect();

	return true;
}

//
//   根据所有点的极坐标计算出它们的迪卡尔坐标。
//
void CPointFeatureSet::UpdateCartisian()
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->UpdateCartisian();
}

//
//   更新边界值。
//
void CPointFeatureSet::UpdateCoveringRect()
{
	m_rect.Clear();

	for (int i = 0; i < (int)size(); i++)
		m_rect += *at(i);
}

//
//   将点特征集合由世界坐标系变换到局部坐标系。
//
void CPointFeatureSet::Transform(CPosture& pstLocal)
{
	// 建立坐标变换
	CTransform trans(pstLocal);

	// 进行点特征的坐标变换
	for (int i = 0; i < (int)size(); i++)
	{
		CPnt pt = trans.GetLocalPoint(at(i)->GetPntObject());
		at(i)->GetPntObject() = pt;
	}

	UpdateCoveringRect();
}

//
//   将点特征集合由局部坐标系变换到世界坐标系。
//
void CPointFeatureSet::InvTransform(CPosture& pstOrigin)
{
	// 建立坐标变换
	CTransform trans(pstOrigin);

	// 进行直线的坐标变换
	for (int i = 0; i < (int)size(); i++)
	{
		CPnt pt = trans.GetWorldPoint(at(i)->GetPntObject());
		at(i)->GetPntObject() = pt;
	}

	UpdateCoveringRect();
}

//
//   以给定的点为中心，以指定的范围为半径，截取出一个特征子集。
//
bool CPointFeatureSet::GetSubset(const CPnt& ptCenter, float fRange, CPointFeatureSet& Subset)
{
	Subset.Clear();

	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = at(i);
		if (pFeature->DistanceTo(ptCenter) < fRange)
			Subset += pFeature;
	}

	return true;
}

//
//   更改指定点的位置。
//
bool CPointFeatureSet::ModidfyPointPos(int nId, CPnt& pt)
{
	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = at(i);
		if (pFeature->m_nFeatureId == nId)
		{
			pFeature->GetPntObject() = pt;
			return true;
		}
	}

	ClearDistanceCache();
	return false;
}

//
//   通过在pstScanner处对给定的点集进行虚拟扫描采样，进而生成局部扫描点集。
//   返回值：
//     　生成的点的数量。
//
int CPointFeatureSet::CreateFromSimuScan(const CPointFeatureSet& Model, const CPosture& pstScanner,
	float fMaxScanDist, bool bAddNoise)
{
	// 先清除点集内原来的数据
	Clear();

	// 扫描原点位置
	float fMaxScanDist2 = fMaxScanDist * fMaxScanDist;

	for (int i = 0; i < (int)Model.size(); i++)
	{
		// 取得指向此特征的指针
		CPointFeature* pFeature = Model[i];

		// 构造当前扫描线
		CLine ln(pstScanner, *pFeature);

		// 判断此扫描线是否可以被反射
		if (!pFeature->CheckInRay(ln))
			continue;

		// 计算扫描线长度的平方值
		float fLen2 = ln.Length2();

		// 如线段长度超限，则忽略该线
		if (fLen2 > fMaxScanDist2)
			continue;

		float r = sqrt(fLen2);

		// 如果是仿真，在此对扫描得到的极径施加随机噪声
		if (bAddNoise)
			AddNoise(r, r, NOISE_RATIO);

		pFeature->r = r;                                                    // 极径
		pFeature->a = (ln.SlantAngle() - pstScanner.fThita).NormAngle();    // 极角

		int nType = Model[i]->GetSubType();

		// 为新的点特征分配空间
		CPointFeature* pNewFeature = pFeature->Duplicate();

		// 添加此点特征
		*this += pNewFeature;
	}

	// 对所有特征点按极角进行排序
	SortByPolarAngle();

	return GetCount();
}

//
//   从给定的扫描点云生成反光板特征。
//
bool CPointFeatureSet::CreateFromScan(const CScan& Scan, CReflectorCreationParam* pParam)
{
	CReflectorCreationParam Param;
	if (pParam != NULL)
	{
		Param = *pParam;
	}

	// 下面生成反光板点特征
	Clear();

	// 反光板尚未开始
	int nReflectorStart = -1;
	float r1 = 0;
	bool bStartAtRef = false;           // 标记是否第一线即为反光板
	int  nStartRefEndPos = -1;          // 第一线为反光板的情况下，反光板的结束位置

	int nCount = Scan.m_nCount;
	for (int i = 0; i < nCount; i++)
	{
		const CScanPoint& sp = Scan.m_pPoints[i];

		// 如果反光板尚未开始
		if (nReflectorStart < 0)
		{
			// 如果极径超限，继续考察下一点
			if (sp.r < 0)
				continue;
			else if (sp.m_nIntensity > Param.nMinReflectorIntensity)
			{
				nReflectorStart = i;     // 记录反光板开始处的序号
				r1 = sp.r;               // 记录此处的极径

												 // 判断是否第一线即为强反光线
				if (i == 0)
					bStartAtRef = true;
			}
		}

		// 如果反光板已开始，但下面满足反光板结束条件
		else if (sp.r < 0 || sp.m_nIntensity < Param.nMinReflectorIntensity)
		{
			// 反光板在此结束，需要验证反光板长度没有超限
			if (sp.DistanceTo(Scan.m_pPoints[nReflectorStart]) < Param.fMaxReflectorSize)
			{
				// 计算该反光板所占的点的数量
				int nCountPoints = i - nReflectorStart;

				// 如果这是起始的第一个反光板，记录反光板结束位置
				if (bStartAtRef && nStartRefEndPos < 0)
					nStartRefEndPos = i;

				// 在此计算反光板中心坐标
				int nCenterIdx = nReflectorStart + nCountPoints / 2;

				// 验证特征距离激光头的距离在规定的范围内

				if (Scan.m_pPoints[nCenterIdx].r / 1000.0f > 0.3f /*&&
					Scan.m_pPoints[nCenterIdx].r / 1000.0f > 30*/)
				{
					CFlatReflectorFeature* p = new CFlatReflectorFeature;
					p->GetPntObject() = Scan.m_pPoints[nCenterIdx];
					p->m_nIntensity = Scan.m_pPoints[nCenterIdx].m_nIntensity;
					p->m_nPointCount = nCountPoints;
					push_back(p);
				}
			}

			// 清除反光板起始标记
			nReflectorStart = -1;
			r1 = 0;
		}

		// 反光板继续中
		else
		{
			// 同一反光板内相邻点极径变化过大，则认为这是跨两个反光物的“假反光板”
			if (fabs(sp.r - r1) / 1000 > Param.fMaxPolarRadiusVariance)
			{
				nReflectorStart = -1;
				r1 = 0;
				i += 3;     // 跳过不稳定区
			}

			// 如果第一线即为反光板光线，而且本周最后一线也为反光板光线，现在需要修正第一个反光板的位置
			else if (bStartAtRef && (i == nCount - 1))
			{
				int nCountPoints = nStartRefEndPos + nCount - nReflectorStart;
				int nPos = (nReflectorStart + nCountPoints / 2) % nCount;
				at(0)->GetPntObject() = Scan.m_pPoints[nPos];
			}
			else
				r1 = sp.r;
		}
	}

	return true;
}

//
//   清除内部的各点之间的距离值。
//
void CPointFeatureSet::ClearDistanceCache()
{
	if (m_pDistCache != NULL)
	{
		for (int i = 0; i < GetCount(); i++)
		{
			if (m_pDistCache[i] != NULL)
			{
				delete[]m_pDistCache[i];
				m_pDistCache[i] = NULL;
			}
		}

		delete[]m_pDistCache;
		m_pDistCache = NULL;
	}
}

//
//   为各点之间的距离分配存储空间。
//
bool CPointFeatureSet::CreateDistanceCache()
{
	int nSize = (int)size();

	// 生成新数据之前要求原缓冲区已清除
	if (m_pDistCache != NULL)
		ClearDistanceCache();

	// 重新分配空间
	m_pDistCache = new float*[nSize];
	if (m_pDistCache == NULL)
		return false;

	for (int i = 0; i < nSize; i++)
	{
		m_pDistCache[i] = new float[nSize];
		if (m_pDistCache[i] == NULL)
			return false;
	}

	// 计算两两特征点之间的距离
	for (int m = 0; m < nSize - 1; m++)
		for (int n = m + 1; n < nSize; n++)
			m_pDistCache[n][m] = m_pDistCache[m][n] = at(m)->DistanceTo(*at(n));

	return true;
}

//
//   取得i, j两点之间的距离。
//
float CPointFeatureSet::PointDistance(int i, int j)
{
	if (m_pDistCache != NULL)
		return m_pDistCache[i][j];
	else
		return at(i)->DistanceTo(*at(j));
}


typedef CPointFeature* CPtrPointFeature;

//
//   用于极角排序的比较函数。
//
bool CompFunc(const CPtrPointFeature& f1, const CPtrPointFeature &f2)
{
	return (f1->a < f2->a);
}

//
//   对所有的点特征按极角从小到大的顺序进行排序。
//
void CPointFeatureSet::SortByPolarAngle()
{
	sort(begin(), end(), CompFunc);
}

//
//   对重叠点进行合并。
//
void CPointFeatureSet::MergeOverlappedPoints(float fDistGate)
{
	int nCount = (int)size();
	bool *pDelete = new bool[nCount];
	for (int i = 0; i < nCount; i++)
		pDelete[i] = false;

	// 依次比较两个点特征，看它们之间的距离是否比fDistGate近
	for (int i = 0; i < nCount - 1; i++)
	{
		if (pDelete[i])
			continue;

		CPointFeature* pFeature1 = (CPointFeature*)at(i);

		// 取第二个点
		for (int j = i + 1; j < nCount; j++)
		{
			if (pDelete[j])
				continue;

			CPointFeature* pFeature2 = (CPointFeature*)at(j);

			// 如果这两个特征之间的距离非常近，则将第一点调整为两点之间的中间位置，并删除第二点
			if (pFeature1->DistanceTo(*pFeature2) < fDistGate)
			{
				float x = (pFeature1->x + pFeature2->x) / 2;
				float y = (pFeature1->y + pFeature2->y) / 2;
				pFeature1->x = x;
				pFeature1->y = y;
				pDelete[j] = true;
			}
		}
	}

	for (int i = nCount - 1; i >= 0; i--)
	{
		if (pDelete[i])
		{
			delete at(i);
			erase(begin() + i);
		}
	}

	delete[]pDelete;

	ClearDistanceCache();

	// 为所有的点重新编号
	for (int i = 0; i < (int)size(); i++)
		at(i)->id = i;
}

//
//   从文件中装入特征集合数据。
//   返回值：
//     < 0 : 读取失败
//     >= 0: 读取的特征数量
//
int CPointFeatureSet::LoadText(FILE* fp)
{
	Clear();

	int nCount, nSubType;

	// 读取特征数量
	if (fscanf(fp, "%d\n", &nCount) != 1 || nCount < 0)
		return -1;

	for (int i = 0; i < nCount; i++)
	{
		// 读取本特征的类型
		fscanf(fp, "%d", &nSubType);

		CPointFeature* pFeature = NewPointFeature(nSubType);
		if (pFeature == NULL)
			return -1;                // 空间分配失败

		// 根据类型自动读入参数
		if (pFeature->LoadText(fp) < 0)
			return -1;

		// 附加点的序号
		pFeature->GetPntObject().id = i;

		// 将对象指针加入到表中
		push_back(pFeature);

		// 调整边界值
		m_rect += *pFeature;
	}

	CreateDistanceCache();
	return nCount;
}

//
//   将特征集合数据保存到文件中。
//
int CPointFeatureSet::SaveText(FILE* fp)
{
	// 写入特征数量
	int nCount = (int)size();
	fprintf(fp, "%d\n", nCount);

	for (int i = 0; i < nCount; i++)
	{
		if (at(i)->SaveText(fp) < 0)
			return -1;
	}

	return nCount;
}

//
//   从二进制文件中装入特征集合数据。
//   返回值：
//     < 0 : 读取失败
//     >= 0: 读取的特征数量
//
int CPointFeatureSet::LoadBinary(FILE* fp)
{
	Clear();

	int nCount;

	// 读取特征数量
	if (fread(&nCount, sizeof(int), 1, fp) != 1 || nCount < 0)
		return -1;

	for (int i = 0; i < nCount; i++)
	{
		// 读取本特征的类型
		int nSubType;
		if (fread(&nSubType, sizeof(int), 1, fp) != 1)
			return -1;

		CPointFeature* pFeature = NewPointFeature(nSubType);
		if (pFeature == NULL)
			return -1;

		// 根据类型自动读入参数
		if (pFeature->LoadBinary(fp) < 0)
			return -1;

		// 附加点的序号
		pFeature->GetPntObject().id = i;

		// 将对象指针加入到表中
		push_back(pFeature);

		// 调整边界值
		m_rect += *pFeature;
	}

	CreateDistanceCache();
	return nCount;
}

//
//   将特征集合数据保存到文件中。
//
int CPointFeatureSet::SaveBinary(FILE* fp)
{
	// 保存特征数量
	int nCount = (int)size();
	if (fwrite(&nCount, sizeof(int), 1, fp) != 1)
		return -1;

	for (int i = 0; i < nCount; i++)
	{
		if (at(i)->SaveBinary(fp) < 0)
			return -1;
	}

	return nCount;
}

//
//   从二进制文件中装入用户编辑数据。
//
bool CPointFeatureSet::LoadUserData(FILE* fp)
{
	int n;
	for (int i = 0; i < (int)size(); i++)
	{
		if (fread(&n, sizeof(int), 1, fp) != 1)
			return false;

		// 数据为0时特征使能
		at(i)->Enable(n == 0);
	}

	return true;
}

//
//   将用户编辑数据保存到二进制文件中。
//
bool CPointFeatureSet::SaveUserData(FILE* fp)
{
	int n;
	for (int i = 0; i < (int)size(); i++)
	{
		// 特征使能时置0
		n = !(at(i)->IsEnabled());
		if (fwrite(&n, sizeof(int), 1, fp) != 1)
			return false;
	}

	return true;
}

//
//   从另外一个PointFeatureSet复制其用户使能设置。
//
bool CPointFeatureSet::CopyUserData(const CPointFeatureSet& another)
{
	if (another.size() != size())
		return false;

	for (int i = 0; i < (int)size(); i++)
	{
		bool bEnabled = another.at(i)->IsEnabled();
		at(i)->Enable(bEnabled);
	}

	return true;
}

//
//   进行坐标正变换。
//
void CPointFeatureSet::Transform(const CFrame& frame)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->Transform(frame);

	UpdateCoveringRect();
}

//
//   进行坐标逆变换。
//
void CPointFeatureSet::InvTransform(const CFrame& frame)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->InvTransform(frame);

	UpdateCoveringRect();
}

//   判断指定的世界点是否触碰到本特征集合中的某个点特征。
//
int CPointFeatureSet::PointHit(const CPnt& pt, float fDistGate)
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i)->DistanceTo(pt) < fDistGate)
			return i;

	return -1;
}

#ifdef _MFC_VER

void CPointFeatureSet::Dump()
{
	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = (CPointFeature*)at(i);
		CPnt& pt = pFeature->GetPntObject();
		pt.Dump();
	}
}

//
//   在屏幕上绘制此点特征集合。
//
void CPointFeatureSet::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, COLORREF crSelected, 
	int nLineWidth, bool bShowId)
{
	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = (CPointFeature*)at(i);
		pFeature->Plot(ScrnRef, pDC, cr, crSelected, 5, nLineWidth);

		if (bShowId)
			pFeature->PlotId(ScrnRef, pDC, RGB(0, 0, 255));
	}
}

#elif defined QT_VERSION

void CPointFeatureSet::Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, QColor crSelected, 
	int nLineWidth, bool bShowId)
{
	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = (CPointFeature*)at(i);
		pFeature->Plot(ScrnRef, pPainter, cr, crSelected, 5, nLineWidth);

		if (bShowId)
			pFeature->PlotId(ScrnRef, pPainter, Qt::blue));
	}
}

#endif