#include "stdafx.h"
#include <string.h>
#include "ScanPointCloud.h"
#include "Frame.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
//   “CScanPointCloud”类的实现。

//
//   根据指定的点数量生成对象(只分配空间)。
//
CScanPointCloud::CScanPointCloud(int nNum)
{
	m_nCount = nNum;
	m_pPoints = new CScanPoint[nNum];

	ASSERT(m_pPoints != NULL);
}

//
//   根据另外一个点云生成对象(复制全部的扫描点)。
//
CScanPointCloud::CScanPointCloud(const CScanPointCloud& Cloud)
{
	m_nCount = 0;
	m_pPoints = NULL;

	*this = Cloud;
}

//
//   生成空的对象。
//
CScanPointCloud::CScanPointCloud()
{
	m_nCount = 0;
	m_pPoints = NULL;
}

//
//   析构函数。
//
CScanPointCloud::~CScanPointCloud()
{
	Clear();
}

//
//   清除所有原来的数据。
//
void CScanPointCloud::Clear()
{
	if (m_pPoints != NULL)
		delete []m_pPoints;

	m_nCount = 0;
	m_pPoints = NULL;
}

//
//   为点云数据分配空间。
//
bool CScanPointCloud::Create(int nNum)
{
	m_pPoints = new CScanPoint[nNum];

	if (m_pPoints == NULL)
	{
		m_nCount = 0;
		return false;
	}
	else
	{
		m_nCount = nNum;
		return true;
	}
}

//
//   向点集末尾添加一个点。
//   注意：此操作将为点云重新分配空间，原有的空间将被释放。
//
bool CScanPointCloud::Add(const CScanPoint& sp)
{
	CScanPoint* pNewBuf = new CScanPoint[m_nCount+1];
	if (pNewBuf == NULL)
		return false;

	// 复制原有数据
	memmove(pNewBuf, m_pPoints, sizeof(CScanPoint) * m_nCount);

	// 释放原来的点云空间
	delete []m_pPoints;
	
	// 指向新的缓冲区
	m_pPoints = pNewBuf;

	// 添加新点
	m_pPoints[m_nCount++] = sp;

	return true;
}

//
//   从点云中删除一个点。
//   注意：删除后点云空间中有多余未用的空间。
//
bool CScanPointCloud::Delete(int nIndex)
{
	if (nIndex < 0 || nIndex >= m_nCount)
		return false;

	for (int i = nIndex; i < m_nCount - 1; i++)
		m_pPoints[i] = m_pPoints[i + 1];

	// 更改点云中点的数量(点云空间中可能有多余未用的空间)
	m_nCount--;

	return true;
}

//
//   重载“=”操作符。
//
void CScanPointCloud::operator = (const CScanPointCloud& Cloud2)
{
	if (m_nCount != Cloud2.m_nCount)
	{
		Clear();
		VERIFY(Create(Cloud2.m_nCount));
	}

	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i] = Cloud2.m_pPoints[i];

	Stamp(Cloud2.m_uTime);
}

//
//   重载“+=”操作符。
//
void CScanPointCloud::operator += (const CScanPointCloud& Cloud2)
{
	// 先复制原来的点云。
	CScanPointCloud CopyThis(*this);

	// 清除原点云数据
	Clear();

	// 重新分配空间以便容纳两个点云的数据
	VERIFY(Create(CopyThis.m_nCount + Cloud2.m_nCount));

	int i;

	// 复制回原有数据
	for (i = 0; i < CopyThis.m_nCount; i++)
		m_pPoints[i] = CopyThis.m_pPoints[i];

	// 再增加入新点云数据
	for (i = 0; i < Cloud2.m_nCount; i++)
		m_pPoints[CopyThis.m_nCount + i] = Cloud2.m_pPoints[i];
}

//
//   从文本文件中装入点云数据。
//
bool CScanPointCloud::Load(FILE* file)
{
	int nCount;
	fscanf(file, "%d\n", &nCount);

	Clear();
	if (!Create(nCount))
		return false;

	for (int i = 0; i < nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];
		fscanf(file, "%f\t%f\n", &(sp.x), &(sp.y));
		sp.id = i;
	}

	return true;
}

//
//   将点云数据存入文本文件。
//
bool CScanPointCloud::Save(FILE* file)
{
	fprintf(file, "%ld\n", m_nCount);
	for (int i = 0; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];
		fprintf(file, "%.6f\t%.6f\n", sp.x, sp.y);
	}

	return true;
}

//
//   从二进制文件中读取扫描数据。
//
bool CScanPointCloud::LoadBinary(FILE* fp, float fStartAngle, float fEndAngle, int nLineCount, 
	int nFileVersion)
{
	// 计算扫描的角分辨率
	float fAngReso = (fEndAngle - fStartAngle) / nLineCount;
	
	Clear();
	if (!Create(nLineCount))
		return false;

	// 如果版本在V2.10以上，在此需要读入时间戳
	unsigned int uTimeStamp;
	if (nFileVersion >= 210)
	{
		if (fread(&uTimeStamp, sizeof(unsigned int), 1, fp) != 1)
			return false;

		Stamp(uTimeStamp);
	}

	// 依次读入所有点的数据
	for (int i = 0; i < m_nCount; i++)
	{
		float r;
		int nIntensity;

		// 如果文件格式版本不超过2.00版(极径+强度共需8个字节)
		if (nFileVersion < 200)
		{
			// 读入极径
			if (fread(&r, sizeof(float), 1, fp) != 1)
				return false;

			// 读入反光强度
			if (fread(&nIntensity, sizeof(int), 1, fp) != 1)
				return false;
		}

		// 如果文件格式在2.00版以上(极径+强度共需3个字节)
		else
		{
			unsigned short int ur;
			unsigned char ui;
			if (fread(&ur, sizeof(unsigned short int), 1, fp) != 1)
				return false;

			if (fread(&ui, sizeof(unsigned char), 1, fp) != 1)
				return false;

			r = ur * 2.0f;                       // 极径分辨率为2mm
			nIntensity = (int)ui * 10;                // 强度范围(0-255)
		}

		if (r < 0 || r > 65500)    // r > 65500为临时措施，解决扫描线出现一圈半径为65534的弧的问题
			r = 0;

		CScanPoint& sp = m_pPoints[i];

		sp.r = r;
		sp.a = fStartAngle + fAngReso * i;
		sp.m_nIntensity = nIntensity;
		
		// 极径大于0，表示数据有效
		if (r >= 0)
		{
//			sp.r /= 1000;          // 目前文件格式以mm为单位，需要转换为m
			// 计算迪卡尔坐标
			sp.UpdateCartisian();

			sp.x /= 1000;          // 目前文件格式以mm为单位，需要转换为m
			sp.y /= 1000;
		}
#if 0
		// 处理无效点
		else
			sp.x = sp.y = 0;
#endif

		sp.id = i;
	}

	return true;
}

//
//   将扫描数据保存到十进制文件。
//
bool CScanPointCloud::SaveBinary(FILE* fp, int nFileVersion)
{
	for (int i = 0; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];

		// V2.00版本格式以下
		if (nFileVersion < 200)
		{
			fwrite(&sp.r, sizeof(float), 1, fp);
			fwrite(&sp.m_nIntensity, sizeof(int), 1, fp);
		}

		// V2.00以上版本格式
		else
		{
			unsigned short int ur = sp.r / 2;                   // 极径用双字节表示，分辨率为2mm
			fwrite(&ur, sizeof(unsigned short int), 1, fp);

			unsigned char ui = (unsigned char)(sp.m_nIntensity / 10);  // 强度用单字节表示
			fwrite(&ui, sizeof(unsigned char), 1, fp);
		}
	}

	return true;
}

//
//   根据各点的迪卡尔坐标计算出所有点的极坐标。
//
void CScanPointCloud::UpdatePolar()
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].UpdatePolar();
}

//
//   根据极坐标计算出所有点的迪卡尔坐标。
//
void CScanPointCloud::UpdateCartisian()
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].UpdateCartisian();
}

//
//   将全部点移动指定的距离。
//
void CScanPointCloud::Move(float fDx, float fDy)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Move(fDx, fDy);
}

//
//   将全部点绕指定的中心点进行旋转。
//
void CScanPointCloud::Rotate(float centerX, float centerY, float angle)
{
	if (angle != 0.0)
	{
		float sina = sin(angle);
		float cosa = cos(angle);

		CScanPoint *sp = m_pPoints;

		float x, y;
		for (int i = 0; i < m_nCount; i++, sp++)
		{
			x = sp->x - centerX;
			y = sp->y - centerY;
			sp->x = centerX + cosa * x - sina * y;
			sp->y = centerY + sina * x + cosa * y;
		}
	}
}
#if 1
//
//   将整个点集进行坐标变换。
//
void CScanPointCloud::Transform(float x, float y, float thita)
{
	CPnt pt(x, y);
	CTransform trans(pt, CAngle(thita));

	for (int i = 0; i < m_nCount; i++)
	{
		CPnt pt = trans.GetWorldPoint(m_pPoints[i]);
		m_pPoints[i].x = pt.x;
		m_pPoints[i].y = pt.y;
	}
}

//
//   将整个点云进行坐标变换(第二种形式)。
//
void CScanPointCloud::Transform(const CPosture& pstOrigin)
{
	Transform(pstOrigin.x, pstOrigin.y, pstOrigin.fThita);
}
#endif

//
//   对整个点云进行坐标系变换。
//
void CScanPointCloud::Transform(const CFrame& frame)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Transform(frame);
}

//
//   对整个点云进行坐标系逆变换。
//
void CScanPointCloud::InvTransform(const CFrame& frame)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].InvTransform(frame);
}


//
//   取得最左点X坐标。
//
float CScanPointCloud::LeftMost()
{
	if (m_nCount == 0)
		return -HUGE_VAL;

	float fMost = m_pPoints[0].x;
	for (USHORT i = 0; i < m_nCount; i++)
	{
		if (fMost > m_pPoints[i].x)
			fMost = m_pPoints[i].x;
	}

	return fMost;
}

//
//   取得最上点Y坐标。
//
float CScanPointCloud::TopMost()
{
	if (m_nCount == 0)
		return HUGE_VAL;

	float fMost = m_pPoints[0].y;
	for (USHORT i = 0; i < m_nCount; i++)
	{
		if (fMost < m_pPoints[i].y)
			fMost = m_pPoints[i].y;
	}

	return fMost;
}

//
//   取得最右点X坐标。
//
float CScanPointCloud::RightMost()
{
	if (m_nCount == 0)
		return HUGE_VAL;

	float fMost = m_pPoints[0].x;
	for (USHORT i = 0; i < m_nCount; i++)
	{
		if (fMost < m_pPoints[i].x)
			fMost = m_pPoints[i].x;
	}

	return fMost;
}

//
//   取得最下点Y坐标。
//
float CScanPointCloud::BottomMost()
{
	if (m_nCount == 0)
		return -HUGE_VAL;

	float fMost = m_pPoints[0].y;
	for (USHORT i = 0; i < m_nCount; i++)
	{
		if (fMost > m_pPoints[i].y)
			fMost = m_pPoints[i].y;
	}

	return fMost;
}

//
//   取得点云的X向宽度。
//
float CScanPointCloud::Width()
{
	return RightMost() - LeftMost();
}

//
//   取得点云的Y向高度。
//
float CScanPointCloud::Height()
{
	return TopMost() - BottomMost();
}

//
//   取得此点集所覆盖的最大区域。
//
CRectangle CScanPointCloud::GetCoveringRect()
{
	CRectangle r;
	for (int i = 0; i < m_nCount; i++)
		r += m_pPoints[i];

	return r;
}

//
//   核对点云是否包含指定的点。
//   返回值：
//     -1: 未找到
//     0~n: 找到的点的序号
//
int CScanPointCloud::ContainPoint(const CPnt& pt, float fThreshHoldDist)
{
	for (int i = 0; i < m_nCount; i++)
		if (m_pPoints[i].DistanceTo(pt) <= fThreshHoldDist)
			return i;

	return -1;
}

//
//   以指定的点的为中心，按指定的半径对所有点进行过滤，只留下处于半径以内的点。
//   注意：
//      1. 经此处理后，所有有效的点都已具有正确的极径值
//
void CScanPointCloud::ReduceByRadius(const CPnt& ptCenter, float dRadius)
{
	int nNewCount = 0;                     // 点云点的总数

	for (int i = 0; i < m_nCount; i++)
	{
		// 计算该点到中心点的距离
		float fDist = m_pPoints[i].DistanceTo(ptCenter);

		// 如果距离小于指定半径，则将该点留在新点云中
		if (fDist < dRadius)
		{
			// 在此附加上极径值
			m_pPoints[i].r = fDist;

			if (nNewCount != i)
				m_pPoints[nNewCount] = m_pPoints[i];

			nNewCount++;
		}
	}

	// 设定新点云中点的数量(点云空间中可能有多余未用的空间)
	m_nCount = nNewCount;

	// 删除所有标明为“m_bDelete”的点 (为何执行这个操作???)
	RemoveDeletedPoints();
}

//
//   以指定的方向角为中心，指定的角度范围对所有点进行过滤，只留下有效角度内的点。
//   注意：
//      1. 经此处理后，所有有效的点都已具有正确的极角值
//      2. 如此处理后，点云缓冲区内会有标明为“m_bDelete”但尚未删除的点
//
void CScanPointCloud::ReduceByAngle(CPosture& pst, float fViewAngle)
{
	CPnt& ptCenter = pst.GetPntObject();

	for (int i = 0; i < m_nCount; i++)
	{
		// 构造扫描线
		CScanPoint& sp = m_pPoints[i];

		// 跳过那些标明为“待删除”的点
		if (sp.m_bDelete)
			continue;

		CLine ln(ptCenter, sp.GetPntObject());

		// 记录扫描点的扫描角
		sp.a = (ln.SlantAngle() - pst.GetAngle()).NormAngle2();

		// 如果该点与中心点所成夹角不在激光扫描器可能的夹角范围内，则将该点从点云中剔除
		CAngle angDiff(sp.a);
		if (fabs(angDiff.NormAngle2()) > fViewAngle / 2)
		{
			sp.m_bDelete = true;
		}
	}
}

//
//   判断两个扫描点哪一个扫描角更大。
//
//   -1: 第二个点扫描角更大
//    1: 第一个点扫描角更大
//    0: 两者一样
//
int AngleSort(const void *x1, const void *x2)
{
	CScanPoint *sp1 = (CScanPoint*)x1;
	CScanPoint *sp2 = (CScanPoint*)x2;

	float d = sp1->a - sp2->a;

	if (d < 0.0)
		return -1;
	else if (d > 0.0)
		return 1;

	return 0;
}

//
//   依据指定的姿态，对点云根据扫描角进行排序。
//
void CScanPointCloud::SortByAngle()
{
	qsort(m_pPoints, m_nCount, sizeof(CScanPoint), AngleSort);
}

//
//   针对已按角度排序的点云，删除那些被遮挡的点。
//
void CScanPointCloud::RemoveHiddenPoints(const CPosture& pst, float fDistGate)
{
	bool bNewSeg = true;

	int j;
	for (int i = 0; i < m_nCount - 1; i++)
	{
		CScanPoint* pt1 = &(m_pPoints[i]);

		// 跳过那些已被标为“删除”的点
		if (pt1->m_bDelete)
			continue;

		// 计算超限角
		float fMaxAngGap = atan2(fDistGate, pt1->r);

		// 准备记录pt1点后的第一个相邻点的序号
		int nNeighbor = 0;

		// 找i点的下一个有效相邻点
		for (j = i + 1; j < m_nCount; j++)
		{
			// 命名当前的j点为pt2
			CScanPoint& pt2 = m_pPoints[j];

			if (pt2.m_bDelete)
				continue;

			// 如果p2的极角与pt1的极角之差超限，则它们之间的距离必定超限
			if (pt2.a - pt1->a > fMaxAngGap)
				break;

			// 计算p1点到p2点的距离，如果小于设定的门限，则视为p1与p2是相连的
			if (pt1->DistanceTo(pt2) < fDistGate)
			{
				nNeighbor = j;
				break;
			}
		}

		// 若无有效的相邻点，pt1后移
		if (nNeighbor == 0)
		{
			// 如果当前点是新段的第一点，则说明它是个孤点(不与任何其它点相邻)
			if (bNewSeg)
				pt1->m_bDelete = true;

			// 下面的点将开始一个新段
			bNewSeg = true;

			continue;
		}

		// p1后有邻接点，它不是孤点
		bNewSeg = false;

		// 令pt2为找到的与pt1最近的相邻点
		CScanPoint& pt2 = m_pPoints[nNeighbor];

		// 现在需要考核pt1与pt2之间的点
		for (j = i + 1; j < nNeighbor; j++)
		{
			CScanPoint& pt3 = m_pPoints[j];

			if (pt3.m_bDelete)
				continue;

			// 如果pt1与pt2之间的点极径大于这两个点的极径，则应删除之
			if (pt3.r > pt1->r && pt3.r > pt2.r)
			{
				pt3.m_bDelete = true;
			}
		}
	}

	RemoveDeletedPoints();
}

//
//   将所有超出距离限的点移除。
//
void CScanPointCloud::RemoveOutOfRangePoints()
{
	for (int i = 0; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];
		if (sp.r == SCAN_OUT_OF_RANGE)
		{
			sp.m_bDelete = true;
		}
	}

	RemoveDeletedPoints();
}

#define RESAMPLE_MIN_DIST	    500

//
//    按指定的角分辨率和扫描距离对点云进行重新采样。
//    注意：原来的点云必须是已经按角度排序过的。
//
CScanPointCloud* CScanPointCloud::ReSample(CPosture& pstScanner, float fStartAng, float fViewAngle, float fAngReso, float fMaxRange)
{
	bool bFinish = false;

	// 计算新点云应含的点数(结果4舍5入)
	int nCount = (int)(fViewAngle / fAngReso + 0.5f);

	// 为新的点云分配空间
	CScanPointCloud* pNewCloud = new CScanPointCloud(nCount);

	int i = 0, j;
	for (j = 0; j < nCount; j++)
	{
		CScanPoint& spNew = pNewCloud->m_pPoints[j];

		// 如果原点云中的所有点都已查看过，那么新点云中后面的点都应置为SCAN_OUT_OF_RANGE
		if (bFinish)
		{
			spNew.SetOutOfRange();
			continue;
		}

		// 先确定对应于该扫描线的扫描角
		float fScanAng = fStartAng + j * fAngReso;

		// 转换到相对于激光头姿态的相对角
		CAngle angScan(fScanAng);
		angScan -= pstScanner.GetAngle();

		// 先在原点云中找到对应于起始角度的点
		while (m_pPoints[i].a < angScan.NormAngle2())
		{
			i++;

			// 看一看是否原点云中的所有点都已查看过
			if (i > m_nCount)
			{
				bFinish = true;
				break;
			}
		}

		if (bFinish)
			continue;

		// 现在原点云中的起始点在i-1和i之间
		// 如果新扫描线在原点云集中相邻的两条扫描线中有超出范围的，则新线也视为超出范围
		if (m_pPoints[i - 1].r > fMaxRange || m_pPoints[i].r > fMaxRange)
		{
			spNew.SetOutOfRange();
		}
		// 否则，新线采用原点云集中相邻线的差补值
		else
		{
			// 构造新的扫描线
			CAngle ang(fScanAng);
			CLine ln1(pstScanner.GetPntObject(), ang, fMaxRange);

			// 先构造在原点云集中相邻的两个扫描点的连线
			CPnt& pt1 = m_pPoints[i - 1];
			CPnt& pt2 = m_pPoints[i];
			CLine ln2(pt1, pt2);

			// 计算上面ln1, ln2两条线的交点
			CPnt ptIntersect;
			float r;
			if (ln1.IntersectLineAt(ln2, ptIntersect, r))
			{
				if (fabs(r - pt1.r) > RESAMPLE_MIN_DIST || fabs(r - pt2.r) > RESAMPLE_MIN_DIST)
				{
					spNew.SetOutOfRange();
				}
				else
				{
					spNew.GetPntObject() = ptIntersect;
					spNew.a = angScan.NormAngle2();
					spNew.r = r;
					spNew.m_bDelete = false;
				}
			}
			else
			{
				spNew.SetOutOfRange();
			}
		}
	}

	return pNewCloud;
}

//
//   在此将那些标记为“删除”的点真正删除。
//
void CScanPointCloud::RemoveDeletedPoints()
{
	int k = 0;
	for (int i = 0; i < m_nCount; i++)
	{
		if (!m_pPoints[i].m_bDelete)
			m_pPoints[k++] = m_pPoints[i];
	}

	// 最后，调整点云中点的数量
	m_nCount = k;
}


//////////////////////////////////////////////////////////////////////////////
//   根据给定的模型仿真产生点云数据。

void AddNoise(CPnt& ptIn, float fNoise, CPnt& ptOut)
{
	int nAng = rand() % 360;
	CAngle ang(nAng*PI/180);
	
	int nRadius = (int)(fNoise);
	if (nRadius == 0)
		nRadius = 1;
	nRadius = rand() % nRadius;
	if (nRadius == 0)
		nRadius = 1;

	float fRadius = nRadius;

	CLine ln(ptIn, ang, fRadius);
	ptOut = ln.m_ptEnd;
}

//
//   根据给定的直线仿真生成点云.
//
bool CScanPointCloud::CreateFromLine(CLine& ln, float fGap, float fNoise)
{
	// 清除原有点云
	Clear();

	m_nCount = (int)(ln.Length() / fGap);
	m_pPoints = new CScanPoint[m_nCount];
	if (m_pPoints == NULL)
		return false;

	for (int i = 0; i < m_nCount; i++)
	{

		CPnt pt = ln.TrajFun(i * fGap);

		AddNoise(pt, fNoise, pt);

		m_pPoints[i].GetPntObject() = pt;
	}
	return true;
}

//
//   根据给定的直线数组生成点云.
//
bool CScanPointCloud::CreateFromLineArray(int nCountLines, CLine* lines, float fPointDist, float fNoise)
{
	Clear();

	CScanPointCloud cloud;

	for (int i = 0; i < nCountLines; i++)
	{
		cloud.CreateFromLine(lines[i], fPointDist, fNoise);
		*this += cloud;
	}
	return true;
}

//
//   根据给定的圆仿真生成点云。
//
bool CScanPointCloud::CreateFromCircle(CCircle& circle, float fPointDist, float fNoise)
{
	// 清除原有点云
	Clear();

	m_nCount = (int)(circle.Perimeter() / fPointDist);
	m_pPoints = new CScanPoint[m_nCount];
	if (m_pPoints == NULL)
		return false;

	float fAngleStep = 2*PI / m_nCount;

	for (int i = 0; i < m_nCount; i++)
	{
		CPnt pt = circle.m_ptCenter;

		pt.x += circle.m_fRadius * cos(fAngleStep * i);
		pt.y += circle.m_fRadius * sin(fAngleStep * i);

		AddNoise(pt, fNoise, pt);

		m_pPoints[i].GetPntObject() = pt;
	}
	return true;
}

//
//   根据给定的圆的数组生成点云。
//
bool CScanPointCloud::CreateFromCircleArray(int nCountCircles, CCircle* circles, float fPointDist, float fNoise)
{
	Clear();

	CScanPointCloud cloud;

	for (int i = 0; i < nCountCircles; i++)
	{
		cloud.CreateFromCircle(circles[i], fPointDist, fNoise);
		*this += cloud;
	}
	return true;
}

#ifdef _MSC_VER

//
//   在调试器内显示点云数据。
//
void CScanPointCloud::Dump()
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Dump();

	TRACE(_T("\n"));
}

//
//   绘制点云。
//
void CScanPointCloud::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Draw(ScrnRef, pDC,  color, nPointSize);
}
#endif
