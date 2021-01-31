#include "stdafx.h"
#include "misc.h"
#include "ScanPointCloud.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#define MIN_ANGLEDIST              DEG2RAD(3.0)

///////////////////////////////////////////////////////////////////////////////
//   ��CScanPointCloud�����ʵ�֡�

//
//   ����ָ���ĵ��������ɶ���(ֻ����ռ�)��
//
CScanPointCloud::CScanPointCloud(int nNum)
{
	m_nCount = nNum;
	m_pPoints = new CScanPoint[nNum];

	ASSERT(m_pPoints != NULL);
}

//
//   ��������һ���������ɶ���(����ȫ����ɨ���)��
//
CScanPointCloud::CScanPointCloud(const CScanPointCloud& Cloud)
{
	m_nCount = 0;
	m_pPoints = NULL;

	*this = Cloud;
}

//
//   ���ɿյĶ���
//
CScanPointCloud::CScanPointCloud()
{
	m_nCount = 0;
	m_pPoints = NULL;
}

CScanPointCloud::~CScanPointCloud()
{
	Clear();
}

//
//   �������ԭ�������ݡ�
//
void CScanPointCloud::Clear()
{
	if (m_pPoints != NULL)
		delete []m_pPoints;

	m_nCount = 0;
}

//
//   Ϊ�������ݷ���ռ䡣
//
BOOL CScanPointCloud::Create(int nNum)
{
	m_pPoints = new CScanPoint[nNum];

	if (m_pPoints == NULL)
	{
		m_nCount = 0;
		return FALSE;
	}
	else
	{
		m_nCount = nNum;
		return TRUE;
	}
}

//
//   ���ء�=����������
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
}

//
//   ���ء�+=����������
//
void CScanPointCloud::operator += (const CScanPointCloud& Cloud2)
{
	// �ȸ���ԭ���ĵ��ơ�
	CScanPointCloud CopyThis(*this);

	// ���ԭ��������
	Clear();

	// ���·���ռ��Ա������������Ƶ�����
	VERIFY(Create(CopyThis.m_nCount + Cloud2.m_nCount));

	int i;

	// ���ƻ�ԭ������
	for (i = 0; i < CopyThis.m_nCount; i++)
		m_pPoints[i] = CopyThis.m_pPoints[i];

	// ���������µ�������
	for (i = 0; i < Cloud2.m_nCount; i++)
		m_pPoints[CopyThis.m_nCount+i] = Cloud2.m_pPoints[i];
}

//
//   ��ȫ�����ƶ�ָ���ľ��롣
//
void CScanPointCloud::Move(float fDx, float fDy)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Move(fDx, fDy);
}

//
//   ��ȫ������ָ�������ĵ������ת��
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

//
//   ������ƿɼ����(Ŀǰδʹ��)��
//   Returns the size of the visible area in scan.
//
float CScanPointCloud::AreaSize()
{
	const CScanPoint *sp, *last = NULL;
	float total = 0.0;
	float x=0.0, y=0.0;
	bool start = TRUE;
	long i;

	if (m_nCount == 0)
		return 0.0;

	sp = m_pPoints;
	for (i = 0; i < m_nCount; i++)
	{
		if (start)
		{
			x = sp[i].x;
			y = sp[i].y;
			start = FALSE;
		}
		else if (last != NULL)
		{
			float dx = last->x - x;
			float dy = last->y - y;
			float c = sqrt(dx*dx + dy*dy);
			float a = atan2(dy, dx);
			float h;
		
			dx = sp[i].x - x;
			dy = sp[i].y - y;
			h = dx*sin(-a) + dy*cos(-a);
			total += 0.5 * h * c;
		}
		last = &sp[i];
	}
	
	return total;
}

//
//   ����ɨ�������ռ��ɨ��ǡ�
//
float CScanPointCloud::FieldOfView()
{
	float visible = 0.0;         // ���ӽ�
	float dist;                  // ������֮���(������)�ǶȲ�

	// �����������2��ֱ�ӷ���0
	if (m_nCount < 2)
		return 0.0;

	int i;
	for (i = 1; i < m_nCount; i++)
	{
		// ȡ��������֮�ǵĽǶȲ�
		dist = m_pPoints[i-1].AngleDistanceTo(m_pPoints[i]);

		// ���������֮��ĽǶȲ�ϴ���Ϊ����ɨ�����Ƕ���
		if (dist < MIN_ANGLEDIST)
			visible += dist;
	}

	// ��Ҫ���ǵ�1�������m_nCount-1����֮��ĽǶȲ�
	dist = m_pPoints[i-1].AngleDistanceTo(m_pPoints[0]);
	if (dist < MIN_ANGLEDIST)
		visible += dist;

	// �Ƕȹ�һ��
	if (visible > 2*PI)
		visible = 2*PI;

	return visible;
}

//
//   ����Ӹ����㵽�����и����ƽ�����롣
//
float CScanPointCloud::AverageDistanceFrom(const CPnt& pt)
{
	float sum = 0.0;

	for (int i = 0; i < m_nCount; i++)
		sum += m_pPoints[i].DistanceTo(pt);

	return (sum / m_nCount);
}

//
//   ��������ĵ㵽�����и����ƽ�����롣
//
float CScanPointCloud::AverageDistanceFromGravity()
{
	CPnt pt = CenterOfGravity();
	return AverageDistanceFrom(pt);
}

//
//   ������Ƶ����Ĳ�(�������о���������Զ�����������֮��ľ���)��
//
float CScanPointCloud::DynamicsGravity()
{
	float min = HUGE_VAL;
	float max = 0.0;
	float d;

	if (m_nCount <= 1)
		return 0.0;

	// ȡ�����ĵ�
	CPnt ptCenter = CenterOfGravity();

	// �������������ĵ���Զ���������
	for (int i = 0; i < m_nCount; i++)
	{
		d = m_pPoints[i].DistanceTo(ptCenter);
		min = MIN(min, d);
		max = MAX(max, d);
	}

	return (max - min);
}

//
//   ���ص��Ƶ����ĵ㡣
//
CPnt CScanPointCloud::CenterOfGravity()
{
	CPnt pt(HUGE_VAL, HUGE_VAL);               // X,Y�����ʼ��Ϊ���ֵ

	if (m_nCount != 0)
	{
		float sx = 0.0;
		float sy = 0.0;

		for (int i = 0; i < m_nCount; i++)
		{
			sx += m_pPoints[i].x;
			sy += m_pPoints[i].y;
		}

		pt.x = sx / m_nCount;
		pt.y = sy / m_nCount;
	}

	return pt;
}

//
//   ������ƵĻ��Ƴ��ȡ�
//
float CScanPointCloud::TotalLength()
{
	float sum = 0.0, d, dstart = 0.0;
	const float mind = 100.0, maxd = 250.0;
	long i, start = 0;

	if (m_nCount < 2)
		return 0.0;
   
	for (i = 1; i < m_nCount; i++)
	{
		d = (m_pPoints[i-1]).DistanceTo(m_pPoints[i]);
		if (d > maxd)			   /* a gap? */
		{
			sum += dstart;
			start = i;
			dstart = 0.0;
		}
		else if (dstart > mind)		 /* far enough from start? */
		{
			sum += dstart;
			start = i-1;
			dstart = d;
		}
		else
		{
			dstart = (m_pPoints[start]).DistanceTo(m_pPoints[i]);
		}
	}

	sum += dstart;
	d = (m_pPoints[i-1]).DistanceTo(m_pPoints[0]);
	if (d < maxd)								   /* not a gap? */
		sum += d;

	return sum;
}

//
//   Get the X coordinate of the left-most point.
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
//   Get the Y coordinate of the top-most point.
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
//   Get the X coordinate of the right-most point.
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
//   Get the Y coordinate of the bottom-most point.
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
//   Get the width of the map area.
//
float CScanPointCloud::Width()
{
	return RightMost() - LeftMost();
}

//
//   Get the height of the map area.
//
float CScanPointCloud::Height()
{
	return TopMost() - BottomMost();
}

//
//   ��ָ���ĵ��Ϊ���ģ���ָ���İ뾶�����е���й��ˣ�ֻ���´��ڰ뾶���ڵĵ㡣
//   (��˴���󣬻���һЩ�ռ�δ��ʹ��)��
//
void CScanPointCloud::Reduce(CPnt& ptCenter, float dRadius)
{
	int nNewCount = 0;                     // ���Ƶ������

	for (int i = 0; i < m_nCount; i++)
	{
		// ����õ������ĵ�ľ���С��ָ���뾶���򽫸õ������µ�����
		if (m_pPoints[i].DistanceTo(ptCenter) <= dRadius)
		{
			if (nNewCount != i)
				m_pPoints[nNewCount] = m_pPoints[i];

			nNewCount++;
		}
	}

	// �趨�µ����е������(���ƿռ��п����ж���δ�õĿռ�)
	m_nCount = nNewCount;
}

int CScanPointCloud::ClosestPoint(const CPnt& ptNew, float* fDist)
{
	int nIndex = -1;
	float fMinDist = FLT_MAX;

	for (int i = 0; i < m_nCount; i++)
	{
		float f = m_pPoints[i].DistanceTo(ptNew);
		if (f < fMinDist)
		{
			nIndex = i;
			fMinDist = f;
		}
	}

	if (fDist != NULL)
		*fDist = fMinDist;

	return nIndex;
}

void CScanPointCloud::Add(int nIndex, int nCurCount, const CScanPoint& sp)
{
	if (nIndex < 0 || nCurCount >= m_nCount || nIndex > nCurCount)
		return;

	for (int i = nCurCount; i > nIndex; i--)
		m_pPoints[i] = m_pPoints[i-1];

	m_pPoints[nIndex] = sp;
}

//
//   �ӵ�����ɾ��һ���㡣
//
void CScanPointCloud::Delete(int nIndex)
{
	if (nIndex < 0 || nIndex >= m_nCount)
		return;

	for (int i = nIndex; i < m_nCount-1; i++)
		m_pPoints[i] = m_pPoints[i+1];

	// ���ĵ����е������(���ƿռ��п����ж���δ�õĿռ�)
	m_nCount--;
}

//
//   �˶Ե����Ƿ����ָ���ĵ㡣
//   ����ֵ��
//     -1: δ�ҵ�
//     0~n: �ҵ��ĵ�����
//
int CScanPointCloud::ContainPoint(const CPnt& pt, float fThreshHoldDist)
{
	for (int i = 0; i < m_nCount; i++)
		if (m_pPoints[i].DistanceTo(pt) <= fThreshHoldDist)
			return i;

	return -1;
}

void CScanPointCloud::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Draw(ScrnRef, pDC, nPointSize, color);
}

BOOL CScanPointCloud::LoadFromFile(FILE* file)
{
	int nCount;
	fscanf(file, "%d\n", &nCount);

	Clear();
	if (!Create(nCount))
		return FALSE;

	for (int i = 0; i < nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];
		fscanf(file, "%f\t%f\t%f\t%f\n", &(sp.x), &(sp.y), &(sp.a), &(sp.r));
	}

	return TRUE;
}

BOOL CScanPointCloud::SaveToFile(FILE* file)
{
	fprintf(file, "%ld\n", m_nCount);
	for (int i = 0; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];
		fprintf(file, "%.6f\t%.6f\t%.6f\t%.6f\n", sp.x, sp.y, sp.a, sp.r);
	}

	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////

/*
** Calculates a line in the form (n1, n2) * (x, y) + c = 0,
** so that the leastsquare sum of the corresponding scan points
** is minimized.
** sp is an array of scan points, num specifies the number of scan points,
** n1, n2 and c are the result values.
*/
BOOL RegressionLine(const CScanPoint *sp, long num, float *n1, float *n2, float *c)
{
	float xm = 0.0, ym = 0.0, xxm = 0.0, yym = 0.0, xym = 0.0;
	float a, dx, dy;
	long i;

	if (num <= 1 || n1==NULL || n2==NULL || c==NULL)
		return FALSE;

	for (i = 0; i < num; i++)
	{
		float x = sp[i].x;
		float y = sp[i].y;

		xm += x;
		ym += y;
		xxm += x*x;
		yym += y*y;
		xym += x*y;
	}

	a = 0.5f * atan2((float)(-2.0*(xym - xm*ym/num)), (float)(yym - ym*ym/num - xxm + xm*xm/num));
	*n1 = cos(a);
	*n2 = sin(a);
	dx = sp[num-1].x - sp[0].x;
	dy = sp[num-1].y - sp[0].y;
	if (dx * *n2 - dy * *n1 > 0.0)
	{
		*n1 = -*n1;
		*n2 = -*n2;
	}
	*c = -(*n1 * xm + *n2 * ym)/num;

	return TRUE;
}

/*
** Calculates variance of distance from a set of scan points 
** given in (sp, num) to a line given in (n1, n2, c).
*/
float LineDeviation(CScanPoint *sp, long num, float n1, float n2, float c)
{
	float sum = 0.0, sigma2;
	long i;

	for (i = 0; i < num; i++)
	{
		float val = n1*sp[i].x + n2*sp[i].y + c;
		sum += val*val;
	}

	sigma2 = sum / (float)num;
	return sigma2;
}

void CScanPointCloud::Dump()
{
	TRACE(_T("Dumping Scan point cloud:(%d points)\n"), m_nCount);

	for (int i = 0; i < m_nCount; i++)
		TRACE(_T("%.2f, %.2f\n"), m_pPoints[i].x, m_pPoints[i].y);
	
	TRACE(_T("\n"));
}

void CScanPointCloud::DebugFilter(float fMinX, float fMaxX, float fMinY, float fMaxY)
{
	CScanPoint* sp;
	int nNewCount = 0;
	for (int i = 0; i < m_nCount; i++)
	{
		sp = &(m_pPoints[i]);
		if ((sp->x < fMinX) || (sp->x > fMaxX) || (sp->y < fMinY) || (sp->y > fMaxY))
			continue;
		else
		{
			TRACE(_T("%.2f, %.2f\n"), sp->x, sp->y);
			m_pPoints[nNewCount++] = m_pPoints[i];
		}
	}
	m_nCount = nNewCount;
}

