#ifndef __CCorrList
#define __CCorrList

class CCsmScan;

// 两个点云的对应关系表。
class CCorrList
{
public:
	int  m_nCount;
	int* m_pIdx;

public:
	CCorrList()
	{
		m_nCount = 0;
		m_pIdx = NULL;
	}

	CCorrList(const CCorrList& another)
	{
		m_pIdx = NULL;
		Create(another.m_nCount);

		for (int i = 0; i < m_nCount; i++)
			m_pIdx[i] = another.m_pIdx[i];
	}

	CCorrList& operator = (const CCorrList& another)
	{
		Create(another.m_nCount);

		for (int i = 0; i < m_nCount; i++)
			m_pIdx[i] = another.m_pIdx[i];

		return *this;
	}

	~CCorrList()
	{
		Clear();
	}

	void Clear()
	{
		if (m_pIdx != NULL)
		{
			delete[]m_pIdx;
			m_pIdx = NULL;
		}
		m_nCount = 0;
	}

	bool Create(int nCount)
	{
		Clear();

		m_pIdx = new int[nCount];
		if (m_pIdx != NULL)
		{
			m_nCount = nCount;
			for (int i = 0; i < nCount; i++)
				m_pIdx[i] = -1;

			return true;
		}
		else
			return false;
	}

	// 根据扫描数据生成匹配对应表
	bool CreateFromScan(const CCsmScan& scan);

	// 核对两个小段的点是否可以认为是重叠的
	bool CheckSmallSegOverlap(int nSeg1Start, int nSeg1End, int nSeg2Start, int nSeg2End);

	// 核对两个长段(即直线)的点是否可以认为是有相当部分是重叠的
	bool CheckLineSegOverlap(int nSeg1Start, int nSeg1End, int nSeg2Start, int nSeg2End);
};
#endif
