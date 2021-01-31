#ifndef __CRange
#define __CRange

#include <vector>

using namespace std;

/////////////////////////////////////////////////////////////////////
// 关于范围的描述。
class CRange
{
public:
	float fFrom;     // 起始处
	float fTo;       // 终止处

public:
	CRange(float _from, float _to)
	{
		fFrom = _from;
		fTo = _to;
	}

	CRange() 
	{
		fFrom = 0;
		fTo = 0;
	}

	CRange& CRangeObject()
	{
		return *this;
	}

	// 判断此范围是否合法
	bool IsValid() { return fFrom >= fTo; }

	// 判断此范围是否大于另一个
	bool operator > (const CRange& another)
	{
		return (fFrom > another.fTo);
	}

	// 判断此范围是否小于另一个
	bool operator < (const CRange& another)
	{
		return (fTo < another.fFrom);
	}

	// 判断一个数是否在该范围中
	bool Contain(float f) const
	{
		return (f >= fFrom && f <= fTo);
	}

	// 判断此范围是否与另一个范围有重叠区
	bool IsOverlap(const CRange& another)
	{
		if (*this > another || *this < another)
			return false;
		else
			return true;
	}

	// 尝试将两个范围进行合并
	bool Merge(const CRange& another)
	{
		if (!IsOverlap(another))
			return false;

		if (another.fFrom < fFrom)
			fFrom = another.fFrom;

		if (another.fTo > fTo)
			fTo = another.fTo;

		return true;
	}
};

/////////////////////////////////////////////////////////////////////
// 定义“范围段的集合”。

class CRangeSet : public vector<CRange>
{
private:

public:
	CRangeSet() {}

	// 直接按另一集合构造
	CRangeSet(const CRangeSet& another);

	// 添加一个范围
	void operator +=(const CRange& range);

	// 添加一个范围集合
	void operator +=(const CRangeSet& another);

	// 实现集合与元素的加法
	CRangeSet operator + (const CRange& range);

	// 实现集合与集合的加法
	CRangeSet operator + (const CRangeSet& another);

	// 赋值
	void operator = (const vector<CRange>& another);

	// 判断一个数是否在该范围集合中
	bool Contain(float f) const;

	// 对一个“范围集合”进行标准化
	void Normalize();
};
#endif
