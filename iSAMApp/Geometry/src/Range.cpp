#include <stdafx.h>
#include "Range.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   实现“范围集合”类。

CRangeSet::CRangeSet(const CRangeSet& another)
{
	for (int i = 0; i < (int)another.size(); i++)
		push_back(another[i]);
}

//
//   向集合中添加一个新的范围(递归实现)。
//
void CRangeSet::operator +=(const CRange& range)
{
#if 0
	switch (size())
	{
	case 0:
		// 如果原来的集合为空，则直接将新范围加入(成为唯一元素)
		push_back(range);
		break;

	case 1:
		// 如果原来的集合仅有一个成员，则尝试进行合并
		if (!(*this)[0].Merge(range))
		{
			push_back(range);       // 不能合并，直接加入第二个元素
		}
		break;

	default:
	{
		// 构造临时集合，只含有原集合中的最后一个元素
		CRangeSet temp;
		temp.push_back(back());

		// 将它与新元素合并
		temp += range;

		// 删除原集合中最后一个元素
		pop_back();

		// 再与生成的临时集合进行合并
		for (int i = 0; i < temp.size(); i++)
			*this += temp[i];
	}
	break;
	}
#endif

	push_back(range);

	Normalize();
}

//
//   添加一个范围集合。
//
void CRangeSet::operator +=(const CRangeSet& another)
{
	for (int i = 0; i < (int)another.size(); i++)
		*this += another[i];
}

//
//   实现集合与元素的加法。
//
CRangeSet CRangeSet::operator + (const CRange& range)
{
	CRangeSet temp(*this);
	temp += range;

	return temp;
}

//
//   实现集合与集合的加法。
//
CRangeSet CRangeSet::operator + (const CRangeSet& another)
{
	CRangeSet temp(*this);
	temp += another;
	return temp;
}

//
//   判断一个数是否在该范围集合中。
//
bool CRangeSet::Contain(float f) const
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).Contain(f))
			return true;

	return false;
}

//
//   对一个“范围集合”进行标准化。
//
void CRangeSet::Normalize()
{
	int nSize = size();
	if (nSize < 2)
		return;

	// 分配临时空间
	bool* bDelete = new bool[nSize];

	// 开始时，标明所有项都不删除
	for (int i = 0; i < nSize; i++)
		bDelete[i] = false;

	// 逐项进行合并
	for (int i = 0; i < nSize; i++)
	{
		// 跳过已删除的项
		if (bDelete[i])
			continue;

		for (int j = 0; j < nSize; j++)
		{
			// 跳过已删除的项
			if (bDelete[j] || (i == j))
				continue;

			// 如果两项可合并，则标明j项删除
			if (at(i).Merge(at(j)))
			{
				bDelete[j] = true;
			}
		}
	}

	// 实际删除所有已标记“delete”的项
	for (int i = size() - 1; i >= 0; i--)
	{
		if (bDelete[i])
			erase(begin() + i);
	}
	delete[]bDelete;

	// 重新排序
	for (int i = 0; i < (int)size()-1; i++)
	{
		for (int j = i + 1; j < (int)size(); j++)
		{
			if (at(i) > at(j))
			{
				CRange temp = at(i);
				at(i) = at(j);
				at(j) = temp;
			}
		}
	}
}

//
//   赋值。
//
void CRangeSet::operator = (const vector<CRange>& another)
{
	clear();

	for (int i = 0; i < (int)another.size(); i++)
		push_back(another[i]);
}

