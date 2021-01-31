#include <stdafx.h>
#include "ScannerParam.h"
#include "Frame.h"

///////////////////////////////////////////////////////////////////////////////

CLaserScannerParam::CLaserScannerParam()
{
	Set(-PI, PI, 3600);
	m_pst.Create(0, 0, 0);
	m_bUseCreateModel = m_bUseLocalize = true;
}

//
//   设置起始角、终止线和扫描线数。
//
void CLaserScannerParam::Set(float fStartAngle, float fEndAngle, int nLineCount)
{
	m_fStartAngle = fStartAngle;
	m_fEndAngle = fEndAngle;
	m_nLineCount = nLineCount;
	m_fReso = (m_fEndAngle - m_fStartAngle) / nLineCount;
}

//
//   从二进制文件中读取参数。
//
bool CLaserScannerParam::LoadBinary(FILE* fp, int nFileVersion)
{
	float f[3];
	int n;

	if (fread(f, sizeof(float), 2, fp) != 2)
		return false;

	if (fread(&n, sizeof(int), 1, fp) != 1)
		return false;

	Set(f[0], f[1], n);

	// 当版本在V2.00以上时，需要读入激光器的相对安装姿态
	if (nFileVersion >= 200)
	{
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;

		m_pst.Create(f[0], f[1], f[2]);

		// 在此临时设定传感器实际可用角度范围(将来应从数据文件读入)
		int nRangeCount;
		if (fread(&nRangeCount, sizeof(int), 1, fp) != 1)
			return false;

		for (int i = 0; i < nRangeCount; i++)
		{
			if (fread(f, sizeof(float), 2, fp) != 2)
				return false;

			CRange range(f[0], f[1]);
			m_AppAngleRange.push_back(range);
		}
	}
	else
	{
		CRange range(-PI, PI);
		m_AppAngleRange.push_back(range);
	}
	return true;
}

//
//   向二进制文件中写入参数。
//
bool CLaserScannerParam::SaveBinary(FILE* fp, int nFileVersion)
{
	float f[3] = { m_fStartAngle, m_fEndAngle };

	if (fwrite(f, sizeof(float), 2, fp) != 2)
		return false;

	if (fwrite(&m_nLineCount, sizeof(int), 1, fp) != 1)
		return false;

	// 当版本在2.0以上时，需要读入激光器的相对安装姿态
	if (nFileVersion >= 200)
	{
		f[0] = m_pst.x;
		f[1] = m_pst.y;
		f[2] = m_pst.fThita;

		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		int nRangeCount = (int)m_AppAngleRange.size();
		if (fwrite(&nRangeCount, sizeof(int), 1, fp) != 1)
			return false;

		for (int i = 0; i < nRangeCount; i++)
		{
			f[0] = m_AppAngleRange[i].fFrom;
			f[1] = m_AppAngleRange[i].fTo;

			if (fwrite(f, sizeof(float), 2, fp) != 2)
				return false;
		}
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////

//
//   设置当前扫描器(编号:nCurScanner)与参考扫描器(编号:nRefScaner)的相对姿态关系。
//   注：只改度nCurScanner的安装姿态，nRefScanner的安装姿态不作调整。
//
void CScannerGroupParam::SetRelativePosture(int nRefScanner, int nCurScanner, const CPosture& pst)
{
	const CLaserScannerParam& ref = at(nRefScanner);
	CLaserScannerParam& cur = at(nCurScanner);

	CFrame frmRef(ref.m_pst);
	CPosture pst1(pst);

#if 0
	pst1.x = -pst1.x;
	pst1.y = -pst1.y;
	pst1.fThita = -pst1.fThita;
#endif

	// 得到当前激光器在机器人坐标系内的姿态
	pst1.InvTransform(frmRef);
	cur.m_pst = pst1;
}

//
//   从二进制文件中读取参数。
//
bool CScannerGroupParam::LoadBinary(FILE* fp, int nFileVersion)
{
	clear();
	int nScannerCount = 1;         // 第2.00版以下仅支持一个激光扫描器

	// 从第2.00版开始，数据集支持多个激光扫描器同时采集
	if (nFileVersion >= 200)
	{
		if (fread(&nScannerCount, sizeof(int), 1, fp) != 1)
			return false;
	}

	// 依次读入激光器参数
	for (int i = 0; i < nScannerCount; i++)
	{
		CLaserScannerParam Param;
		if (!Param.LoadBinary(fp, nFileVersion))
			return false;

		push_back(Param);
	}

	return true;
}

// 向二进制文件中写入参数
bool CScannerGroupParam::SaveBinary(FILE* fp, int nFileVersion)
{
	int nScannerCount = size();

	if (nFileVersion >= 200)
	{
		if (fwrite(&nScannerCount, sizeof(int), 1, fp) != 1)
			return false;
	}

	// 依次写入各激光扫描器的参数
	for (int i = 0; i < nScannerCount; i++)
	{
		if (!at(i).SaveBinary(fp, nFileVersion))
			return false;
	}

	return true;
}

//
//   设置各扫描器是否应用于建模。
//
void CScannerGroupParam::UseWhenCreateModel(int nIdx, bool bYesOrNo)
{
	at(nIdx).m_bUseCreateModel = bYesOrNo;
}

//
//   设置各扫描器是否应用于定位。
//
void CScannerGroupParam::UseWhenLocalize(int nIdx, bool bYesOrNo)
{
	at(nIdx).m_bUseLocalize = bYesOrNo;
}
