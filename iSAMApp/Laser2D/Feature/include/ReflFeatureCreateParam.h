#pragma once

// 反光板识别缺省参数值
#define MIN_REFLECTOR_INTENSITY         800     // 反光板数据强度门限
#define MAX_REFLECTOR_SIZE              1.0f    // 反光板数据最大允许宽度(起点到终点的距离)
#define MAX_REFLECTOR_RADIUS_VARIANCE   1.0f    // 同一反光板相邻极径最大允许距离差(单位:米)
#define MIN_DIST_BETWEEN_REFLECTORS     0.3f    // 任意两个反光板之间所允许的最小距离(单位:米)

///////////////////////////////////////////////////////////////////////////////
// 从点云生成点特征时所用的参数
struct CReflectorCreationParam
{
public:
	int   nMinReflectorIntensity;        // 反光板强度门限
	float fMaxReflectorSize;             // 反光板最大宽度(起点到终点的距离)
	float fMaxPolarRadiusVariance;       // 同一反光板相邻极径最大允许距离差(单位:米)
	float fMinDistBetweenReflectors;     // 任意两反光板之间所允许的最小距离(单位:米)

public:
	CReflectorCreationParam()
	{
		nMinReflectorIntensity = MIN_REFLECTOR_INTENSITY;
		fMaxReflectorSize = MAX_REFLECTOR_SIZE;
		fMaxPolarRadiusVariance = MAX_REFLECTOR_RADIUS_VARIANCE;
		fMinDistBetweenReflectors = MIN_DIST_BETWEEN_REFLECTORS;
	}

	// 从二进制文件装入参数
	bool Load(FILE* fp)
	{
		if (fread(&nMinReflectorIntensity, sizeof(int), 1, fp) != 1)
			return false;

		float f[3];
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;

		fMaxReflectorSize = f[0];
		fMaxPolarRadiusVariance = f[1];
		fMinDistBetweenReflectors = f[2];

		return true;
	}

	// 将参数保存到二进制文件
	bool Save(FILE* fp)
	{
		if (fwrite(&nMinReflectorIntensity, sizeof(int), 1, fp) != 1)
			return false;

		float f[3];
		f[0] = fMaxReflectorSize;
		f[1] = fMaxPolarRadiusVariance;
		f[2] = fMinDistBetweenReflectors;

		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		return true;
	}
};
