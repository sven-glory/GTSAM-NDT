// 这段 MFC 示例源代码演示如何使用 MFC Microsoft Office Fluent 用户界面 
// (“Fluent UI”)。该示例仅供参考，
// 用以补充《Microsoft 基础类参考》和 
// MFC C++ 库软件随附的相关电子文档。  
// 复制、使用或分发 Fluent UI 的许可条款是单独提供的。  
// 若要了解有关 Fluent UI 许可计划的详细信息，请访问  
// http://go.microsoft.com/fwlink/?LinkId=238214。
//
// 版权所有(C) Microsoft Corporation
// 保留所有权利。

// iSAMAppDoc.h : CiSAMAppDoc 类的接口
//


#pragma once

#include "ScrnRef.h"
#include "PclPointCloud.h"
#include "geometry.h"
#include <deque>

//#define USE_CSM
#define  SIMU_READY_STEP 0
#define  SIMU_END_STEP  101
//#define  NEW_TRAJ


class CScanData
{
public:
	Eigen::Affine3d m_pstOdometry;
	CPclPointCloud cloud;

	CScanData(){};
};

//typedef vector<CScanData*> CScanDataSet;
typedef deque<CScanData*> CScanDataSet;

class CiSAMAppDoc : public CDocument
{
protected: // 仅从序列化创建
	CiSAMAppDoc();
	DECLARE_DYNCREATE(CiSAMAppDoc)

// 特性
public:
	CScreenReference m_ScrnRef;
	CScanDataSet m_pScanDataSet;

	CCriticalSection         m_crit;

	BOOL m_bUseCSM;
	USHORT   m_nFrameCount;
	BOOL   m_bPause;
	BOOL   m_bMapNdt;
	BOOL   m_bMapPointCloud;
// 操作
public:

// 重写
public:
	virtual BOOL OnNewDocument();
	void OnSimuStart();
	virtual void Serialize(CArchive& ar);
#ifdef SHARED_HANDLERS
	virtual void InitializeSearchContent();
	virtual void OnDrawThumbnail(CDC& dc, LPRECT lprcBounds);
#endif // SHARED_HANDLERS

// 实现
public:
	virtual ~CiSAMAppDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()

#ifdef SHARED_HANDLERS
	// 用于为搜索处理程序设置搜索内容的 Helper 函数
	void SetSearchContent(const CString& value);
#endif // SHARED_HANDLERS
public:
	
	afx_msg void OnResimu();
	afx_msg void OnUpdateResimu(CCmdUI *pCmdUI);
	afx_msg void OnUpdateRobotx(CCmdUI *pCmdUI);
	afx_msg void OnUpdateRoboty(CCmdUI *pCmdUI);
	afx_msg void OnUpdateRobotthita(CCmdUI *pCmdUI);
	afx_msg void OnPauseAndStart();
	afx_msg void OnUpdatePauseAndStart(CCmdUI *pCmdUI);
	afx_msg void OnUpdateEditlLcateState(CCmdUI *pCmdUI);
	afx_msg void OnUpdateEditCountGood(CCmdUI *pCmdUI);
	afx_msg void OnUpdateEditSourceNDT(CCmdUI *pCmdUI);
	afx_msg void OnMapNdt();
	afx_msg void OnUpdateMapNdt(CCmdUI *pCmdUI);
	afx_msg void OnMapPointCloud();
	afx_msg void OnUpdateMapPointCloud(CCmdUI *pCmdUI);
};
