// ��� MFC ʾ��Դ������ʾ���ʹ�� MFC Microsoft Office Fluent �û����� 
// (��Fluent UI��)����ʾ�������ο���
// ���Բ��䡶Microsoft ������ο����� 
// MFC C++ ������渽����ص����ĵ���  
// ���ơ�ʹ�û�ַ� Fluent UI ����������ǵ����ṩ�ġ�  
// ��Ҫ�˽��й� Fluent UI ��ɼƻ�����ϸ��Ϣ�������  
// http://go.microsoft.com/fwlink/?LinkId=238214��
//
// ��Ȩ����(C) Microsoft Corporation
// ��������Ȩ����

// iSAMAppDoc.h : CiSAMAppDoc ��Ľӿ�
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
protected: // �������л�����
	CiSAMAppDoc();
	DECLARE_DYNCREATE(CiSAMAppDoc)

// ����
public:
	CScreenReference m_ScrnRef;
	CScanDataSet m_pScanDataSet;

	CCriticalSection         m_crit;

	BOOL m_bUseCSM;
	USHORT   m_nFrameCount;
	BOOL   m_bPause;
	BOOL   m_bMapNdt;
	BOOL   m_bMapPointCloud;
// ����
public:

// ��д
public:
	virtual BOOL OnNewDocument();
	void OnSimuStart();
	virtual void Serialize(CArchive& ar);
#ifdef SHARED_HANDLERS
	virtual void InitializeSearchContent();
	virtual void OnDrawThumbnail(CDC& dc, LPRECT lprcBounds);
#endif // SHARED_HANDLERS

// ʵ��
public:
	virtual ~CiSAMAppDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// ���ɵ���Ϣӳ�亯��
protected:
	DECLARE_MESSAGE_MAP()

#ifdef SHARED_HANDLERS
	// ����Ϊ����������������������ݵ� Helper ����
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
