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

// SimuScanView.h : CSimuScanView ��Ľӿ�
//
#include <deque>
#pragma once

#define CONTROL_ANGLE

#define RGB_(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))


class CSimuScanView : public CScrollView
{
protected: // �������л�����
	CSimuScanView();
	DECLARE_DYNCREATE(CSimuScanView)

private:
	float	 m_fWorldSize;
	CPnt m_pstWindowCenter;
	BOOL     m_bBestFitMode;
	CPosture m_pstCur;
	CPosture m_pstLast;

	BOOL m_bLButtonDown;
	BOOL m_bRButtonDown;

	BOOL m_bSetWindowViewPort;
	CPoint m_pntRecordMouseMove;

	CSize m_sizeTotalScroll;

	CScreenReference m_ScrnRef;

	deque<CScan> m_ScanData;
	//vector<CScan> m_ScanData;
	vector<CPnt>  m_TrajPoints;
	float    m_fSimuStartAngle;
	float    m_fSimuEndAngle;
	BOOL     m_bFirstFrame;

		
public:
	//USHORT m_nFrequency;
	USHORT   m_nSimuFrequency;
	BOOL     m_bInSetTrajPointMode;
	
	static HANDLE            m_hKillThread;
	static HANDLE            m_hThreadDead;
	static UINT AutoScanProc(LPVOID pParam);
	//int m_nScanCount;

// ����
public:
	CiSAMAppDoc* GetDocument() const;

// ����
public:

// ��д
public:
	virtual void OnDraw(CDC* pDC);  // ��д�Ի��Ƹ���ͼ
	void OnMagnify(CPoint point);
	void OnReduce(CPoint point);
	void ScaleToFitView();
	void DrawPoints(vector<CPnt>& points, CScreenReference& ScrnRef, CDC* pDC, COLORREF crColorPoint, int nPointSize);
#ifndef CONTROL_ANGLE
	void Explore(CPnt pt, BOOL bMute = FALSE);
#else
	void Explore(CPosture pt, BOOL bMute = FALSE);
#endif
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual void OnInitialUpdate(); // ������һ�ε���

	void OnTimer(UINT_PTR nIDEvent);
	// ʵ��
public:
	virtual ~CSimuScanView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// ���ɵ���Ϣӳ�亯��
protected:
	afx_msg void OnFilePrintPreview();
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message);
	afx_msg void OnUsecsm();
	afx_msg void OnUpdateUsecsm(CCmdUI *pCmdUI);
	//afx_msg void OnScanfrequency();
	//afx_msg void OnUpdateScanfrequency(CCmdUI *pCmdUI);
	afx_msg void OnSimustartangle();
	afx_msg void OnSimuendangle();
	afx_msg void OnSimufrequency();
	void OnDisplaySimufrequency();
	void OnDisplaySimustartangle();
	void OnDisplaySimuendangle();
	void InitDisplay();
	//afx_msg void OnUpdateSimufrequency(CCmdUI *pCmdUI);
	
	afx_msg void OnBestShow();
	afx_msg void OnStartSimuScan();
	afx_msg void OnSetTrajPoint();
};

#ifndef _DEBUG  // iSAMAppView.cpp �еĵ��԰汾
inline CiSAMAppDoc* CSimuScanView::GetDocument() const
   { return reinterpret_cast<CiSAMAppDoc*>(m_pDocument); }
#endif

