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

// SimuScanView.h : CSimuScanView 类的接口
//
#include <deque>
#pragma once

#define CONTROL_ANGLE

#define RGB_(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))


class CSimuScanView : public CScrollView
{
protected: // 仅从序列化创建
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

// 特性
public:
	CiSAMAppDoc* GetDocument() const;

// 操作
public:

// 重写
public:
	virtual void OnDraw(CDC* pDC);  // 重写以绘制该视图
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
	virtual void OnInitialUpdate(); // 构造后第一次调用

	void OnTimer(UINT_PTR nIDEvent);
	// 实现
public:
	virtual ~CSimuScanView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
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

#ifndef _DEBUG  // iSAMAppView.cpp 中的调试版本
inline CiSAMAppDoc* CSimuScanView::GetDocument() const
   { return reinterpret_cast<CiSAMAppDoc*>(m_pDocument); }
#endif

