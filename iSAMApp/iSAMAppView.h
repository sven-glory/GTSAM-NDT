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

// iSAMAppView.h : CiSAMAppView ��Ľӿ�
//

#pragma once
#include "RangeScanner.h"
#include "RawScan.h"
#include "PointCloudMap.h"
#include "NdtKeyFrame.h"
//#include "Loader.h"
//#include <afxmt.h>
#include "ndt_fuser\ndt_fuser_node.h"
#include "ndt_map\ndt_map.h"
#include "ndt_map\lazy_grid.h"
#include "vector.h"
//
//using namespace std;

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// a Gauss-Newton solver
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include "gtsam/nonlinear/ISAM2.h"

using namespace gtsam;
using namespace std;


class CiSAMAppView : public CScrollView
{
private:
	CSize m_sizeTotalScroll;
	BOOL     m_bBestFitMode;

	BOOL m_bLButtonDown;
	BOOL m_bRButtonDown;

	BOOL m_bSetWindowViewPort;
	CPoint m_pntRecordMouseMove;

	CRangeScanner* m_RealScanner;
	BOOL m_bLaserScan;
	BOOL m_bFirstFrame;
	BOOL m_bConnectLaserDelay;
	DWORD m_dwTicRec;
	BOOL m_bStopReflesh;
	BOOL m_bRotateMapMode;
	USHORT m_nRotateStage;
	CPnt m_ptRotateCenter;
	CPnt m_ptFrom;
	CPnt m_ptTo;
public:
	CPosture m_RobotPose;
	BOOL m_bLocateState;
	int m_nCountGood;
	int m_nSourceNDT;
	//Loader *loader;

	NonlinearFactorGraph graph;
	Values initial_estimate;
	ISAM2 *isam;
	Values  isam_current_estimate;

	noiseModel::Diagonal::shared_ptr prior_noise_;
	noiseModel::Diagonal::shared_ptr odom_noise_;
	noiseModel::Diagonal::shared_ptr constraint_noise_;

	CPointCloudMap *ndtcloudMap;

	CPointCloudMap *cloudMap;//���ƽ�ͼ��

	//CNdtKeyFrame ndtKeyFrame;
	//perception_oru::NDTMap *NdtKeyFrame;

	vector<CNdtKeyFrame> NdtKeyFrames;
	//CNdtKeyFrame NdtKeyFrames[100];
	int keyFrameCount;
	int FrameCount;

	BOOL  m_bMoveCurFrameMode;
	USHORT m_nMoveCurFrameStage;

	BOOL  m_bRotateCurFrameMode;
	USHORT m_nRotateCurFrameStage;
	CPnt m_ptRotateCurFrameCenter;
	CPnt m_ptCurFrameFrom;
	CPnt m_ptCurFrameTo;

	BOOL  m_bOptimizeConfirm;

	Eigen::Affine3d TransCurFrame;

	BOOL   m_bDisPointCloud;
	
protected: // �������л�����
	CiSAMAppView();
	DECLARE_DYNCREATE(CiSAMAppView)

// ����
public:
	CiSAMAppDoc* GetDocument() const;

	static HANDLE            m_hKillThread;
	static HANDLE            m_hThreadDead;

	static HANDLE            m_hKillThreadpc;
	static HANDLE            m_hThreadDeadpc;

	static UINT ProcessNDTProc(LPVOID pParam);
	static UINT ProcessPointCloudProc(LPVOID pParam);
	// ����
public:
	int    m_nGetDataCount;
// ��д
public:
	virtual void OnDraw(CDC* pDC);  // ��д�Ի��Ƹ���ͼ
	void OnTimer(UINT_PTR nIDEvent);
	void OnMagnify(CPoint point);
	void OnReduce(CPoint point);
	void SupportNDT();
	void SupportPointCloudMap();
	CScan* TransformCloud(const std::shared_ptr<sensor::CRawPointCloud>& pRawCloud/*, CScan* pScan*/);
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual void OnInitialUpdate(); // ������һ�ε���

// ʵ��
public:
	virtual ~CiSAMAppView();
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
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnConnectlaser();
	void LoadParam();
	void InitDisplay();
	void OnDisplayLaserIP();
	void OnDisplayLaserLimitNear();
	void OnDisplayLaserLimitFar();
	void OnDisplayLaserInstallAngle();
	void OnDisplayLaserScanPointNum();
	void OnDisplayLaserScanFrequency();
	afx_msg void OnUpdateLaserip(CCmdUI *pCmdUI);
	afx_msg void OnLaserip();
	afx_msg void OnLaserlimitnear();
	afx_msg void OnLaserlimitfar();
	afx_msg void OnInstallangle();
	afx_msg void OnScanpointsnum();
	afx_msg void OnScanfrequency();
	
	afx_msg void OnStopLaserScanner();
	afx_msg void OnRotateMap();
	void DealWithRotateMap(CPoint point);
	afx_msg void OnButtonRotateMap();
	void OnLButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnMoveGraph();
	afx_msg void OnRotateGraph();
	void DealWithRotateCurFrame(CPoint point);
	void DealWithMoveCurFrame(CPoint point);
	afx_msg void OnOpConfirm();
};

#ifndef _DEBUG  // iSAMAppView.cpp �еĵ��԰汾
inline CiSAMAppDoc* CiSAMAppView::GetDocument() const
   { return reinterpret_cast<CiSAMAppDoc*>(m_pDocument); }
#endif

