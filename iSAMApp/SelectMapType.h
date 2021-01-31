#pragma once

#define POINTCLOUD   0
#define NDTMAP       1

// SelectMapType 对话框

class SelectMapType : public CDialogEx
{
	DECLARE_DYNAMIC(SelectMapType)

public:
	SelectMapType(CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~SelectMapType();

	int    m_nMapType;

// 对话框数据

	enum { IDD = IDD_SEL_MAP };


protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedPointMap();
	afx_msg void OnBnClickedNdtMap();
};
