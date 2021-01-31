// SelectMapType.cpp: 实现文件
//

#include "stdafx.h"
#include "iSAMApp.h"
#include "SelectMapType.h"
#include "afxdialogex.h"


// SelectMapType 对话框



IMPLEMENT_DYNAMIC(SelectMapType, CDialogEx)

SelectMapType::SelectMapType(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_SEL_MAP, pParent)
{

}

SelectMapType::~SelectMapType()
{
}

void SelectMapType::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(SelectMapType, CDialogEx)
	ON_BN_CLICKED(IDC_POINTMAP, &SelectMapType::OnBnClickedPointMap)
	ON_BN_CLICKED(IDC_NDTMAP, &SelectMapType::OnBnClickedNdtMap)
END_MESSAGE_MAP()


// SelectMapType 消息处理程序


void SelectMapType::OnBnClickedPointMap()
{
	// TODO: 在此添加控件通知处理程序代码
	m_nMapType = POINTCLOUD;
	CString str;
	str.Format(_T("点云地图"));
	GetDlgItem(IDC_DIS_MAP_TYPE)->SetWindowTextW(str);
}


void SelectMapType::OnBnClickedNdtMap()
{
	// TODO: 在此添加控件通知处理程序代码
	m_nMapType = NDTMAP;
	CString str;
	str.Format(_T("NDT地图"));
	GetDlgItem(IDC_DIS_MAP_TYPE)->SetWindowTextW(str);
}
