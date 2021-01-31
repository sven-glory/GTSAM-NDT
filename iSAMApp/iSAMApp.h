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

// iSAMApp.h : iSAMApp Ӧ�ó������ͷ�ļ�
//
#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"       // ������


// CiSAMAppApp:
// �йش����ʵ�֣������ iSAMApp.cpp
//


class CiSAMAppView;
class CSimuScanView;

class CiSAMAppApp : public CWinAppEx
{
private:
	CMultiDocTemplate* m_pOptimizeViewTempt;
	CMultiDocTemplate* m_pSimuScanViewTempt;

public:
	CiSAMAppView* m_pOptimizeView;
	CSimuScanView* m_pSimuScanView;

public:
	CiSAMAppApp();


// ��д
public:
	virtual BOOL InitInstance();
	void CreateSimuScanView();
	CDocTemplate * GetSimuScanTempt() const;
	virtual int ExitInstance();

// ʵ��
	UINT  m_nAppLook;
	BOOL  m_bHiColorIcons;

	virtual void PreLoadState();
	virtual void LoadCustomState();
	virtual void SaveCustomState();

	DECLARE_MESSAGE_MAP()
};

extern CiSAMAppApp theApp;
