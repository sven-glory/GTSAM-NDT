//
//   The interface of class "CRawMap".
//

#include <fstream>
#include "sensor/RawScan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "RawScan".
namespace sensor {

//
//   �Ӷ������ļ��ж�ȡɨ�����ݡ�
//
bool CRawPointCloud::LoadBinary(FILE* fp, float fStartAngle, float fEndAngle, int nLineCount,
    int nFileVersion)
{
    unsigned long dis_size = static_cast<unsigned long>(nLineCount + 1); // ÿ��ɨ��Ϊ1080���������Ϊ1081
    unsigned long inten_size = static_cast<unsigned long>(nLineCount + 1);

    num_points = static_cast<unsigned int>(nLineCount);
    //laser_type = m_uType; //���ݼ����޴��������Ͳ���
    start_angle = fStartAngle;
    end_angle = fEndAngle;
    distance.clear();
    intensity.clear();
    distance.reserve(dis_size);
    intensity.reserve(inten_size);

    // ����ɨ��ĽǷֱ���
    float fAngReso = (fEndAngle - fStartAngle) / nLineCount;

    // ����汾��V2.10���ϣ��ڴ���Ҫ����ʱ���
    unsigned int uTimeStamp = 0;
    if (nFileVersion >= 210)
    {
        if (fread(&uTimeStamp, sizeof(unsigned int), 1, fp) != 1)
            return false;

        timestamp_raw = uTimeStamp;
        timestamp_sync = uTimeStamp;
    }

    // ���ζ������е������
    for (unsigned int i = 0; i < num_points; i++)
    {
        float r = 0;
        int nIntensity = 0;

        // ����ļ���ʽ�汾������2.00��(����+ǿ�ȹ���8���ֽ�)
        if (nFileVersion < 200)
        {
            // ���뼫��
            if (fread(&r, sizeof(float), 1, fp) != 1)
                return false;

            // ���뷴��ǿ��
            if (fread(&nIntensity, sizeof(int), 1, fp) != 1)
                return false;
        }

        // ����ļ���ʽ��2.00������(����+ǿ�ȹ���3���ֽ�)
        else
        {
            unsigned short int ur = 0;
            unsigned char ui = 0;
            if (fread(&ur, sizeof(unsigned short int), 1, fp) != 1)
                return false;

            if (fread(&ui, sizeof(unsigned char), 1, fp) != 1)
                return false;

            r = ur * 2.0f;                       // �����ֱ���Ϊ2mm
            //FIXME:��ͬ�Ĵ��������ͣ��ڰ�ǿ��������ͳһ��Χʱ�����Եı���ϵ����һ����������Ҫ���ݴ��������ͽ���׼ȷ��������ǿ��ֵ����
            //p+fǿ�ȳ���10��hokuyoǿ�ȳ���50
            nIntensity = (int)ui * 10;                // ǿ�ȷ�Χ(0-255)
        }

        if (r < 0 || r > 65500)    // r > 65500Ϊ��ʱ��ʩ�����ɨ���߳���һȦ�뾶Ϊ65534�Ļ�������
            r = 0;

        distance.push_back(static_cast<unsigned int>(r));
        intensity.push_back(static_cast<unsigned int>(nIntensity));
    }

    return true;
}

//
//   ��ɨ�����ݱ��浽ʮ�����ļ���
//
bool CRawPointCloud::SaveBinary(FILE* fp, int nFileVersion)
{
    float r = 0;
    int inten = 0;
    for (unsigned int i = 0; i < num_points; i++)
    {
        r = static_cast<float>(distance[i]);
        inten = static_cast<int>(intensity[i]);

        // V2.00�汾��ʽ����
        if (nFileVersion < 200)
        {
            fwrite(&r, sizeof(float), 1, fp);
            fwrite(&inten, sizeof(int), 1, fp);
        }

        // V2.00���ϰ汾��ʽ
        else
        {
            unsigned short int ur = r / 2;                   // ������˫�ֽڱ�ʾ���ֱ���Ϊ2mm
            fwrite(&ur, sizeof(unsigned short int), 1, fp);

            unsigned char ui = (unsigned char)(inten / 10);  // ǿ���õ��ֽڱ�ʾ
            fwrite(&ui, sizeof(unsigned char), 1, fp);
        }
    }

    return true;
}


//
//   �Ӷ������ļ��ж�ȡɨ�����ݡ�
//
bool CRawScan::LoadBinary(FILE* fp, const CPosture& pstRobot, const CScannerGroupParam& ScannerParam, int nFileVersion)
{
    // �ֱ��ȡ������������ɨ������
    for (size_t i = 0; i < ScannerParam.size(); i++){
        auto pCloud = std::make_shared<sensor::CRawPointCloud>();
        if(!pCloud) {
            return false;
        }

        if (!pCloud->LoadBinary(fp, ScannerParam[i].m_fStartAngle, ScannerParam[i].m_fEndAngle,
                                ScannerParam[i].m_nLineCount, nFileVersion)){
            return false;
        }

        pCloud->laser_id = static_cast<short>(i);
        point_cloud.push_back(pCloud);
        pCloud.reset();
        pCloud = nullptr;
    }

    return true;
}

//
//   ��ɨ�����ݱ��浽ʮ�����ļ���
//
bool CRawScan::SaveBinary(FILE* fp, int nFileVersion)
{
    for (size_t i = 0; i < point_cloud.size(); i++){
        if(point_cloud[i] == NULL){
            continue;
        }
        if (!point_cloud[i]->SaveBinary(fp, nFileVersion)){
            return false;
        }
    }

    return true;
}




} // namespace sensor
