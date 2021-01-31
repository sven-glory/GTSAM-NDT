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
//   从二进制文件中读取扫描数据。
//
bool CRawPointCloud::LoadBinary(FILE* fp, float fStartAngle, float fEndAngle, int nLineCount,
    int nFileVersion)
{
    unsigned long dis_size = static_cast<unsigned long>(nLineCount + 1); // 每次扫描为1080间隔，点数为1081
    unsigned long inten_size = static_cast<unsigned long>(nLineCount + 1);

    num_points = static_cast<unsigned int>(nLineCount);
    //laser_type = m_uType; //数据集中无传感器类型参数
    start_angle = fStartAngle;
    end_angle = fEndAngle;
    distance.clear();
    intensity.clear();
    distance.reserve(dis_size);
    intensity.reserve(inten_size);

    // 计算扫描的角分辨率
    float fAngReso = (fEndAngle - fStartAngle) / nLineCount;

    // 如果版本在V2.10以上，在此需要读入时间戳
    unsigned int uTimeStamp = 0;
    if (nFileVersion >= 210)
    {
        if (fread(&uTimeStamp, sizeof(unsigned int), 1, fp) != 1)
            return false;

        timestamp_raw = uTimeStamp;
        timestamp_sync = uTimeStamp;
    }

    // 依次读入所有点的数据
    for (unsigned int i = 0; i < num_points; i++)
    {
        float r = 0;
        int nIntensity = 0;

        // 如果文件格式版本不超过2.00版(极径+强度共需8个字节)
        if (nFileVersion < 200)
        {
            // 读入极径
            if (fread(&r, sizeof(float), 1, fp) != 1)
                return false;

            // 读入反光强度
            if (fread(&nIntensity, sizeof(int), 1, fp) != 1)
                return false;
        }

        // 如果文件格式在2.00版以上(极径+强度共需3个字节)
        else
        {
            unsigned short int ur = 0;
            unsigned char ui = 0;
            if (fread(&ur, sizeof(unsigned short int), 1, fp) != 1)
                return false;

            if (fread(&ui, sizeof(unsigned char), 1, fp) != 1)
                return false;

            r = ur * 2.0f;                       // 极径分辨率为2mm
            //FIXME:不同的传感器类型，在把强度整定成统一范围时，除以的比例系数不一样，所以需要根据传感器类型进行准确处理，否则强度值不对
            //p+f强度乘以10，hokuyo强度乘以50
            nIntensity = (int)ui * 10;                // 强度范围(0-255)
        }

        if (r < 0 || r > 65500)    // r > 65500为临时措施，解决扫描线出现一圈半径为65534的弧的问题
            r = 0;

        distance.push_back(static_cast<unsigned int>(r));
        intensity.push_back(static_cast<unsigned int>(nIntensity));
    }

    return true;
}

//
//   将扫描数据保存到十进制文件。
//
bool CRawPointCloud::SaveBinary(FILE* fp, int nFileVersion)
{
    float r = 0;
    int inten = 0;
    for (unsigned int i = 0; i < num_points; i++)
    {
        r = static_cast<float>(distance[i]);
        inten = static_cast<int>(intensity[i]);

        // V2.00版本格式以下
        if (nFileVersion < 200)
        {
            fwrite(&r, sizeof(float), 1, fp);
            fwrite(&inten, sizeof(int), 1, fp);
        }

        // V2.00以上版本格式
        else
        {
            unsigned short int ur = r / 2;                   // 极径用双字节表示，分辨率为2mm
            fwrite(&ur, sizeof(unsigned short int), 1, fp);

            unsigned char ui = (unsigned char)(inten / 10);  // 强度用单字节表示
            fwrite(&ui, sizeof(unsigned char), 1, fp);
        }
    }

    return true;
}


//
//   从二进制文件中读取扫描数据。
//
bool CRawScan::LoadBinary(FILE* fp, const CPosture& pstRobot, const CScannerGroupParam& ScannerParam, int nFileVersion)
{
    // 分别读取各个激光器的扫描数据
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
//   将扫描数据保存到十进制文件。
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
