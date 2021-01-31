/*
 * LinuxSetting.h
 *
 *  Created on: 2018-6-7
 *      Author: sfe1012
 */

#ifndef LINUXSETTING_H
#define LINUXSETTING_H

#include"PlatformControl.h"

/******************Debug Fuction***********************************/
//#define     AGV_LINUX_DEBUG

#define   REMOTE_AGV_DEBUG

//#define   AGV_LINUX_DEBUG_TIMER  // 启用调试定时器，避免系统定时器信号中断
/****************************************************************/
//#define LINK_Pf_LASER //physics link the Pf Lase  use 倍加福 laser 传感器

#define MANUAL_DISABLE_PLS           //手动是否可以禁止PLS

/******************** Chose Platform *******************************************/
//#define _Borax_LINUX32    //Gsrd (Mrc 04) this box platform own fpga mcu, and have only two nomal can0

//#define _MRC_LINUX32        //Mrc 02 03

/*****************************Deal with special PLS (such as: 217888) ***************************************************/

//#define DEAL_WITH_SPECIAL_PLS

/******************** Chose Laguage *******************************************/
#define USING_CHINESE_LANGUAGE

//#define USING_ENGLISH_LANGUAGE

//#define USING_RUSSIAN_LANGUAGE

/********************************************************************************/

/********************************************************************************/

#define  USING_DGUS_OR_PROFACE_SCRENN  //使用迪文屏幕和Proface屏幕
//#define  USING_TTY_USB_SERIAL_DEBUG    //桌面调试仿真用
/*******************************************************************************/

#ifdef _X86_LINUX64
      #define WORK_PATH             "../Parameters/"
      #define  LOG_FILE_PATH        "../BlackBox/"
#else
    #ifdef AGV_LINUX_DEBUG_BALCK_BOX
        #ifdef  AGV_LINUX_DEBUG
                 #define  WORK_PATH               "/home/root/CarryBoy/"
                 #define  LOG_FILE_PATH           "/BlackBox/"
                 #define  LOG_TAR_PATH            "/"
                 #define  FTOK_KEY_PATH           "/home/root/CarryBoy/"
        #endif
    #else
        #ifdef  AGV_LINUX_DEBUG
            #ifndef GSRD_SIMULATE
                #define  WORK_PATH              "../../../Parameters/"
                #define  LOG_FILE_PATH          "../../../Log/BlackBox/"
                #define  LOG_TAR_PATH           "../../../Log/"
                #define  FTOK_KEY_PATH          "../../../"
            #else
                #define  WORK_PATH               "/home/root/CarryBoy/"
                #define  LOG_FILE_PATH           "/BlackBox/"
                #define  LOG_TAR_PATH            "/"
                #define  FTOK_KEY_PATH           "/home/root/CarryBoy/"
            #endif// end GSRD_SIMULATE
        #else
                 #define  WORK_PATH               "/home/root/CarryBoy/"
                 #define  LOG_FILE_PATH           "/BlackBox/"
                 #define  LOG_TAR_PATH            "/"
                 #define  FTOK_KEY_PATH           "/home/root/CarryBoy/"
        #endif // end AGV_LINUX_DEBUG
    #endif
#endif

/*************************NVBlackBox*****************************************/

#define   PLATFORM_WINDOWS_ARMV4I  //open for mrc borax32 nv ram

/*************** sfe1012 share mem BlackBox Function Seting  **********************/
#define  USING_SHARE_MEMORY_BLACK_BOX
#define  USING_FILE_MAPPING_BLACK_BOX
#define  USING_NVRAM_BLACK_BOX

#define  EVEN_BASE_ADDRESS_OFFSET      0
#define  NET_BASE_ADDRESS_OFFSET       512000
#define  CAN_BASE_ADDRESS_OFFSET       1024000           //512000*2
#define  NAV_BASE_ADDRESS_OFFSET       1536000          //512000*3
#define  LASER_BASE_ADDRESS_OFFSET     2048000         //512000*4
#define  CUSTOM_BASE_ADDRESS_OFFSET    2560000        //512000*5
#define  COM_BASE_ADDRESS_OFFSET       3072000       //512000*6

#define  SHARE_MEM_MAX_SIZE            3584000     //512000*7


#define BB_EVENT_ID              0
#define BB_NET_ID                1
#define BB_CAN_ID                2
#define BB_NAV_ID                3
#define BB_LASER_ID              4
#define BB_CUSTOM_ID             5

#define BLACK_BOX_SIZE           6

/*****************************************************************/

#define FORK_HANDLE

/*****************UI Show postion*********************/
#define UI_AX   0
#define UI_AY   0
#define UI_AW   800
//#define UI_AH   480
#define UI_AH   600
/****************************************************/

#endif // LINUXSETTING_H
