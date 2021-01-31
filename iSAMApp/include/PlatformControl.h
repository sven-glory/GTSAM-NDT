#ifndef PLATFORMCONTROL_H
#define PLATFORMCONTROL_H

    #ifdef __linux__
        #define LINUX_PLATFORM_USING
    #else
        #define WINDOWS_PLATFORM_USING
    #endif

    #ifdef WINDOWS_PLATFORM_USING
        #define SIASUN_LIB_DECL_EXPORT     __declspec(dllexport)
        #define SIASUN_LIB_DECL_IMPORT     __declspec(dllimport)
        #define SIASUN_PTHREAD_PROC_DECL   unsigned int  //UNIT  pthread pro funtion
        #define SiaSun_AfxBeginThread      AfxBeginThread
        #define SIASUN_PTHREAD_T           unsigned int
    #elif defined(LINUX_PLATFORM_USING)
        #define SIASUN_LIB_DECL_EXPORT     __attribute__((visibility("default")))
        #define SIASUN_LIB_DECL_IMPORT     __attribute__((visibility("default")))
        #define SIASUN_LIB_DECL_HIDDEN     __attribute__((visibility("hidden")))
        #define SIASUN_PTHREAD_PROC_DECL        void*  //UNIT
        #define SIASUN_PTHREAD_T                pthread_t
        #include <QtCore/qglobal.h>
        #define DllExport   SIASUN_LIB_DECL_EXPORT
        #define DllImport   SIASUN_LIB_DECL_IMPORT
        /*************************Set pthrea priority*******************************************/
        #ifndef PTHREAD_PRIORITY
            #define THREAD_PRIORITY_LOWEST          1
            #define THREAD_PRIORITY_BELOW_NORMAL    (THREAD_PRIORITY_LOWEST+1)
            #define THREAD_PRIORITY_NORMAL          96
            #define THREAD_PRIORITY_HIGHEST         99
            #define THREAD_PRIORITY_ABOVE_NORMAL    (THREAD_PRIORITY_HIGHEST-1)
        #endif
        /*************************************************************************************/
        //1��SCHED_OTHER ��ʱ���Ȳ��ԣ�
        //2��SCHED_FIFOʵʱ���Ȳ��ԣ��ȵ��ȷ���һ��ռ��cpu��һֱ���С�һֱ����ֱ���и������ȼ����񵽴���Լ�����
        //3��SCHED_RRʵʱ���Ȳ��ԣ�ʱ��Ƭ��ת�������̵�ʱ��Ƭ���꣬ϵͳ�����·���ʱ��Ƭ�������ھ�������β�����ڶ���β��֤�����о�����ͬ���ȼ���RR����ĵ��ȹ�ƽ
        //SCHED_OTHER�ǲ�֧�����ȼ�ʹ�õģ���SCHED_FIFO��SCHED_RR֧�����ȼ���ʹ�ã����Ƿֱ�Ϊ1��99����ֵԽ�����ȼ�Խ�ߡ�
    #endif

#endif // PLATFORMCONTROL_H

