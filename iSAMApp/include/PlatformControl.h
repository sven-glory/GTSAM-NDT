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
        //1，SCHED_OTHER 分时调度策略，
        //2，SCHED_FIFO实时调度策略，先到先服务。一旦占用cpu则一直运行。一直运行直到有更高优先级任务到达或自己放弃
        //3，SCHED_RR实时调度策略，时间片轮转。当进程的时间片用完，系统将重新分配时间片，并置于就绪队列尾。放在队列尾保证了所有具有相同优先级的RR任务的调度公平
        //SCHED_OTHER是不支持优先级使用的，而SCHED_FIFO和SCHED_RR支持优先级的使用，他们分别为1和99，数值越大优先级越高。
    #endif

#endif // PLATFORMCONTROL_H

