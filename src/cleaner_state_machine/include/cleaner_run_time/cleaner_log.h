#ifndef CLEANER_STATE_MACHINE_LOG_H_
#define CLEANER_STATE_MACHINE_LOG_H_
#include <ros/ros.h>
 #include <glog/logging.h>
// Here we define our application severity levels.  
enum severity_level  
{  
    normal,  
    notification,  
    warning,  
    error,  
    critical  
};  

//#define USE_GOOGLE_LOG

#define CLOG_INFO(...) cleaner_state_machine::CleanerLog::LOG_INFO(__VA_ARGS__)
#define CLOG_WARN(...) cleaner_state_machine::CleanerLog::LOG_WARN(__VA_ARGS__)
namespace cleaner_state_machine{
    /**
    * @class CleanerLog
    * @brief Provides a class for recording log with several severity levels
    */
    class CleanerLog{
    public:
        CleanerLog();
        ~CleanerLog();
        static void LOG_INFO(const char* fmt, ...)
        {
            va_list v;  
            va_start(v, fmt);  
            ROS_INFO(fmt,v);  
            va_end(v); 
            
            if(use_google_log_)
             {
                 LOG(INFO)<<fmt;
             }

        }
        static void LOG_WARN(const char* fmt, ...)
        {
            va_list v;  
            va_start(v, fmt);  
            ROS_WARN(fmt,v);  
            va_end(v); 
            
            if(use_google_log_)
             {
                 LOG(WARNING)<<fmt;
             }

        }
        static void EnableGlog(){use_google_log_ = true;}
        static void DisableGlog(){use_google_log_ = false;}
        static bool IsGlogEnable(){return use_google_log_ ;}
    private:
        static bool use_google_log_;
    };

}   

#endif 
