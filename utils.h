#ifndef SAM_UTILS_H_
#define SAM_UTILS_H_

#include "config.h"
#include <chrono>
#include <ratio>
#include <sstream>
#include <fstream>
#include <mutex>

namespace sam_utils
{

struct ProfileResult
{
    std::string name;
    long long start;
    long long duration;
    uint32_t threadID;
};
 

class JSONLogger
{
    std::string     m_sessionName   = "None";
    std::ofstream   m_outputStream;
    int             m_profileCount  = 0;
    std::mutex      m_lock;
    bool            m_activeSession = false;
 
    JSONLogger() { }
 
public:
 
    static JSONLogger& Instance()
    {
        static JSONLogger instance;
        return instance;
    }
 
    ~JSONLogger()
    {
        endSession();
    }
 
    void beginSession(const std::string& name, const std::string& filepath = "results.json")
    {
        if (m_activeSession) { endSession(); }
        m_activeSession = true;
        m_outputStream.open(filepath);
        writeHeader();
        m_sessionName = name;
    }
 
    void endSession()
    {
        if (!m_activeSession) { return; }
        m_activeSession = false;
        writeFooter();
        m_outputStream.close();
        m_profileCount = 0;
    }
 
    void writeProfile(const ProfileResult& result)
    {
        std::lock_guard<std::mutex> lock(m_lock);
 
        if (m_profileCount++ > 0) { m_outputStream << ","; }
 
        std::string name = result.name;
        // std::replace(name.begin(), name.end(), '"', '\'');
 
        m_outputStream << "{";
        m_outputStream << "\"cat\":\"function\",";
        m_outputStream << "\"dur\":" << result.duration << ',';
        m_outputStream << "\"name\":\"" << name << "\",";
        m_outputStream << "\"ph\":\"X\",";
        m_outputStream << "\"pid\":\"SAM\",";
        m_outputStream << "\"tid\":" << result.threadID << ",";
        m_outputStream << "\"ts\":" << result.start<< ",";
        m_outputStream << "\"someBS\":" << "\"helloworld\"";
        m_outputStream << "}";
    }
 
    void writeHeader()
    {
        m_outputStream << "{\"otherData\": {},\"traceEvents\":[";
    }
 
    void writeFooter()
    {
        m_outputStream << "]";
        // m_outputStream << "\",displayTimeUnit\": \"ms\"";
        m_outputStream << "}";
    }
};

#if ENABLE_TIMER
//------------------------------------------------------------------//
//                           Scope Timer                            //
//------------------------------------------------------------------//
/**
* @brief 
*
* @tparam TIME_UNIT an std::chrono::duration<{{representation}},{{period}}>
*         where representation could be int64, int32 , float etc...
*         Period is a ratio from the second unit: milli is std::ratio<1,1000>
*         minute would be std::ratio<60> (which default to std::ratio<60,1>)
*/
template<typename TIME_UNIT = std::chrono::microseconds>
struct ScopedTimer
{

  /**
  * @brief name of the timer (recommended to relate to the function)
  */
  const char*                                        name;
  std::chrono::time_point<std::chrono::steady_clock> start
      = std::chrono::steady_clock::now();

  std::chrono::time_point<std::chrono::steady_clock> end;

  ScopedTimer(const char* name) : name(name) {};

  ~ScopedTimer()
  {
    end = std::chrono::steady_clock::now();

    // computing the duration. With nanoseconds, no need to cast
    TIME_UNIT duration;
    if constexpr( std::is_same_v<typename TIME_UNIT::period, std::nano> )
       duration = end-start;
    else
       duration = std::chrono::duration_cast<TIME_UNIT>(end - start);

    // add a cout or an I/O here
    std::cout << "Timer for " << name << " : " << duration.count()
              << get_unit_str() <<"\n"; 
  }

  // NOTE: these are the same thing:
  // NOTE:         - std::chrono::milliseconds::period::type
  // NOTE:         - std::milli
  // NOTE:         - std::ratio<1,1000>

  // compile time assert: support for the time period specified
  static_assert(
       std::is_same_v<typename TIME_UNIT::period, std::chrono::nanoseconds::period::type>  // std::chrono::milliseconds::period is same as std::milli is same as 
    || std::is_same_v<typename TIME_UNIT::period, std::chrono::microseconds::period::type>
    || std::is_same_v<typename TIME_UNIT::period, std::chrono::milliseconds::period::type>
    || std::is_same_v<typename TIME_UNIT::period, std::chrono::seconds::period::type>
    || std::is_same_v<typename TIME_UNIT::period, std::ratio<60>>
    || std::is_same_v<typename TIME_UNIT::period, std::ratio<3600>>
    , "Unknown time units");

  static constexpr const char* get_unit_str (){
    if constexpr (std::is_same_v<typename TIME_UNIT::period, std::ratio<3600>>) // hours
      return "h";
    else if constexpr (std::is_same_v<typename TIME_UNIT::period, std::ratio<60> >) // minutes
      return "min";
    else if constexpr (std::is_same_v<typename TIME_UNIT::period, std::ratio<1> >)   // seconds
      return "s";
    else if constexpr (std::is_same_v<typename TIME_UNIT::period, std::milli>)  // milli is alias to ratio<1,1000> in ratio.h
      return "ms";
    else if constexpr (std::is_same_v<typename TIME_UNIT::period, std::micro>)  // micro is alias to ratio<1,1000000>
      return "us";
    else if constexpr (std::is_same_v<typename TIME_UNIT::period, std::nano>) // nano is alias to ratio<1,1000000000>
      return "ns";
    }
};


#endif
}

#endif
