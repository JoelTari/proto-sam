#pragma once

#include <ratio>
#include <sstream>
// #include <fstream>
// #include "utils/config.h"

#include <json/value.h>
#include <json/json.h>
#include <mutex>
#include <thread>

#ifndef ENABLE_JSON_OUTPUT
#define ENABLE_JSON_OUTPUT 1
#endif

#if ENABLE_TIMER > 0
#include <chrono>
#define PROFILE_SCOPE(name, logger) sam_utils::ScopedTimer timer##__LINE__((name), logger)
#define PROFILE_FUNCTION()    PROFILE_SCOPE(__FUNCTION__, sam_utils::JSONLogger::Instance())
#else
#define PROFILE_SCOPE(name, logger)   // nada
#define PROFILE_FUNCTION(logger)      // nada
#endif


namespace sam_utils
{

  // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
  inline const std::string currentDateTime()
  {
    time_t    now = time(0);
    struct tm tstruct;
    char      buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y%m%d_%H-%M-%S", &tstruct);

    return buf;
  }

#if ENABLE_JSON_OUTPUT
  class JSONLogger
  {
    std::string m_program_name = "None";
    std::string m_pb_id;
    std::string m_launch_time_point;
    uint        m_spanCount = 0;
    std::mutex  m_lock;
    // bool            m_activeSession = false;
    Json::Value m_JsonRoot;   // init as null

    // JSONLogger() { }

    public:
    // std::stringstream   m_outputStream;

    static JSONLogger& Instance()
    {
      static JSONLogger instance;
      return instance;
    }

    std::string out()
    {
      std::stringstream ss;
      ss << m_JsonRoot;
      return ss.str();
    }

    void beginSession(const std::string& program_name,
                      const std::string& pb_id,
                      const std::string& time_point = sam_utils::currentDateTime())
    {
      std::lock_guard<std::mutex> lock(m_lock);
      // if session already, start a new session
      // if (m_activeSession) { endSession(); }

      // m_activeSession = true;
      // m_outputStream = ss;
      // writeHeader();
      m_program_name      = program_name;
      m_pb_id             = pb_id;
      m_launch_time_point = time_point;
    }

#if ENABLE_TIMER
    /**
     * @brief return a spanIdx so that when the duration information can be added later in this same
     * span (other span might be added to the list in the interval)
     *
     * @param name
     * @param threadID
     * @param start
     *
     * @return  spanIdx
     */
    uint writeInitProfile(const std::string& name, const uint32_t threadID, const long long start)
    {
      std::lock_guard<std::mutex> lock(m_lock);

      Json::Value span;
      span["cat"]  = "function";
      span["name"] = name;
      span["ph"]   = "X";
      span["pid"]  = m_program_name;
      span["tid"]  = threadID;
      span["ts"]   = std::to_string(start);

      m_JsonRoot["traceEvents"].append(span);

      return m_spanCount++;
    }

    void writeEndProfile(const long long duration, const int spanIdx)
    {
      std::lock_guard<std::mutex> lock(m_lock);
      m_JsonRoot["traceEvents"][spanIdx]["dur"] = std::to_string(duration);
      // if(m_JsonRoot["traceEvents"][spanIdx]["name"] == "main")
      //   std::cout << duration << '\n';
    }
#endif

    void writeGraph(const Json::Value& graph)
    {
      // note: this also erases previous content of jsonRoot
      std::lock_guard<std::mutex> lock(m_lock);
      m_JsonRoot["header"]    = graph["header"];
      m_JsonRoot["factors"]   = graph["factors"];
      m_JsonRoot["marginals"] = graph["marginals"];
    }
  };
#endif

#if ENABLE_TIMER
  //------------------------------------------------------------------//
  //                           Scope Timer                            //
  //------------------------------------------------------------------//
  /**
   * @brief ScopedTimer is a timer that destroys itself at the end of
   * scope and writes its value to the json logger
   *
   * @tparam TIME_UNIT an std::chrono::duration<{{representation}},{{period}}>
   *         where representation could be int64, int32 , float etc...
   *         Period is a ratio from the second unit: milli is std::ratio<1,1000>
   *         minute would be std::ratio<60> (which default to std::ratio<60,1>).
   *         Default is microseconds because that's the unit the current tracing UI expects
   *         without addtional config
   */
  template <typename TIME_UNIT = std::chrono::microseconds>
  struct ScopedTimer
  {
    /**
     * @brief name of the timer (recommended to relate to the function)
     */
    const char* name;
    // all this verbose trouble to get the start time point in terms of microseconds, not
    // nanoseconds. However, it would still be possible to register the steady clocks time point in
    // nanosecons (as std::chrono::time_point<std::chrono::steady_clock>, indepent of our TIME_UNIT
    // ) to get the most accurate time_point difference duration. But if the duration is casted then
    // as microseconds, don't forget to also 'downgrade' to microseconds the timepoint The main
    // point is that the time point and the duration should have the same time unit when they enter
    // the logs
    std::chrono::time_point<std::chrono::steady_clock, TIME_UNIT> start
        = std::chrono::time_point_cast<TIME_UNIT>(std::chrono::steady_clock::now());

    // std::chrono::time_point<std::chrono::steady_clock> end; // no need for end to be a member
    JSONLogger& logger; // FIX: remove
    int         spanIdx = 0;

    ScopedTimer(const char* name, JSONLogger& logger) : name(name), logger(logger)
    {
      uint32_t threadID = std::hash<std::thread::id> {}(std::this_thread::get_id());
      spanIdx           = logger.writeInitProfile(name, threadID, start.time_since_epoch().count());
    };

    ~ScopedTimer()
    {
      // end = std::chrono::steady_clock::now();  // same remark as above on verbosity
      auto end = std::chrono::time_point_cast<TIME_UNIT>(std::chrono::steady_clock::now());

      // if constexpr( std::is_same_v<typename TIME_UNIT::period, std::nano> )
      auto duration = end - start;
      // else
      //    auto duration = std::chrono::duration_cast<TIME_UNIT>(end - start);

      logger.writeEndProfile(duration.count(), spanIdx);

#if ENABLE_DEBUG_TRACE && 0   //
      // add a cout or an I/O here
      std::cout << "Timer for " << name << " : " << duration.count() << get_unit_str() << "\n";
#endif
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
    || std::is_same_v<typename TIME_UNIT::period, std::ratio<60>>  // ratio<60> is same as ratio<60,1>
    || std::is_same_v<typename TIME_UNIT::period, std::ratio<3600>>
    , "Unknown time units");

    // TODO: try that with the logger  'displayTimeUnit': 'ms' (sibling of 'traceEvents' in JSON)
    static constexpr const char* get_unit_str()
    {
      if constexpr (std::is_same_v<typename TIME_UNIT::period, std::ratio<3600>>)   // hours
        return "h";
      else if constexpr (std::is_same_v<typename TIME_UNIT::period, std::ratio<60>>)   // minutes
        return "min";
      else if constexpr (std::is_same_v<typename TIME_UNIT::period, std::ratio<1>>)   // seconds
        return "s";
      else if constexpr (std::is_same_v<typename TIME_UNIT::period,
                                        std::milli>)   // milli is alias to ratio<1,1000> in ratio.h
        return "ms";
      else if constexpr (std::is_same_v<typename TIME_UNIT::period,
                                        std::micro>)   // micro is alias to ratio<1,1000000>
        return "us";
      else if constexpr (std::is_same_v<typename TIME_UNIT::period,
                                        std::nano>)   // nano is alias to ratio<1,1000000000>
        return "ns";
    }
  };


#endif
}   // namespace sam_utils
