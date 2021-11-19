#ifndef SAM_UTILS_H_
#define SAM_UTILS_H_

#include <chrono>
#include <json/value.h>
#include <ratio>
#include <sstream>
#include <fstream>
#include <mutex>
#include <thread>
#include <json/json.h>

#include "config.h"

namespace sam_utils
{

class JSONLogger
{
    std::string     m_sessionName   = "None";
    std::ofstream   m_outputStream;
    uint            m_spanCount=0;
    std::mutex      m_lock;
    bool            m_activeSession = false;
    Json::Value     m_JsonRoot; // init as null
 
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
       // if session already, start a new session
        if (m_activeSession) { endSession(); }
        
        m_activeSession = true;
        m_outputStream.open(filepath);
        // writeHeader();
        m_sessionName = name;
    }
 
    void endSession()
    {
        if (!m_activeSession) { return; }
        m_activeSession = false;
        m_outputStream << m_JsonRoot;
        m_outputStream.close();
        // m_profileCount = 0;
    }

    /**
    * @brief return a spanIdx so that when the duration information can be added later in this same span (other span might be added to the list in the interval)
    *
    * @param name
    * @param threadID
    * @param start
    *
    * @return  spanIdx
    */
    uint writeInitProfile(const std::string &name,const uint32_t threadID,const long long start)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        Json::Value span; 
        span["cat"]="function";
        span["name"]=name;
        span["ph"]="X";
        span["pid"]=m_sessionName;
        span["tid"]=threadID;
        span["ts"]= std::to_string(start);

        m_JsonRoot["traceEvents"].append(span);
 
        return m_spanCount++;
    }
 
    void writeEndProfile(const long long duration,const int spanIdx)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        m_JsonRoot["traceEvents"][spanIdx]["dur"]=std::to_string(duration);
        // if(m_JsonRoot["traceEvents"][spanIdx]["name"] == "main")
        //   std::cout << duration << '\n';
    }

    void writeGraph(const Json::Value & graph)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        m_JsonRoot["graph"] = graph;
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
*         minute would be std::ratio<60> (which default to std::ratio<60,1>).
*         Default is microseconds because that's the unit the current tracing UI expects
*         without addtional config
*/
template<typename TIME_UNIT = std::chrono::microseconds>
struct ScopedTimer
{

  /**
  * @brief name of the timer (recommended to relate to the function)
  */
  const char*                                        name;
  // all this verbose trouble to get the start time point in terms of microseconds, not nanoseconds.
  // However, it would still be possible to register the steady clocks time point in nanosecons 
  // (as std::chrono::time_point<std::chrono::steady_clock>, indepent of our TIME_UNIT ) to get the most
  // accurate time_point difference duration.
  // But if the duration is casted then as microseconds, don't forget to also 'downgrade' to microseconds the timepoint
  // The main point is that the time point and the duration should have the same time unit when they enter the logs
  std::chrono::time_point<std::chrono::steady_clock,TIME_UNIT> start
      = std::chrono::time_point_cast<TIME_UNIT>(std::chrono::steady_clock::now());

  // std::chrono::time_point<std::chrono::steady_clock> end; // no need for end to be a member
  JSONLogger & logger;
  int spanIdx=0;

  ScopedTimer(const char* name, JSONLogger& logger) : name(name),logger(logger)
  {
    uint32_t threadID = std::hash<std::thread::id>{}(std::this_thread::get_id());
    spanIdx=logger.writeInitProfile(name,threadID,start.time_since_epoch().count());
  };

  ~ScopedTimer()
  {
    // end = std::chrono::steady_clock::now();  // same remark as above on verbosity
    auto end = std::chrono::time_point_cast<TIME_UNIT>(std::chrono::steady_clock::now());

    // if constexpr( std::is_same_v<typename TIME_UNIT::period, std::nano> )
    auto duration = end-start;
    // else
    //    auto duration = std::chrono::duration_cast<TIME_UNIT>(end - start);

    logger.writeEndProfile(duration.count(),spanIdx);

#if ENABLE_DEBUG_TRACE
    // add a cout or an I/O here
    std::cout << "Timer for " << name << " : " << duration.count()
              << get_unit_str() <<"\n"; 
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


//------------------------------------------------------------------//
//                            TUPLE LOOP                            //
//------------------------------------------------------------------//
namespace detail
{
    template<typename T, typename F, std::size_t... Is>
    void
    for_each(T&& t, F f, std::index_sequence<Is...>)
    {
        auto l = { (f(std::get<Is>(t),Is), 0)... };
    }
} // namespace detail

template<typename... Ts, typename F>
void for_each_in_tuple(std::tuple<Ts...> const& t, F f)
{
    detail::for_each(t, f, std::make_index_sequence<sizeof...(Ts)>{});
}

#endif
