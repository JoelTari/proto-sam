#include "core/sam-system.h"
#include "factor_impl/anchor.hpp"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/linear-translation.hpp"
#include "utils/tuple_patterns.h"
#include "core/marginal.h"
#include <sstream>
#include <stdexcept>


template <typename... Ts>
struct cat_tuple_in_depth;
template <typename T>
struct cat_tuple_in_depth<T>
{
  using type = std::tuple<typename T::KeyMeta_t>;   // WARNING: weakness here: use macro ?
};
template <typename T, typename... Ts>
struct cat_tuple_in_depth<T, Ts...>
{
  using type = sam_tuples::tuple_cat_t<std::tuple<typename T::KeyMeta_t>,
                                          typename cat_tuple_in_depth<Ts...>::type>;
};
// extract tuple template argument specialisation
template <typename... Ts>
struct cat_tuple_in_depth<std::tuple<Ts...>> : cat_tuple_in_depth<Ts...>
{
};
template <typename... Ts, typename... Tss>
struct cat_tuple_in_depth<std::tuple<Ts...>, std::tuple<Tss...>> : cat_tuple_in_depth<Ts..., Tss...>
{
};
template <typename T>
struct cat_tuple_in_depth<std::tuple<T>> : cat_tuple_in_depth<T>
{
};


//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  // logger
  sam_utils::JSONLogger::Instance().beginSession("sam-system-factor_test.cpp");
  // scoped Timer
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  Json::Value rootJson;   // the desired 'container' (to be filled)
                          // initialized as null
  if (argc > 1)
  {
    // getting a filename as argument
    std::ifstream file(argv[1]);
    Json::Reader  reader;   // the parser
    reader.parse(file, rootJson);
  }
  else
  {
    // or receiving the json directly in std input
    // The parse is not necessary here
    std::cin >> rootJson;
  }

  // now we can read the data via:
  std::cout << "all the data:  " << rootJson << '\n';
  std::cout << "\n\n Declaring a sam system:\n";

  auto syst = SAM::SamSystem<AnchorFactor, LinearTranslationFactor>();
  int fcount = 0;
  for (const auto & mesJson : rootJson)
  {
    // std::cout << mesJson << '\n';
    if ( mesJson["type"] == "anchor")
    {
        Eigen::Vector2d z { mesJson["value"]["x"].asDouble(), mesJson["value"]["y"].asDouble()};
        // Eigen::Matrix2d Sigma;
        std::vector<double> cov_vect;
        for (const auto & c : mesJson["covariance"])
          cov_vect.push_back(c.asDouble());
        Eigen::Map<Eigen::Matrix2d> Sigma(cov_vect.data());
        std::stringstream fid; 
        fid << "f" << fcount;
        syst.register_new_factor<AnchorFactor>(fid.str(), z, Sigma, {mesJson["vars_id"][0].asCString()} );
        fcount++;
    }
    else if (mesJson["type"] == "linear-translation")
    {

        Eigen::Vector2d z { mesJson["value"]["dx"].asDouble(), mesJson["value"]["dy"].asDouble()};
        std::vector<double> cov_vect;
        for (const auto & c : mesJson["covariance"])
          cov_vect.push_back(c.asDouble());
        Eigen::Map<Eigen::Matrix2d> Sigma(cov_vect.data());
        std::stringstream fid; 
        fid << "f" << fcount;
        syst.register_new_factor<LinearTranslationFactor>(fid.str(), z, Sigma, {mesJson["vars_id"][0].asCString(),mesJson["vars_id"][1].asCString()} );
        fcount++;
    }
    else 
      throw std::runtime_error("measure type not supported");
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  try
  {
    syst.smooth_and_map();
  }
  catch (const char* e)
  {
#if ENABLE_DEBUG_TRACE
    std::cerr << "SLAM algorithm failed. Reason: " << e << '\n';
#endif
  }

  return 0;
}
