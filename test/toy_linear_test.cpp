#include "core/sam-system.h"
#include "factor_impl/anchor.hpp"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/linear-translation.hpp"
#include "utils/tuple_patterns.h"
#include "core/marginal.h"


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
  std::string result_filename
      = sam_utils::currentDateTime() + "_results_toy_linear_test.json";
  sam_utils::JSONLogger::Instance().beginSession("toy_linear_test.cpp", result_filename);

  // scoped Timer
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  std::cout << "\n\n Declaring a sam system:\n";

  auto syst = SAM::SamSystem<AnchorFactor, LinearTranslationFactor>("A");
  // test cat_tuple_in_depth
  using aggrkeymeta_t
      = cat_tuple_in_depth<AnchorFactor::KeysSet_t, LinearTranslationFactor::KeysSet_t>::type;

  std::cout << "number of element in the tuple aggrkeymeta_t (exp. 3) : "
            << std::tuple_size<aggrkeymeta_t>::value << '\n';
  std::cout << "number of unique element in the tuple aggrkeymeta_t (exp. 1) : "
            << sam_tuples::tuple_filter_duplicate<aggrkeymeta_t>::size << '\n';

  using uniq_keymeta_set_t = sam_tuples::tuple_filter_duplicate<aggrkeymeta_t>::type ;

  using marginals_t = MarginalsContainer<uniq_keymeta_set_t> ;
  

  AnchorFactor::measure_vect_t z {0, 0};
  AnchorFactor::measure_cov_t      Sigma {{0.2, 0}, {0, 0.2}};

  syst.register_new_factor<AnchorFactor>("f0", z, Sigma, {"x0"});
  syst.register_new_factor<LinearTranslationFactor>(
      "f1",
      LinearTranslationFactor::measure_vect_t {-0.95, 0.1},
      LinearTranslationFactor::measure_cov_t {{0.1, 0}, {0, 0.1}},
      {"x0", "x1"});

  syst.register_new_factor<LinearTranslationFactor>(
      "f2",
      LinearTranslationFactor::measure_vect_t {-0.01654, -1.21},
      LinearTranslationFactor::measure_cov_t {{0.02, 0}, {0, 0.3}},
      {"x1", "x2"});

  syst.register_new_factor<LinearTranslationFactor>(
      "f3",
      LinearTranslationFactor::measure_vect_t {1.01654, -.11},
      LinearTranslationFactor::measure_cov_t {{0.32, 0}, {0, 0.1}},
      {"x2", "x3"});

  // loop-closure
  syst.register_new_factor<LinearTranslationFactor>(
      "f4",
      LinearTranslationFactor::measure_vect_t {0.01654, 1.181},
      LinearTranslationFactor::measure_cov_t {{0.002, 0}, {0, 0.173}},
      {"x3", "x0"});
  syst.register_new_factor<LinearTranslationFactor>(
      "f5",
      LinearTranslationFactor::measure_vect_t {-1.01654, -0.8},
      LinearTranslationFactor::measure_cov_t {{0.2, 0}, {0, 0.17}},
      {"x0", "x2"});

  // std::this_thread::sleep_for(std::chrono::seconds(1));

  try
  {
    syst.sam_optimize();
  }
  catch (const char* e)
  {
#if ENABLE_DEBUG_TRACE
    std::cerr << "SLAM algorithm failed. Reason: " << e << '\n';
#endif
  }

  return 0;
}
