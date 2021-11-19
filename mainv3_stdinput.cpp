#include "anchor.hpp"
#include "linear-translation.hpp"
#include "sam-systemV3.h"

//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  // logger
  sam_utils::JSONLogger::Instance().beginSession("sam-system-factor_test.cpp");
  // scoped Timer
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  std::cout << "\n\n Declaring a sam system:\n";
  auto samsyst = SAM::SamSystem<AnchorFactor, LinearTranslationFactor>();

  auto syst = SAM::SamSystem<AnchorFactor, LinearTranslationFactor>();

  // receive the measurement from stdin (as a string that can be converted in a
  // C++ container)

  AnchorFactor::measure_vect_t            z {0, 0};
  AnchorFactor::measure_cov_t Sigma {{0.2, 0}, {0, 0.2}};

  syst.register_new_factor<AnchorFactor>("f0", z, Sigma, {"x0"});
  syst.register_new_factor<LinearTranslationFactor>(
      "f1",
      LinearTranslationFactor::measure_vect_t {-0.95, 0.1},
      LinearTranslationFactor::measure_cov_t {{0.1, 0},
                                                            {0, 0.1}},
      {"x0", "x1"});

  syst.register_new_factor<LinearTranslationFactor>(
      "f2",
      LinearTranslationFactor::measure_vect_t {-0.01654, -1.21},
      LinearTranslationFactor::measure_cov_t {{0.02, 0},
                                                            {0, 0.3}},
      {"x1", "x2"});

  syst.register_new_factor<LinearTranslationFactor>(
      "f3",
      LinearTranslationFactor::measure_vect_t {1.01654, -.11},
      LinearTranslationFactor::measure_cov_t {{0.32, 0},
                                                            {0, 0.1}},
      {"x2", "x3"});

  // loop-closure
  syst.register_new_factor<LinearTranslationFactor>(
      "f4",
      LinearTranslationFactor::measure_vect_t {0.01654, 1.181},
      LinearTranslationFactor::measure_cov_t {{0.002, 0},
                                                            {0, 0.173}},
      {"x3", "x0"});
  syst.register_new_factor<LinearTranslationFactor>(
      "f5",
      LinearTranslationFactor::measure_vect_t {-1.01654, -0.8},
      LinearTranslationFactor::measure_cov_t {{0.2, 0},
                                                            {0, 0.17}},
      {"x0", "x2"});

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
