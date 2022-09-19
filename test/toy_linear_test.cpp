// #define ENABLE_DEBUG_TRACE 1

#include "anchor2d/anchor2d.h"
#include "relative-matcher-2d/relative-matcher-2d.h"
#include "system/sam-system.h"
#include "test_utils.h"

#include <gtest/gtest.h>

//------------------------------------------------------------------//
//                          SetUp Fixture                           //
//------------------------------------------------------------------//
class ToyLinear2dSystem : public ::testing::Test
{
  protected:
  // called N times (one each before each TEST_F() )
  void SetUp() override
  {
    sam_utils::JSONLogger::Instance().beginSession("gtest_2d", "toy_square_linear_2d");

    // Ground truth:
    // x0: 0, 0
    // x1: 1, 0
    // x2: 1, 1
    // x3: 0, 1
    //
    // x3  ─────── x2
    //  │          │
    //  │          │
    //  │          │
    // x0 ──────── x1
    //  ┼

    // anchor
    ::sam::Factor::Anchor2d::measure_t     z {0, 0};
    ::sam::Factor::Anchor2d::measure_cov_t Sigma {{0.002, 0}, {0, 0.002}};
    this->AnchorBundle.vfid.push_back("f0");
    this->AnchorBundle.vmeasure.push_back(z);
    this->AnchorBundle.vmeasure_cov.push_back(Sigma);
    this->AnchorBundle.vkeys_id.push_back({"x0"});

    // relative matcher
    sam::Factor::RelativeMatcher2d::measure_t     zrm;
    sam::Factor::RelativeMatcher2d::measure_cov_t Sigmarm;
    //   f1
    zrm     = {-0.95, 0.1};
    Sigmarm = sam::Factor::RelativeMatcher2d::measure_cov_t {{0.8, 0}, {0, 0.5}};
    this->RelativeMatcherBundle.vfid.push_back("f1");
    this->RelativeMatcherBundle.vmeasure.push_back(zrm);
    this->RelativeMatcherBundle.vmeasure_cov.push_back(Sigmarm);
    this->RelativeMatcherBundle.vkeys_id.push_back({"x0", "x1"});
    //   f2
    zrm     = {-0.01654, -1.21};
    Sigmarm = sam::Factor::RelativeMatcher2d::measure_cov_t {{0.5, 0}, {0, 0.8}};
    this->RelativeMatcherBundle.vfid.push_back("f2");
    this->RelativeMatcherBundle.vmeasure.push_back(zrm);
    this->RelativeMatcherBundle.vmeasure_cov.push_back(Sigmarm);
    this->RelativeMatcherBundle.vkeys_id.push_back({"x1", "x2"});
    //   f3
    zrm     = {1.01654, -.11};
    Sigmarm = sam::Factor::RelativeMatcher2d::measure_cov_t {{0.8, 0}, {0, 0.5}};
    this->RelativeMatcherBundle.vfid.push_back("f3");
    this->RelativeMatcherBundle.vmeasure.push_back(zrm);
    this->RelativeMatcherBundle.vmeasure_cov.push_back(Sigmarm);
    this->RelativeMatcherBundle.vkeys_id.push_back({"x2", "x3"});
    //   f4
    zrm     = {0.0, 1};
    Sigmarm = sam::Factor::RelativeMatcher2d::measure_cov_t {{0.002, 0}, {0, 0.002}};
    this->RelativeMatcherBundle.vfid.push_back("f4");
    this->RelativeMatcherBundle.vmeasure.push_back(zrm);
    this->RelativeMatcherBundle.vmeasure_cov.push_back(Sigmarm);
    this->RelativeMatcherBundle.vkeys_id.push_back({"x3", "x0"});
  }


  PreRegistrationBundle<sam::Factor::Anchor2d>          AnchorBundle;
  PreRegistrationBundle<sam::Factor::RelativeMatcher2d> RelativeMatcherBundle;

  // void TearDown() override

  template <typename SYS_T>
  void process_tests(SYS_T& syst)
  {
    // register bundle anchor
    syst.template register_factors_in_bundle<sam::Factor::Anchor2d>(AnchorBundle.vfid,
                                                                    AnchorBundle.vmeasure,
                                                                    AnchorBundle.vmeasure_cov,
                                                                    AnchorBundle.vkeys_id);
    // register bundle RelativeMatcher
    syst.template register_factors_in_bundle<sam::Factor::RelativeMatcher2d>(
        RelativeMatcherBundle.vfid,
        RelativeMatcherBundle.vmeasure,
        RelativeMatcherBundle.vmeasure_cov,
        RelativeMatcherBundle.vkeys_id);

    // expected hessian nnz: semantic 12, scalar 48
    EXPECT_EQ(sam::Inference::MatrixConverter::Semantic::HessianNNZ(syst.get_all_factors()), 12);
    EXPECT_EQ(sam::Inference::MatrixConverter::Scalar::HessianNNZ(syst.get_all_factors()), 48);
    // expected semantic jacobian nnz: semanctic 7, scalar 4+4*8
    EXPECT_EQ(sam::Inference::MatrixConverter::Semantic::JacobianNNZ(syst.get_all_factors()), 9);
    EXPECT_EQ(sam::Inference::MatrixConverter::Scalar::JacobianNNZ(syst.get_all_factors()),
              4 + (4 * 8));

    // std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << " Pre Optimised points: \n";
    auto sys_marginals = syst.get_marginals_as_map();
    std::cout << ::sam::Marginal::stringify_marginal_container_block(sys_marginals.data_map_tuple);

    try
    {
      syst.sam_optimise();
    }
    catch (const char* e)
    {
#if ENABLE_DEBUG_TRACE
      std::cerr << "SLAM algorithm failed. Reason: " << e << '\n';
#endif
    }
    std::cout << " After optimisation: \n";
    sys_marginals = syst.get_marginals_as_map();   // map {keyid : marginal} (tuple of maps, because
                                                   // marginals are not all the same type)
    std::cout << ::sam::Marginal::stringify_marginal_container_block(sys_marginals.data_map_tuple);

    // test: print all marginals, register values, hardcode them, and google test them
    // after optim
    auto expected_x3map = ::sam::Key::Spatial2d_t(-4.757e-05, 1);
    auto expected_x2map = ::sam::Key::Spatial2d_t(0.9975, 0.9513);
    auto expected_x1map = ::sam::Key::Spatial2d_t(0.969, -0.161);
    auto expected_x0map = ::sam::Key::Spatial2d_t(-4.034e-18, 1.862e-16);

    // sys_marginals is a 1-uple, let's simplify


    std::cout << ::sam::Meta::Key::Spatial2d::stringify_key_oneliner(
        *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("x3").value()))
              << '\n';

    EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
        "x0",
        expected_x0map,
        *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("x0").value()),
        1e3);   // 1000s precision, since we are close zero (1e-19), and the way the precision
                // works, this is ok
    EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
        "x1",
        expected_x1map,
        *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("x1").value()));
    EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
        "x2",
        expected_x2map,
        *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("x2").value()));
    EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
        "x3",
        expected_x3map,
        *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("x3").value()));

    // additional test: remove f5
    int nbfactors_before = std::apply([](const auto&... vect) { return (vect.size() + ...); },
                                      syst.get_all_factors());

    syst.remove_factor("f2");

    int nbfactors_after = std::apply([](const auto&... vect) { return (vect.size() + ...); },
                                     syst.get_all_factors());

    EXPECT_EQ(nbfactors_before, nbfactors_after + 1);


    // print all the factors
    std::cout << sam::Factor::stringify_wrapped_factor_container_block(syst.get_all_factors());
  }
};

//------------------------------------------------------------------//
//------------------------------------------------------------------//
//------------------------------------------------------------------//
//------------------------------------------------------------------//
//------------------------------------------------------------------//
//------------------------------------------------------------------//
//------------------------------------------------------------------//
//------------------------------------------------------------------//
TEST_F(ToyLinear2dSystem, SparseNaive)
{
  // scoped Timer
  PROFILE_FUNCTION();

  std::cout << "\n\n Declaring a sam system:\n";

  // auto syst = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseQR,
  // sam::Factor::Anchor2d, sam::Factor::RelativeMatcher2d>("A");
  auto syst = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseNaive,
                                             sam::Factor::Anchor2d,
                                             sam::Factor::RelativeMatcher2d>("A");

  this->process_tests(syst);
}

TEST_F(ToyLinear2dSystem, SparseQR)
{
  // scoped Timer
  PROFILE_FUNCTION();

  std::cout << "\n\n Declaring a sam system:\n";

  // auto syst = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseQR,
  // sam::Factor::Anchor2d, sam::Factor::RelativeMatcher2d>("A");
  auto syst = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseQR,
                                             sam::Factor::Anchor2d,
                                             sam::Factor::RelativeMatcher2d>("A");

  this->process_tests(syst);
}

TEST_F(ToyLinear2dSystem, SparseSimplicialLLT)
{
  // scoped Timer
  PROFILE_FUNCTION();

  std::cout << "\n\n Declaring a sam system:\n";

  auto syst = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseSimplicialLLT,
                                             sam::Factor::Anchor2d,
                                             sam::Factor::RelativeMatcher2d>("A");

  this->process_tests(syst);
}

TEST_F(ToyLinear2dSystem, SparseSupernodalLLT)
{

  // scoped Timer
  PROFILE_FUNCTION();

  std::cout << "\n\n Declaring a sam system:\n";

  auto syst = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseSupernodalLLT,
                                             sam::Factor::Anchor2d,
                                             sam::Factor::RelativeMatcher2d>("A");

  this->process_tests(syst);
}

// only for intel mkl
#if EIGEN_USE_MKL_VML
TEST_F(ToyLinear2dSystem, SparsePardisoLLT)
{
  // scoped Timer
  PROFILE_FUNCTION();

  std::cout << "\n\n Declaring a sam system:\n";

  auto syst = ::sam::Inference::SparseSystem<sam::Inference::SolverSparsePardisoLLT,
                                             sam::Factor::Anchor2d,
                                             sam::Factor::RelativeMatcher2d>("A");

  this->process_tests(syst);
}
#endif // EIGEN_USE_MKL_VML
