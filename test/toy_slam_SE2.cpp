#include "anchorSE2/anchorSE2.h"
#include "cartesian-landmark-obs-SE2/cartesian-landmark-obs-SE2.h"
#include "motion-model-SE2/motion-model-SE2.h"
#include "system/sam-system.h"
#include "test_utils.h"

#include <gtest/gtest.h>
#include <iomanip>   // std::precision
#include <iostream>

using std::cout, std::endl;
using namespace sam::Factor;

class ToySLAMSE2System: public ::testing::Test
{
  protected:
    void SetUp() override
    {
      // DEBUG INFO
      cout << endl;
      cout << "2D Smoothing and Mapping. 3 poses, 5 landmarks." << endl;
      cout << " This problem setting originates from the manif library examples." << endl;
      cout << "                  ------- b1        " << endl;
      cout << "          b3    /         |         " << endl;
      cout << "          |   /       b4  |         " << endl;
      cout << "          | /       /    \\|         " << endl;
      cout << "          X0 ---- X1 ---- X2        " << endl;
      cout << "          | \\   /   \\   /           " << endl;
      cout << "          |   b0      b2            " << endl;
      cout << "          *                         " << endl;
      cout << "-----------------------------------------------" << endl;
      // ground truth for landmarks
      std::vector<::sam::Key::Spatial2d_t> landmarks_simu;
      {
        ::sam::Key::Spatial2d_t b0, b1, b2, b3, b4;
        b0 << 3.0, 0.0;
        b1 << 2.0, -1.0;
        b2 << 2.0, 1.0;
        b3 << 3.0, -1.0;
        b4 << 3.0, 1.0;
        landmarks_simu.push_back(b0 * 5);
        landmarks_simu.push_back(b1 * 5);
        landmarks_simu.push_back(b2 * 5);
        landmarks_simu.push_back(b3 * 5);
        landmarks_simu.push_back(b4 * 5);
      }
      //------------------------------------------------------------------//
      //                      Ground truth for poses                      //
      //------------------------------------------------------------------//
      int                                   NUMPOSES = 3;
      std::vector<::sam::Key::SpatialSE2_t> true_poses;
      // applied velocities per time step : vx = 3, omega = -0.4
      ::sam::Meta::Key::SpatialSE2::tangent_space_t u_nom;
      u_nom << 3, 0, -.4;
      // u_nom     << 0.1, 0.0, 0.05;

      // initial pose
      true_poses.emplace_back(10, 1, -MANIF_PI / 6);
      // true_poses.emplace_back(0, 0, 0);
      // applying recursively a control
      for (int i = 1; i < NUMPOSES; i++) true_poses.push_back(true_poses[i - 1] + u_nom);
      cout << "\n\n factors registration:\n";
      // covariances
      Eigen::Array<double, 3, 1> prior_sigmas, control_sigmas;
      Eigen::Array<double, 2, 1> landmark_meas_sigmas;
      prior_sigmas << 0.000002, 0.000002, 0.0000005;
      control_sigmas << 0.5, 0.0003, 0.1;
      landmark_meas_sigmas << 0.1, 0.1;
      ::sam::Factor::AnchorSE2::measure_cov_t cov_prior = prior_sigmas.square().matrix().asDiagonal();
      ::sam::Factor::MotionModelSE2::measure_cov_t cov_control
          = control_sigmas.square().matrix().asDiagonal();
      ::sam::Factor::LandmarkCartesianObsSE2::measure_cov_t cov_landmark
          = landmark_meas_sigmas.square().matrix().asDiagonal();

      // t0
      PreRegistrationBundleCollection<AnchorSE2,MotionModelSE2,LandmarkCartesianObsSE2>::tuple_t collection_t0;
      // f0
      {
        auto z = ::sam::Factor::AnchorSE2::measure_t(
            true_poses[0]);   // or ::sam::Measure::AbsolutePosition2d is the same
        std::get<0>(collection_t0).push("f0", z, cov_prior, {"x0"});
      }
      // f1
      {
        auto z = ::sam::Measure::Motion2d_t(4.8335, 1.7085);
        std::get<2>(collection_t0).push("f1",
                                                                        z,
                                                                        cov_landmark,
                                                                        {"b0", "x0"});
      }
      // f2
      {
        auto z = ::sam::Measure::Motion2d_t(3.0034, -5.2488);
        std::get<2>(collection_t0).push("f2",
                                                                        z,
                                                                        cov_landmark,
                                                                        {"b1", "x0"});
      }
      // f3
      {
        auto z = ::sam::Measure::Motion2d_t(7.3764, -2.6761);
        std::get<2>(collection_t0).push("f3",
                                                                        z,
                                                                        cov_landmark,
                                                                        {"b3", "x0"});
      }
      this->collections.vector_of_collection.push_back(collection_t0);
      // t1
      PreRegistrationBundleCollection<AnchorSE2,MotionModelSE2,LandmarkCartesianObsSE2>::tuple_t collection_t1;
      // f4
      {
        auto z = ::sam::Measure::VelocitySE2_t(4.0623, 0.0001, -0.2979);
        std::get<1>(collection_t1).push("f4", z, cov_control, {"x1", "x0"});
      }
      // f5
      {
        auto z = ::sam::Measure::Motion2d_t(0.8791, 2.7602);
        std::get<2>(collection_t1).push("f5", z, cov_landmark, {"b0", "x1"});
      }
      // f6
      {
        auto z = ::sam::Measure::Motion2d_t(-6.1805, 1.9506);
        std::get<2>(collection_t1).push("f6", z, cov_landmark, {"b2", "x1"});
      }
      // f7
      {
        auto z = ::sam::Measure::Motion2d_t(-3.1906, 5.5414);
        std::get<2>(collection_t1).push("f7", z, cov_landmark, {"b4", "x1"});
      }
      this->collections.vector_of_collection.push_back(collection_t1);
      // t2
      PreRegistrationBundleCollection<AnchorSE2,MotionModelSE2,LandmarkCartesianObsSE2>::tuple_t collection_t2;
      // f8
      {
        auto z = ::sam::Measure::VelocitySE2_t(2.6136, -0.0002, -0.6015);   // vx.t , vy.t , w.t
        std::get<1>(collection_t2).push("f8", z, cov_control, {"x2", "x1"});
      }
      // f9
      {
        auto z = ::sam::Measure::Motion2d_t(0.3480, -3.7441);
        std::get<2>(collection_t2).push("f9", z, cov_landmark, {"b1", "x2"});
      }
      // f10
      {
        auto z = ::sam::Measure::Motion2d_t(-9.2770, -1.3639);
        std::get<2>(collection_t2).push("f10", z, cov_landmark, {"b2", "x2"});
      }
      // f11
      {
        auto z = ::sam::Measure::Motion2d_t(-8.0425, 3.6379);
        std::get<2>(collection_t2).push("f11", z, cov_landmark, {"b4", "x2"});
      }
      this->collections.vector_of_collection.push_back(collection_t2);


    }

    PreRegistrationBundleCollection<AnchorSE2,MotionModelSE2,LandmarkCartesianObsSE2> collections;

    // void TearDown() override;

    template <typename SYS_T>
    void process_tests(SYS_T& sys)
    {
      for (const auto & collection: this->collections.vector_of_collection)
      {
        auto & AnchorBundle = std::get<0>(collection);
        auto & MotionModel = std::get<1>(collection);
        auto & LandmarkObs = std::get<2>(collection);

        if (AnchorBundle.size() >0) sys.template register_factors_in_bundle<AnchorSE2>(AnchorBundle.vfid,AnchorBundle.vmeasure,AnchorBundle.vmeasure_cov,AnchorBundle.vkeys_id);
        if (!MotionModel.empty()) sys.template register_factors_in_bundle<MotionModelSE2>(MotionModel.vfid,MotionModel.vmeasure,MotionModel.vmeasure_cov,MotionModel.vkeys_id);
        if (!LandmarkObs.empty()) sys.template register_factors_in_bundle<LandmarkCartesianObsSE2>(LandmarkObs.vfid,LandmarkObs.vmeasure,LandmarkObs.vmeasure_cov,LandmarkObs.vkeys_id);
      }

      // expected hessian nnz: semantic 30, scalar 7*9 + 4*5 + 18*6 = 191
      EXPECT_EQ( sam::Inference::MatrixConverter::Semantic::HessianNNZ(sys.get_all_factors()),30 );
      EXPECT_EQ( sam::Inference::MatrixConverter::Scalar::HessianNNZ(sys.get_all_factors()),191 );
      // expected semantic jacobian nnz: semanctic 23, scalar 9+2*18+9*10 = 135 
      EXPECT_EQ( sam::Inference::MatrixConverter::Semantic::JacobianNNZ(sys.get_all_factors()),23 );
      EXPECT_EQ( sam::Inference::MatrixConverter::Scalar::JacobianNNZ(sys.get_all_factors()),135 );

      // get init point before optimisation
      std::cout << "Before Optimisation:\n";
      auto sys_marginals = sys.get_marginals_as_map();
      std::cout << ::sam::Marginal::stringify_marginal_container_block(sys_marginals.data_map_tuple);
      auto expected_b4_init = ::sam::Key::Spatial2d_t(15.05, 4.589);
      auto expected_b2_init = ::sam::Key::Spatial2d_t(10.38, 4.332);
      auto expected_b3_init = ::sam::Key::Spatial2d_t(15.05, -5.006);
      auto expected_b1_init = ::sam::Key::Spatial2d_t(9.977, -5.047);
      auto expected_b0_init = ::sam::Key::Spatial2d_t(15.04, 0.06285);
      auto expected_x2_init = ::sam::Key::SpatialSE2_t(14.28, -3.841, -1.423);
      auto expected_x1_init = ::sam::Key::SpatialSE2_t(13.17, -1.521, -0.8215);
      auto expected_x0_init = ::sam::Key::SpatialSE2_t(10, 1, -0.5236);

      // auto all_positionSE2 =
      // auto all_positionSE2      = std::get<1>(sys_marginals);
      // auto all_positionLandmark = std::get<0>(sys_marginals);
      EXPECT_KEY_APPROX<::sam::Meta::Key::SpatialSE2>("x0",
          expected_x0_init,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::SpatialSE2>("x0").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::SpatialSE2>("x1",
          expected_x1_init,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::SpatialSE2>("x1").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::SpatialSE2>("x2",
          expected_x2_init,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::SpatialSE2>("x2").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b0",
          expected_b0_init,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b0").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b1",
          expected_b1_init,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b1").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b2",
          expected_b2_init,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b2").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b3",
          expected_b3_init,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b3").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b4",
          expected_b4_init,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b4").value()));

      sys.sam_optimise();

      // get Keys after optimisation
      std::cout << "After Optimisation:\n";
      sys_marginals        = sys.get_marginals_as_map();
      std::cout << ::sam::Marginal::stringify_marginal_container_block(sys_marginals.data_map_tuple);

      auto expected_b4_map = ::sam::Key::Spatial2d_t(14.89, 4.983);
      auto expected_b2_map = ::sam::Key::Spatial2d_t(9.981, 5.01);
      auto expected_b3_map = ::sam::Key::Spatial2d_t(15.05, -5.006);
      auto expected_b1_map = ::sam::Key::Spatial2d_t(10.04, -5);
      auto expected_b0_map = ::sam::Key::Spatial2d_t(15.04, 0.02159);
      auto expected_x2_map = ::sam::Key::SpatialSE2_t(13.62, -3.645, -1.308);
      auto expected_x1_map = ::sam::Key::SpatialSE2_t(12.32, -1.019, -0.9104);
      auto expected_x0_map = ::sam::Key::SpatialSE2_t(10, 1, -0.5236);

      // all_positionSE2      = std::get<1>(sys_marginals);
      // all_positionLandmark = std::get<0>(sys_marginals);
      EXPECT_KEY_APPROX<::sam::Meta::Key::SpatialSE2>("x0",
          expected_x0_map,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::SpatialSE2>("x0").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::SpatialSE2>("x1",
          expected_x1_map,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::SpatialSE2>("x1").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::SpatialSE2>("x2",
          expected_x2_map,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::SpatialSE2>("x2").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b0",
          expected_b0_map,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b0").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b1",
          expected_b1_map,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b1").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b2",
          expected_b2_map,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b2").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b3",
          expected_b3_map,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b3").value()));
      EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>(
          "b4",
          expected_b4_map,
          *(sys_marginals.template find_mean_ptr<sam::Meta::Key::Spatial2d>("b4").value()));


      // additional test: remove f5
      int nbfactors_before = 
        std::apply([](const auto & ...vect)
            { 
            return (vect.size() + ...) ;  
            }
            ,sys.get_all_factors());

      sys.remove_factor("f5");

      int nbfactors_after = 
        std::apply([](const auto & ...vect)
            { 
            return (vect.size() + ...) ;  
            }
            ,sys.get_all_factors());

      EXPECT_EQ(nbfactors_before, nbfactors_after+1);

      // print all factors
      std::cout << sam::Factor::stringify_wrapped_factor_container_block(sys.get_all_factors());

      Eigen::Array<double, 2, 1> landmark_meas_sigmas;
      landmark_meas_sigmas << 0.1, 0.1;
      ::sam::Factor::LandmarkCartesianObsSE2::measure_cov_t cov_landmark
          = landmark_meas_sigmas.square().matrix().asDiagonal();

      // f5 // this should work since f5 has been remove
      {
        auto z = ::sam::Measure::Motion2d_t(0.8791, 2.7602);
        sys.template register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f5",
            z,
            cov_landmark,
            {"b0", "x1"});
      }
      // f6 // this should fail ("f6 already exists")
      {
        auto z = ::sam::Measure::Motion2d_t(-6.1805, 1.9506);
        EXPECT_THROW(
            sys.template register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f6",
              z,
              cov_landmark,
              {"b2", "x1"});
            ,std::runtime_error);
      }
    }
};

TEST_F(ToySLAMSE2System, SparseNaive)
{
  // logger
  sam_utils::JSONLogger::Instance().beginSession("gtest_slam_SE2", "manif_example_slam_SE2");
  // scoped Timer
  PROFILE_FUNCTION();

  cout << "\n\n Declaring a sam system:\n";
  auto sys = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseNaive,
                                      ::sam::Factor::MotionModelSE2,
                                      ::sam::Factor::AnchorSE2,
                                      ::sam::Factor::LandmarkCartesianObsSE2>("slam system");

  this->process_tests(sys); 
}

TEST_F(ToySLAMSE2System, SparseQR)
{
  // logger
  sam_utils::JSONLogger::Instance().beginSession("gtest_slam_SE2", "manif_example_slam_SE2");
  // scoped Timer
  PROFILE_FUNCTION();

  cout << "\n\n Declaring a sam system:\n";
  auto sys = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseQR,
                                      ::sam::Factor::MotionModelSE2,
                                      ::sam::Factor::AnchorSE2,
                                      ::sam::Factor::LandmarkCartesianObsSE2>("slam system");

  this->process_tests(sys); 
}

TEST_F(ToySLAMSE2System, SparseSimplicialLLT)
{
  // logger
  sam_utils::JSONLogger::Instance().beginSession("gtest_slam_SE2", "manif_example_slam_SE2");
  // scoped Timer
  PROFILE_FUNCTION();

  cout << "\n\n Declaring a sam system:\n";
  auto sys = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseSimplicialLLT,
                                      ::sam::Factor::MotionModelSE2,
                                      ::sam::Factor::AnchorSE2,
                                      ::sam::Factor::LandmarkCartesianObsSE2>("slam system");

  this->process_tests(sys); 
}

TEST_F(ToySLAMSE2System, SparseSupernodalLLT)
{
  // logger
  sam_utils::JSONLogger::Instance().beginSession("gtest_slam_SE2", "manif_example_slam_SE2");
  // scoped Timer
  PROFILE_FUNCTION();

  cout << "\n\n Declaring a sam system:\n";
  auto sys = ::sam::Inference::SparseSystem<sam::Inference::SolverSparseSupernodalLLT,
                                      ::sam::Factor::MotionModelSE2,
                                      ::sam::Factor::AnchorSE2,
                                      ::sam::Factor::LandmarkCartesianObsSE2>("slam system");

  this->process_tests(sys); 
}

// only for intel mkl
#if EIGEN_USE_MKL_VML
TEST_F(ToySLAMSE2System, SparsePardisoLLT)
{
  // logger
  sam_utils::JSONLogger::Instance().beginSession("gtest_slam_SE2", "manif_example_slam_SE2");
  // scoped Timer
  PROFILE_FUNCTION();

  cout << "\n\n Declaring a sam system:\n";
  auto sys = ::sam::Inference::SparseSystem<sam::Inference::SolverSparsePardisoLLT,
                                      ::sam::Factor::MotionModelSE2,
                                      ::sam::Factor::AnchorSE2,
                                      ::sam::Factor::LandmarkCartesianObsSE2>("slam system");

  this->process_tests(sys); 
}
#endif // EIGEN_USE_MKL_VML
