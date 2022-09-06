#pragma once

#include "factor/factor_interface.h"
#include "key-spatial-2d/key-spatial-2d.h"
#include "measure-motion-2d/measure-motion-2d.h"

namespace details_sam::Factor{
  namespace RelativeMatcher2dImpl{

    inline static constexpr const char sighted_var[]           = "sighted";
    inline static constexpr const char observer_var[]           = "observer";
    // the process matrices
    inline static const Eigen::Matrix<double, 2, 2> Hik_Observer {{-1,0},{0,-1}};
    inline static const Eigen::Matrix<double, 2, 2> Hik_Sighted {{1,0},{0,1}};

    using namespace ::sam::Meta;
    using namespace ::details_sam::Conduct;

    using ObserverKeyConduct = LinearKeyContextualConduct<Key::Spatial2d , Measure::Motion2d , observer_var, &Hik_Observer>;
    using SightedKeyConduct = LinearKeyContextualConduct<Key::Spatial2d, Measure::Motion2d, sighted_var, &Hik_Sighted>;

    inline static constexpr const char RelativeMatcher2dLabel[] = "linear translation 2d";

    namespace exports{

      class RelativeMatcher2d
        : public 
          ::sam::Factor::LinearEuclidianFactor
            <
              RelativeMatcher2d,
              RelativeMatcher2dLabel,
              SightedKeyConduct,
              ObserverKeyConduct
            >
      {
        using BaseFactor_t = ::sam::Factor::LinearEuclidianFactor
                              <
                                RelativeMatcher2d,
                                RelativeMatcher2dLabel,
                                SightedKeyConduct,
                                ObserverKeyConduct
                              >;

        friend BaseFactor_t;
        using SightedKey_process_matrix = typename SightedKeyConduct::key_process_matrix_t;
        using ObserverKey_process_matrix = typename ObserverKeyConduct::key_process_matrix_t;

        public:

        // ctor
        RelativeMatcher2d(
            const std::string&                                               factor_id,
            const measure_t&                                            mes_vect,
            const measure_cov_t&                                             measure_cov,
            const std::array<std::string, kNbKeys>& keys_id)
            : BaseFactor_t(factor_id, mes_vect, measure_cov, keys_id)
        {
        }
        static constexpr uint8_t kSightedKeyConductIdx = 0;
        static constexpr uint8_t kObserverKeyConductIdx = 1;

        // guess a potentially missing init key point from another by relying on the measurement
        // (used in NL systems if an init point must be set)
        static
        // std::optional<
        //     std::tuple< std::shared_ptr<SightedKeyConduct::part_state_vect_t>, std::shared_ptr<ObserverKeyConduct::part_state_vect_t>>>
        std::optional<composite_state_ptr_t>
            guess_init_key_points_impl(
                // const std::tuple<std::optional<std::shared_ptr<SightedKeyConduct::part_state_vect_t>>,
                //                  std::optional<std::shared_ptr<ObserverKeyConduct::part_state_vect_t>>>&
                const composite_of_opt_state_ptr_t &
                                      x_init_ptr_optional_tup,
                const measure_t& z)
        {
          // if both values are given, just echo the means
          if (std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
              && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
          {
            return std::make_tuple(std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).value(),
                                   std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value());
          }
          // if sighted is given (eg x_{i-1}) but observer is not (eg x_{i}), use the measure to deduce
          // an init point for observer NOTE: this is the most likely scenario
          else if (std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                   && !std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
          {
            std::shared_ptr<ObserverKeyConduct::Key_t> observer_init_point_ptr;
            std::shared_ptr<SightedKeyConduct::Key_t> sighted_init_point_ptr
                = std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).value();


            observer_init_point_ptr = std::make_shared<ObserverKeyConduct::Key_t>(*sighted_init_point_ptr - z);

            return std::make_tuple(sighted_init_point_ptr, observer_init_point_ptr);
          }
          // if obverseR is given, and not the other one
          else if (!std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                   && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
          {
            std::shared_ptr<ObserverKeyConduct::Key_t> observer_init_point_ptr
                = std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value();
            std::shared_ptr<SightedKeyConduct::Key_t> sighted_init_point_ptr;

            sighted_init_point_ptr = std::make_shared<SightedKeyConduct::Key_t>(z - *observer_init_point_ptr);

            return std::make_tuple(sighted_init_point_ptr, observer_init_point_ptr);
          }
          // if no init on either key is given,
          else
          {
            return std::nullopt;
          }
        }
      };

    } // exports
  } // Impl
} // details_sam

namespace sam::Factor{
  using namespace details_sam::Factor::RelativeMatcher2dImpl::exports;
}

