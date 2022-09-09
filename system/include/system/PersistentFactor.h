#pragma once

#include "system/config.h"

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace sam::Factor
{

  // all the factor data at one specefic point
  template <typename FACTOR_T>
  struct PersistentFactorData
  {
    using matrices_Aik_t = typename FACTOR_T::matrices_Aik_t;
    using vector_bi_t    = typename FACTOR_T::criterion_t;

    matrices_Aik_t Aiks;
    vector_bi_t    bi;
    double         norm;

    PersistentFactorData(const matrices_Aik_t& Aiks, const vector_bi_t& bi, double norm)
        : Aiks(Aiks)
        , bi(bi)
        , norm(norm)
    {
    }
  };

  template <typename FACTOR_T>
  std::string stringify_persistent_factor_data(const PersistentFactorData<FACTOR_T>& pfd,
                                               int                                   tab       = 4,
                                               int                                   precision = 4)
  {
    Eigen::IOFormat
        CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "  ", ";");

    std::stringstream ss;
    ss << std::setw(tab) << " "
       << "Aiks:\n";
    std::apply([&ss](const auto&... Aik) { ((ss << Aik << '\n'), ...); }, pfd.Aiks);
    ss << std::setw(tab) << " "
       << "bi:\n";
    ss << pfd.bi << "\n";
    ss << std::setw(tab) << " norm : " << pfd.norm << "\n";

    return ss.str();
  }


  // a wrapper for factor, with storing informations about a lin point
  template <typename FACTOR_T, bool LinearSystem = false>
  class WrapperPersistentFactor
  {
    public:
    using Factor_t                     = FACTOR_T;
    using composite_state_ptr_t        = typename FACTOR_T::composite_state_ptr_t;
    using composite_of_opt_state_ptr_t = typename FACTOR_T::composite_of_opt_state_ptr_t;
    using measure_t                    = typename FACTOR_T::measure_t;
    using measure_cov_t                = typename FACTOR_T::measure_cov_t;
    using matrices_Aik_t               = typename FACTOR_T::matrices_Aik_t;
    using vector_bi_t                  = typename FACTOR_T::criterion_t;


    WrapperPersistentFactor(const std::string&                                factor_id,
                            const measure_t&                                  z,
                            const measure_cov_t&                              z_cov,
                            const std::array<std::string, FACTOR_T::kNbKeys>& keys_id,
                            const composite_state_ptr_t&                      tup_init_points_ptr)
        // for now, even linear factor in linear system have init point
        : factor(FACTOR_T(factor_id, z, z_cov, keys_id))
        , state_point_view(tup_init_points_ptr)
        , persistentData_(this->compute_persistent_data())
    {
      // print init state
#if ENABLE_DEBUG_TRACE
      std::cout << ">>>>>>>>>>\n";
      std::cout << stringify_factor_blockliner(this->factor) << '\n';
      std::cout << stringify_persistent_factor_data(persistentData_) << '\n';
      // if all the state of the init pointer are VALID pointers
      if (std::apply([&tup_init_points_ptr](auto&&... Xptr) { return ((Xptr != nullptr) && ...); },
                     tup_init_points_ptr))
      {
        std::cout << "init point(s) : \n";
        std::cout << stringify_composite_state_blockliner<typename FACTOR_T::KeysSet_t>::str(
            tup_init_points_ptr,
            this->factor.get_array_keys_id(),
            4,
            4);
      }
      else { std::cout << "no init point(s) given. \n"; }
      std::cout << "------------ \n ";
      std::cout << "<<<<<<<<<<<\n";
#endif
    }

    // shared with other structures in system
    const composite_state_ptr_t state_point_view;

    // factor (the wrapped thing)
    FACTOR_T factor;

    // state point has changed (e.g. the system updates it), the data must be updated
    PersistentFactorData<FACTOR_T> compute_persistent_data() const
    {
      // WARNING: not protected against race condition on the current state X
      vector_bi_t    bj;
      matrices_Aik_t Ajks;

      if constexpr (LinearSystem)
      {
        // std::cout << "linear \n";
        static_assert(FACTOR_T::isLinear);
        std::tie(bj, Ajks) = this->factor.compute_Ai_bi_linear();
      }
      else { std::tie(bj, Ajks) = this->factor.compute_Ai_bi_at(this->state_point_view); }
#if ENABLE_DEBUG_TRACE
      std::cout << stringify_composite_state_blockliner<typename FACTOR_T::KeysSet_t>::str(
          this->state_point_view,
          this->factor.get_array_keys_id(),
          4,
          4);
      // std::cout << bj<<'\n';
      // std::cout << this->factor.rosie <<'\n';
#endif

      double norm = this->factor.factor_norm_at(this->state_point_view);

      // Possibly later compute r(X) too.
      // But note that it norm_at(X) computes r(X) first, which is redundant
      return PersistentFactorData<FACTOR_T>(Ajks, bj, norm);
    }

    PersistentFactorData<FACTOR_T> get_current_point_data() const { return persistentData_; }

    std::vector<double> norm_history;

    void clear_history() { this->norm_history.clear(); }

    void set_persistent_data(const PersistentFactorData<FACTOR_T>& persistent_data)
    {
      persistentData_ = persistent_data;
    }

    private:
    PersistentFactorData<FACTOR_T> persistentData_;
  };


}   // namespace sam::Factor
