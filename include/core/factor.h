#ifndef FACTOR_H_
#define FACTOR_H_

#include "utils/tuple_patterns.h"
#include "utils/utils.h"

#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <map>
#include <string_view>
#include <tuple>
#include <utility>

// TODO: rename KCC to KeyContextModel : model in the context of a factor

template <typename DerivedKCC, typename KEYMETA,size_t DimMes, const char* ContextRole,bool LinearModel=false> 
struct KeyContextualConduct : KEYMETA
{
  using KeyMeta_t = KEYMETA;
  static constexpr const char*       kRole {ContextRole};
  static constexpr const std::size_t kM {DimMes};
  static constexpr const bool kLinear {LinearModel};
  // non static but const
  const std::string key_id;
  // non static, not const
  using process_matrix_t  = Eigen::Matrix<double, DimMes, KEYMETA::kN>;
  using measure_vect_t    = Eigen::Matrix<double, DimMes, 1>;
  using measure_cov_t     = Eigen::Matrix<double, DimMes, DimMes>;
  using part_state_vect_t = Eigen::Matrix<double, KEYMETA::kN, 1>;
  // measure_vect_t   b;
  const measure_cov_t& rho;

  // linearization_point
  // NOTE: not used when the wider system is linear.
  // NOTE: if this model is linear, but the wider system nonlinear, it is still used
  part_state_vect_t linearization_point = part_state_vect_t::Zero();

  process_matrix_t compute_part_A()
  {
    // NOTE: branch only for linear/nonlinear key MODEL
    // NOTE: indeed, if the key model is linear, it does not matter whether or not the wider 
    // NOTE: system is linear or nonlinear, the answer would still be rho*H
   if constexpr (LinearModel)
    return static_cast<DerivedKCC*>(this)->compute_part_A_impl(); 
   //else
    // return static_cast<DerivedKCC*>(this)->compute_part_A_at_LIN_POINT_impl(); // WARNING: AT LIN POINT
  }

    // NOTE: is used ?? h_part(part_x) doesnot make sense
    // NOTE:          ( H_part * part_x does )
    // FIX: redundant: make sure its ok to remove
  measure_vect_t compute_part_h_of_part_x(const part_state_vect_t& x)
  {
    return static_cast<DerivedKCC*>(this)->compute_part_h_of_part_x_impl(x);
  }

  // prevent default constructor, copy constructor, copy assignemnt operator
  KeyContextualConduct() = delete;
  // KeyContextualConduct(const KeyContextualConduct  &) = delete;
  // KeyContextualConduct& operator=(const KeyContextualConduct&) = delete;
 // ~KeyContextualConduct() = delete;
 // imposed constructor
  KeyContextualConduct(const std::string& key_id, const measure_cov_t& rho)
      : key_id(key_id)
      , rho(rho)
  {
  }
};

//------------------------------------------------------------------//
//                         Factor Template                          //
//------------------------------------------------------------------//
template <typename DerivedFactor,
          const char* FactorLabel,
          typename MEASURE_META,
          typename... KeyConducts>
class Factor
{
  public:
  using measure_vect_t = Eigen::Matrix<double, MEASURE_META::kM, 1>;
  using measure_cov_t  = Eigen::Matrix<double, MEASURE_META::kM, MEASURE_META::kM>;
  static constexpr const char* kFactorLabel {FactorLabel};
  static constexpr size_t      kN      = (KeyConducts::kN + ...);
  static constexpr size_t      kM      = MEASURE_META::kM;
  static constexpr size_t      kNbKeys = sizeof...(KeyConducts);
  using state_vector_t                 = Eigen::Matrix<double, kN, 1>;
  using process_matrix_t               = Eigen::Matrix<double, kM, kN>;
  // make a tuple of KeySet.  Michelin *** vaut le détour.
  using KeysSet_t = std::tuple<KeyConducts...>;

  static constexpr const char*                 kMeasureName {MEASURE_META::kMeasureName};
  static constexpr std::array<const char*, kM> kMeasureComponentsName = MEASURE_META::components;
  const std::string                            factor_id;   // fill at ctor
  const measure_vect_t                         z;           // fill at ctor
  const measure_cov_t                          z_cov;       // fill at ctor
  const measure_cov_t                          rho;         // fill at ctor
  const measure_vect_t                         rosie = rho*z;
  KeysSet_t keys_set;   // a tuple of the structures of each keys (dim, id,
                        // process matrix), fill at ctor, modifiable
  // TODO: isLinear : factor is linear if all keys models in this context are linear
  static constexpr bool isLinear = (KeyConducts::kLinear && ...);

  double error = 0;

  std::map<std::string, size_t> keyIdToTupleIdx;   // fill at ctor

  // ctor helper
  std::map<std::string, std::size_t>
      map_keyid(const std::array<std::string, kNbKeys>& keys_id) const
  {
    std::map<std::string, std::size_t> result;
    for (int i = 0; i < keys_id.size(); i++) result[keys_id[i]] = i;
    return result;
  }


  // // TODO: URGENT: , the lin point will be internal (remove argument)
  // template <typename... PARTIAL_STATE_VECTORS_T> // TODO: do a tuple here !
  // measure_vect_t compute_b(const std::tuple<PARTIAL_STATE_VECTORS_T...>& x_tup) const
  // {
  //     static_assert(std::tuple_size_v<std::tuple<PARTIAL_STATE_VECTORS_T...>> == kNbKeys);
  //     static_assert(sizeof...(PARTIAL_STATE_VECTORS_T) == kNbKeys);
  //     return this->rosie - this->rho*this->compute_h_of_x(x_tup);
  // }
  // the refactored compute_b
  measure_vect_t compute_b_nl() const
  {
      auto  tuple_means = this->get_tuple_of_linearization_points();
      return this->rosie - this->rho*this->compute_h_of_x(tuple_means);
  }

  //  func that gets the tup of lin points contained in kcm into state_vector_t
  std::tuple<typename KeyConducts::part_state_vect_t ...> get_tuple_of_linearization_points() const
  {
    std::tuple<typename KeyConducts::part_state_vect_t ...> tup_mean;
    std::apply([this,&tup_mean](const auto & ...kcc) //-> std::tuple<typename KeyConducts::state_vector_t ...>
    {
      tup_mean = {kcc.linearization_point ... };
    }
    ,this->keys_set);
    return tup_mean;
  }

  //  func that transforms a tup of lin points contained in kcm into state_vector_t
  template <typename... PARTIAL_STATE_VECTORS_T>
  state_vector_t get_state_vector_from_tuple(const std::tuple<PARTIAL_STATE_VECTORS_T...> & x_tup) const
  {
    static_assert(sizeof...(PARTIAL_STATE_VECTORS_T) == kNbKeys);
    state_vector_t x;

    std::apply([&x](auto... partx)
    {
      ((x << partx ),  ...);
        // x << (partx, ...) ;        
    }, x_tup); // TODO: check, too good to be true ??

    return x;
  }


  // TODO: deprecate this one (only keep the 'compute_h_of_x' that calls the Eigen::vector)
  // DEPRECATED:
  template <typename... PARTIAL_STATE_VECTORS_T>
  measure_vect_t compute_h_of_x(const std::tuple<PARTIAL_STATE_VECTORS_T...>& x_tup) const
  {
    static_assert(sizeof...(PARTIAL_STATE_VECTORS_T) == kNbKeys);
    measure_vect_t h_of_x=measure_vect_t::Zero();

    state_vector_t x;

    std::apply([&x](auto... partx)
    {
      ((x << partx ),  ...);
        // x << (partx, ...) ;        
    }, x_tup); // TODO: check, too good to be true ??
    return compute_h_of_x(x);
  }

  measure_vect_t compute_h_of_x(const state_vector_t & x) const
  {
    // class instantiation dependent
    return static_cast<const DerivedFactor*>(this)->compute_h_of_x_impl(x);
  }

  // the ctor
  Factor(const std::string&                      factor_id,
         const measure_vect_t&                   z,
         const measure_cov_t&                    z_cov,
         const std::array<std::string, kNbKeys>& keys_id)
      : z(z)
      , z_cov(z_cov)
      , factor_id(factor_id)
      , rho(Eigen::LLT<measure_cov_t>(z_cov.inverse()).matrixU())
      , keys_set(sam_tuples::reduce_array_variadically(
            keys_id,
            []<std::size_t... I>(const auto& my_keys_id,
                                 const auto& rho,
                                 std::index_sequence<I...>) -> decltype(keys_set) {
              // return std::make_tuple(KeyConducts(my_keys_id[I], rho)...); // original
              // might be possible to use perfect forwarding, by declaring an empty tuple
              // and next line expanding tuple_cat with an intermediary function that has perfect forwarding ( TODO:)
              return  { KeyConducts(my_keys_id[I], rho) ... } ;
            },
            rho))
      , keyIdToTupleIdx(map_keyid(keys_id))
  {
  }

  double compute_error(const state_vector_t& x) const
  {
    // TODO: small improvement possible for linear factor (and linear factor in NL syst ?)
    return (this->rho * this->compute_h_of_x(x) - this->rosie).norm();
  }

  // for the nonlinears
  // void set_linearization_point(const state_vector_t & lin_point)
  // {
  //   linearization_point = lin_point;
  // }
};


//------------------------------------------------------------------//
//                      Helper print functions                      //
//------------------------------------------------------------------//
// TODO: use tuple_patterns rather
// static traverse of the KeySet tuple
template <typename TUP, size_t I = 0>
constexpr void traverse_tup()
{
  if constexpr (I == std::tuple_size<TUP>::value) { return; }
  else
  {
    using KT = std::tuple_element_t<I, TUP>;
    std::cout << "\t\t+ Key Nature: " << KT::kKeyName << ".  Role: " << KT::kRole << '\n';

    // recursive call
    traverse_tup<TUP, I + 1>();
  }
}

// runtime traverse of the KeySet tuple
template <typename TUP, size_t I = 0>
void traverse_tup(const TUP& tup)
{
  if constexpr (I == std::tuple_size<TUP>::value) { return; }
  else
  {
    using KT = std::tuple_element_t<I, TUP>;
    std::cout << "\t\t+ Key Nature: " << KT::kKeyName << ".  Role: " << KT::kRole
              << ". Id: " << std::get<I>(tup).key_id << '\n';
    // std::cout << "\t\t\t A:\n" << std::get<I>(tup).A << '\n';

    // recursive call
    traverse_tup<TUP, I + 1>(tup);
  }
}

template <typename KC>
void print_KeyContextConduct(const KC& kcc)
{
  std::cout << "\t\t+ Key Nattupelemure: " << KC::kKeyName << ".  Role: " << KC::kRole
            << ". Id: " << kcc.key_id << '\n';
}


// print static information of a factor label
template <typename FT>
constexpr void factor_print()
{
  std::cout << FT::kFactorLabel << '\n';
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n' << "\tKeys (in order):\n";

  // traverse statically the ttupelemuple of keys data
  traverse_tup<typename FT::KeysSet_t>();

  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << " { ";
  for (const auto& comp : FT::kMeasureComponentsName) std::cout << comp << " ";
  std::cout << "{\n";


  std::cout << "\t----- \n";
}

// This is the non static version
template <typename FT>
void factor_print(const FT& fact)
{
  Eigen::IOFormat
      CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "  ", ";");

  std::cout << FT::kFactorLabel << " - id : " << fact.factor_id << '\n';
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n' << "\tKeys (in order):\n";

  // traverse_tup(fact.keys_set);
  // for_each_in_tuple(fact.keys_set, [](const auto& kcc){
  //   // using KT = ;
  //   std::cout << "\t\t+ Key Nature: " << decltype(kcc)::kKeyName << ".  Role: " <<
  //   decltype(kcc)::kRole
  //             << ". Id: " << kcc.key_id << '\n';
  // });
  // for_each_in_tuple(fact.keys_set, &printtupelem);
  std::apply([](auto... kcc) { ((print_KeyContextConduct(kcc)), ...); }, fact.keys_set);


  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << "\n\t\t { ";
  for (int i = 0; i < FT::kMeasureComponentsName.size(); i++)
    std::cout << FT::kMeasureComponentsName[i] << ": " << fact.z[i] << "  ";
  std::cout << "}\n\t\t Cov: [ " << fact.z_cov.format(CommaInitFmt) << " ] \n";


  std::cout << "\t----- \n";
}

#endif
