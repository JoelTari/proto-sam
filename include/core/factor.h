#ifndef FACTOR_H_
#define FACTOR_H_

#include <array>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <string_view>
#include <tuple>
#include <utility>


template <typename DerivedKCC, typename KEYMETA, size_t DimMes, const char* ContextRole>
struct KeyContextualConduct : KEYMETA
{
  static constexpr const char* kRole {ContextRole};
  // non static but const
  const std::string key_id;
  // non static, not const
  using process_matrix_t  = Eigen::Matrix<double, DimMes, KEYMETA::kN>;
  using measure_vect_t    = Eigen::Matrix<double, DimMes, 1>;
  using measure_cov_t     = Eigen::Matrix<double, DimMes, DimMes>;
  using part_state_vect_t = Eigen::Matrix<double, KEYMETA::kN, 1>;
  // measure_vect_t   b;
  const measure_cov_t& rho;

  process_matrix_t compute_part_A()
  {
    // in linear it would just be a getter to  rho * H
    // in nonlinear, set_linearization_point must occur before
    return static_cast<DerivedKCC*>(this)->compute_part_A_impl();
  }

  measure_vect_t compute_part_h_of_part_x(const part_state_vect_t& x)
  {
    return static_cast<DerivedKCC*>(this)->compute_part_h_of_part_x_impl(x);
  }

  KeyContextualConduct() = delete;
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
          const char FactorLabel[],
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
  // make a tuple of KeySet.  Michelin *** vaut le détour.
  using KeysSet_t = std::tuple<KeyConducts...>;

  static constexpr const char*                 kMeasureName {MEASURE_META::kMeasureName};
  static constexpr std::array<const char*, kM> kMeasureComponentsName = MEASURE_META::components;
  const std::string                            factor_id;   // fill at ctor
  const measure_vect_t                         z;           // fill at ctor
  const measure_cov_t                          z_cov;       // fill at ctor
  const measure_cov_t                          rho;         // fill at ctor
  KeysSet_t keys_set;   // a tuple of the structures of each keys (dim, id,
                        // process matrix), fill at ctor, modifiable

  std::map<std::string, size_t> keyIdToTupleIdx;   // fill at ctor

  // ctor helper
  std::map<std::string, std::size_t>
      map_keyid(const std::array<std::string, kNbKeys>& keys_id) const
  {
    std::map<std::string, std::size_t> result;
    for (int i = 0; i < keys_id.size(); i++) result[keys_id[i]] = i;
    return result;
  }


  // HACK: make_index_sequence <3  . This pattern allows the underlying function
  // to be defined with expansion syntax
  KeysSet_t init_tuple_keys(const std::array<std::string, kNbKeys>& my_keys_id,
                            const measure_cov_t&                    rho) const
  {
    return init_tuple_keys_impl(my_keys_id, rho, std::make_index_sequence<kNbKeys> {});
  }

  template <std::size_t... I>
  KeysSet_t init_tuple_keys_impl(const std::array<std::string, kNbKeys>& my_keys_id,
                                 const measure_cov_t&                    rho,
                                 std::index_sequence<I...>) const
  {
    return std::make_tuple(KeyConducts(my_keys_id[I], rho)...);
  }

  template <typename... PARTIAL_STATE_VECTORS_T>
  measure_vect_t compute_h_of_x(const PARTIAL_STATE_VECTORS_T&... x) const
  {
    static_assert(sizeof...(PARTIAL_STATE_VECTORS_T) == kNbKeys);
    // this is implementation specific
    // if linear, this is the  sum of the part_h_of_part_x (that are part_h \times part_x)
    // In some nonlinear cases, it may still be develop as  \sum part_h_of_x
    return compute_sum_of_part_h_of_part_x(x..., std::make_index_sequence<kNbKeys> {});
  }

  template <typename... PARTIAL_STATE_VECTORS_T>
  measure_vect_t compute_sum_of_part_h_of_part_x(const PARTIAL_STATE_VECTORS_T&... x) const
  {
    // WARNING:  proves the need of the start index for each key ??
    std::make_index_sequence<kNbKeys> I {};
    return (std::get<I>(keys_set).compute_part_h_of_part_x(x) + ...);
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
      , keys_set(init_tuple_keys(keys_id, rho))
      , keyIdToTupleIdx(map_keyid(keys_id))
  {
  }

  measure_vect_t compute_rosie() const   // rho*z = rosie !
  {                                      // TODO: move as a constant member
    return rho * z;
  }

  // NOTE: no compute_b explicitely for now, until the mix NL Lin of a var is
  // understood

  // \|Ax-b\|^2_2
  double compute_error(const state_vector_t& x)
  {
    return (rho * compute_h_of_x(x) - compute_rosie()).norm();
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


// print static information of a factor label
template <typename FT>
constexpr void factor_print()
{
  std::cout << FT::kFactorLabel << '\n';
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n' << "\tKeys (in order):\n";

  // traverse statically the tuple of keys data
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

  traverse_tup(fact.keys_set);

  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << "\n\t\t { ";
  for (int i = 0; i < FT::kMeasureComponentsName.size(); i++)
    std::cout << FT::kMeasureComponentsName[i] << ": " << fact.z[i] << "  ";
  std::cout << "}\n\t\t Cov: [ " << fact.z_cov.format(CommaInitFmt) << " ] \n";


  std::cout << "\t----- \n";
}

#endif
