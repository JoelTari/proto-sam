#ifndef FACTORV3_H_
#define FACTORV3_H_

#include <array>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <string_view>
#include <tuple>
#include <utility>




// some helper: "tie" a string to a type (a key meta here)
template <const char Role[], typename T>
struct StrTie
{
  static constexpr const char* kRole {Role};
  using type = T;
};

  // template <const char KeyName[],
  //           const char Role[],
  //           size_t     DimKey,
  //           size_t     DimMes>
  // struct KeyContextConduct   // in the context of a factor
  // {
  //   static constexpr const char* kKeyName {KeyName};
  //   static constexpr const char* kRole {Role};
  //   static constexpr size_t      kN = DimKey;
  //   std::string                  key_id;   // TODO: add const keyword
  //   // non static, not const
  //   using process_matrix_t = Eigen::Matrix<double, DimKey, kN>;
  //   using measure_vect_t   = Eigen::Matrix<double, DimMes, 1>;
  //   process_matrix_t A;   // NOTE: normed or not normed
  //   measure_vect_t   b;
  // };


template<typename DerivedKCC, typename KEYMETA, size_t DimMes, const char* ContextRole>
struct KeyContextualConduct : KEYMETA
{
  // static constexpr const char* kKeyName {};
  // static constexpr const char* kRole {Role};
  // static constexpr size_t      kN = DimKey;
    static constexpr const char* kRole {ContextRole};
  // non static but const
  std::string                  key_id;   // TODO: add const keyword
  // non static, not const
  using process_matrix_t = Eigen::Matrix<double, DimMes, KEYMETA::kN>;
  using measure_vect_t   = Eigen::Matrix<double, DimMes, 1>;
  process_matrix_t A;   // NOTE: normed or not normed
  // measure_vect_t   b;
    // TODO: const rho = ... (det at ctor)

    process_matrix_t
      compute_part_A()
  {
    // in linear it would just be a getter to  rho * H
    // in nonlinear, set_linearization_point must occur before
    return static_cast<DerivedKCC*>(this)->compute_part_A_impl();
  }

  //KeyContextualConduct() // TODO: CTOR must receive rho and key_id

};

//------------------------------------------------------------------//
//                         Factor Template                          //
//------------------------------------------------------------------//
template <typename DerivedFactor,
          const char FactorLabel[],
          typename MEASURE_META,
          typename... KeyConducts>
class FactorV3
{
  public:
  using measure_vect_t = Eigen::Matrix<double, MEASURE_META::kM, 1>;
  using measure_cov_t
      = Eigen::Matrix<double, MEASURE_META::kM, MEASURE_META::kM>;
  // using keys_ids_t = std::array<std::string, sizeof...(KeyTs)>;
  static constexpr const char* kFactorLabel {FactorLabel};
  static constexpr size_t      kN = (KeyConducts::kN + ...);
  static constexpr size_t      kM = MEASURE_META::kM;
  static constexpr size_t      kNbKeys = sizeof...(KeyConducts);
  // make a tuple of KeySet.  Michelin *** vaut le détour.
  using KeysSet_t
      // = std::tuple<KeyContextConduct<KeyTs::type::kKeyName,
      //                                               KeyTs::kRole,
      //                                               KeyTs::type::kN,
      //                                               kM>...>;
    = std::tuple<KeyConducts ...>;

  static constexpr const char* kMeasureName {MEASURE_META::kMeasureName};
  static constexpr std::array<const char*, kM> kMeasureComponentsName
      = MEASURE_META::components;
  const std::string    factor_id;      // fill at ctor
  const measure_vect_t measure_vect;   // fill at ctor
  const measure_cov_t  measure_cov;    // fill at ctor
  KeysSet_t keys_set;   // a tuple of the structures of each keys (dim, id,
                      // process matrix)
  // id must be filled at ctor
  // partial process matrices and linpoint (if any) are runtime mutable (except
  // if the factor is linear)

  std::map<std::string, size_t> keyIdToTupleIdx;   // fill at ctor

  // ctor helper
  template <size_t I = 0>
  void set_map_keyid(const std::array<std::string, kNbKeys>& keys_id)
  {
    if constexpr (I == kNbKeys)
      return;
    else
    {
      keyIdToTupleIdx[keys_id[I]] = I;
      std::get<I>(keys_set).key_id = keys_id[I];
      set_map_keyid<I + 1>(keys_id);
    }
  }

  // the ctor
  // template <const char* ... VarStrArgs>
  FactorV3(const std::string&                               factor_id,
           const measure_vect_t&                            mes_vect,
           const measure_cov_t&                             measure_cov,
           const std::array<std::string, kNbKeys>& keys_id)
      : measure_vect(mes_vect)
      , measure_cov(measure_cov)
      , factor_id(factor_id)
  {
    set_map_keyid(keys_id);
  }

  measure_vect_t compute_b()
  {
    measure_vect_t b;
    // TODO: b = rho * z  (linear)
    return b;
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
    std::cout << "\t\t+ Key Nature: " << KT::kKeyName
              << ".  Role: " << KT::kRole << '\n';

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
    std::cout << "\t\t+ Key Nature: " << KT::kKeyName
              << ".  Role: " << KT::kRole << ". Id: " << std::get<I>(tup).key_id
              << '\n';
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
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n'
            << "\tKeys (in order):\n";

  // traverse statically the tuple of keys data
  traverse_tup<typename FT::KeysSet_t>();

  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << " [ ";
  for (const auto& comp : FT::kMeasureComponentsName) std::cout << comp << " ";
  std::cout << "]\n";


  std::cout << "\t----- \n";
}

// This is the non static version
template <typename FT>
void factor_print(const FT& fact)
{
  std::cout << FT::kFactorLabel << " - id : " << fact.factor_id << '\n';
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n'
            << "\tKeys (in order):\n";

  traverse_tup(fact.keys_set);

  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << " [ ";
  for (const auto& comp : FT::kMeasureComponentsName) std::cout << comp << " ";
  std::cout << "]\n";


  std::cout << "\t----- \n";
}

#endif
