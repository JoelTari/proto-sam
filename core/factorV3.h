#include <array>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <string_view>
#include <tuple>
#include <utility>


//------------------------------------------------------------------//
//                         KeyMeta Template                         //
//------------------------------------------------------------------//
template <const char  NAME[],
          std::size_t DIM,
          const char... ORDERRED_COMPONENT_NAMEs[]>
struct KeyMeta
{
  static constexpr const char* kKeyName {
      NAME};   // "position2d" "pose2d" etc...
  static constexpr size_t kN {DIM};

  // maps the idx to the component name. e.g. component[0] returns "x" etc..
  static constexpr const std::array<const char*,
                                    sizeof...(ORDERRED_COMPONENT_NAMEs)>
      components {ORDERRED_COMPONENT_NAMEs...};
  // static constexpr const std::array<const char*, 2> ttc = {"dz","dw"};
  // NOT SUPPORTING THE component name to idx for now

  // type enunciation to play well with type_trait convention
  using type = KeyMeta<NAME, DIM, ORDERRED_COMPONENT_NAMEs...>;
};


//------------------------------------------------------------------//
//                       MeasureMeta Template                       //
//------------------------------------------------------------------//
template <const char  NAME[],
          std::size_t DIM,
          const char*... ORDERRED_COMPONENT_NAMEs>
struct MeasureMeta
{
  static constexpr const char* kMeasureName {
      NAME};   // "linear-translation" "rigid-transformation2"
               // "rigid-transformation3"
  static constexpr size_t kM {DIM};

  // maps the idx to the component name. e.g. component[0] returns "dx" etc..
  static constexpr std::array<const char*, sizeof...(ORDERRED_COMPONENT_NAMEs)>
      components
      = std::array<const char*, sizeof...(ORDERRED_COMPONENT_NAMEs)> {
          ORDERRED_COMPONENT_NAMEs...};
  // components {"da","db"};
  static constexpr const std::array<const char*, 2> ttc = {"dz", "dw"};

  using type = MeasureMeta<NAME, DIM, ORDERRED_COMPONENT_NAMEs...>;
};


// some helper: "tie" a string to a type (a key meta here)
template <const char Role[], typename T>
struct StrTie
{
  static constexpr const char* kRole {Role};
  using type = T;
};

namespace __internalKey
{
  template <const char KeyName[],
            const char Role[],
            size_t     DimKey,
            size_t     DimMes>
  struct ContextualKeyInfo   // in the context of a factor
  {
    static constexpr const char* kKeyName {KeyName};
    static constexpr const char* kRole {Role};
    static constexpr size_t      kN = DimKey;
    std::string            keyId;
    // lin point ?
    using process_matrix_t = Eigen::Matrix<double, DimKey, kN>;
    process_matrix_t process_matrix;
  };
}   // namespace __internalKey

//------------------------------------------------------------------//
//                         Factor Template                          //
//------------------------------------------------------------------//
template <const char FactorLabel[], typename MEASURE_META, typename... KeyTs>
class FactorV3
{
  public:
  using measure_vect_t = Eigen::Matrix<double, MEASURE_META::kM, 1>;
  using measure_cov_t
      = Eigen::Matrix<double, MEASURE_META::kM, MEASURE_META::kM>;
  // using keys_ids_t = std::array<std::string, sizeof...(KeyTs)>;
  static constexpr const char* kFactorLabel {FactorLabel};
  static constexpr size_t      kN = (KeyTs::type::kN + ...);
  static constexpr size_t      kM = MEASURE_META::kM;
  // make a tuple of KeyIn.  Michelin *** vaut le détour.
  using KeysSet
      = std::tuple<__internalKey::ContextualKeyInfo<KeyTs::type::kKeyName,
                                                    KeyTs::kRole,
                                                    KeyTs::type::kN,
                                                    kM>...>;

  static constexpr const char* kMeasureName {MEASURE_META::kMeasureName};
  static constexpr std::array<const char*, kM> kMeasureComponentsName
      = MEASURE_META::components;
  const std::string    factor_id;      // fill at ctor
  const measure_vect_t measure_vect;   // fill at ctor
  const measure_cov_t  measure_cov;    // fill at ctor
  KeysSet keys_set;   // a tuple of the structures of each keys (dim, id,
                      // process matrix)
  // id must be filled at ctor
  // partial process matrices and linpoint (if any) are runtime mutable (except
  // if the factor is linear)

  std::map<std::string, size_t> keyIdToTupleIdx;   // fill at ctor

  // ctor helper
  template <size_t I = 0> 
  void set_map_keyid(const std::array<std::string,sizeof...(KeyTs)>& keys_id)
  {
     if constexpr (I == sizeof...(KeyTs))
      return;
    else
    {
      keyIdToTupleIdx[ keys_id[I] ] = I;
      std::get<I>(keys_set).keyId = keys_id[I];
      set_map_keyid<I+1>(keys_id);
    }

  }

  // the ctor
  // template <const char* ... VarStrArgs>
  FactorV3(const std::string&    factor_id,
           const measure_vect_t& mes_vect,
           const measure_cov_t&  measure_cov,
           const std::array<std::string,sizeof...(KeyTs)>& keys_id):
    measure_vect(mes_vect)
    ,measure_cov(measure_cov)
    ,factor_id(factor_id)
  {
    set_map_keyid(keys_id);
  }
};

//------------------------------------------------------------------//
//                       Meta Instantiations                        //
//------------------------------------------------------------------//
// Key meta
namespace __MetaKeyPosition
{
  static constexpr const char position[] = "position";
  static constexpr const char x[]        = "x";
  static constexpr const char y[]        = "y";
  using MetaKeyPosition_t                = KeyMeta<position, 2, x, y>;
}   // namespace __MetaKeyPosition
using MetaKeyPosition_t = __MetaKeyPosition::MetaKeyPosition_t;

// meta linear translation measure
namespace __MetaLinearTranslation
{
  static constexpr const char linearTranslation[] = "linear-translation";
  static constexpr const char dx[]                = "dx";
  static constexpr const char dy[]                = "dy";
  using MetaMeasureLinearTranslation_t
      = MeasureMeta<linearTranslation, 2, dx, dy>;
}   // namespace __MetaLinearTranslation
using MetaMeasureLinearTranslation_t
    = __MetaLinearTranslation::MetaMeasureLinearTranslation_t;

// meta absolute position measure
namespace __MetaMeasureAbsolutePosition
{
  // namespace is necessary so that the vars names (x) doesnt pollute global
  // scope
  static constexpr const char absolute_position[] = "absolute_position";
  static constexpr const char x[]                 = "x";
  static constexpr const char y[]                 = "y";
  using MetaMeasureAbsolutePosition_t = MeasureMeta<absolute_position, 2, x, y>;
}   // namespace __MetaMeasureAbsolutePosition
using MetaMeasureAbsolutePosition_t = __MetaMeasureAbsolutePosition::
    MetaMeasureAbsolutePosition_t;   // unwind namespace

// factor instantiation from templates
// 1. anchor
static constexpr const char anchor[]     = "anchor";
static constexpr const char anchor_var[] = "unique var";
using AnchorFactor                       = FactorV3<anchor,
                              MetaMeasureAbsolutePosition_t,
                              StrTie<anchor_var, MetaKeyPosition_t>>;
// 2. linear translation
static constexpr const char LinearTranslationLabel[] = "linear translation";
static constexpr const char observee_var[]           = "observee";
static constexpr const char observer_var[]           = "observer";
using LinearTranslationMetaFactor
    = FactorV3<LinearTranslationLabel,
               MetaMeasureLinearTranslation_t,
               StrTie<observee_var, MetaKeyPosition_t>,
               StrTie<observer_var, MetaKeyPosition_t>>;


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
template <typename TUP, size_t I =0>
void traverse_tup(const TUP& tup)
{
  if constexpr (I == std::tuple_size<TUP>::value) { return; }
  else
  {
    using KT = std::tuple_element_t<I, TUP>;
    std::cout << "\t\t+ Key Nature: " << KT::kKeyName
              << ".  Role: " << KT::kRole <<  ". Id: " << std::get<I>(tup).keyId <<'\n';
    std::cout << "\t\t\t A:\n" << std::get<I>(tup).process_matrix << '\n';

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
  traverse_tup<typename FT::KeysSet>();

  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << " [ ";
  for (const auto& comp : FT::kMeasureComponentsName) std::cout << comp << " ";
  std::cout << "]\n";


  std::cout << "\t----- \n";
}

// This is the non static version
template <typename FT>
void factor_print(const FT & fact)
{
  std::cout << FT::kFactorLabel << " - id : "<< fact.factor_id << '\n';
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n'
            << "\tKeys (in order):\n";

  traverse_tup(fact.keys_set);

  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << " [ ";
  for (const auto& comp : FT::kMeasureComponentsName) std::cout << comp << " ";
  std::cout << "]\n";


  std::cout << "\t----- \n";
}


// //------------------------------------------------------------------//
// //                               MAIN                               //
// //------------------------------------------------------------------//
// int main(int argc, char* argv[])
// {
//   // AnchorFactor A;
//   AnchorFactor::measure_vect_t m;
//   AnchorFactor::measure_cov_t cov;
//   LinearTranslationMetaFactor::measure_vect_t m2;
//   LinearTranslationMetaFactor::measure_cov_t cov2;
//
//   auto FA = AnchorFactor("f0",m,cov,{"x0"});
//   auto FB = LinearTranslationMetaFactor("f1",m2,cov2,{"x0","x1"});
//
//   std::cout << "Printing runtime infos of a factor : \n";
//   factor_print(FA);
//   factor_print(FB);
//
//   std::cout << "\nPrinting infos of a factor type (only static infos since it is just a type) : \n\n";
//   factor_print<AnchorFactor>();
//   factor_print<LinearTranslationMetaFactor>();
//
//   // std::cout << AnchorFactor::kN << '\n';
//   // std::cout << LinearTranslationMetaFactor::kN << '\n';
//
//   return 0;
// }
