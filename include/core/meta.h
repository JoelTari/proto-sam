#ifndef SAM_META_H_
#define SAM_META_H_

#include <array>
// #include <cstring>
#include <string>
#include <string_view>

// namespace
// {
//------------------------------------------------------------------//
//                         KeyMeta Template                         //
//------------------------------------------------------------------//
// template <typename DerivedKeyMeta, typename KEY_T, typename TANGENT_SPACE_T, const char* NAME, const char... ORDERRED_COMPONENT_NAMEs[]>
template <typename DerivedKeyMeta, const char* NAME, typename KEY_T, typename TANGENT_SPACE_T=KEY_T, const char... ORDERRED_COMPONENT_NAMEs[]>
struct KeyMeta
{
  inline static constexpr const char* kKeyName = NAME;
  // static constexpr const std::size_t  kN {DIM}; // DoF (or dim of the tangent space)

  constexpr static std::size_t compute_kN()
  {
    // WARNING: Eigen assumption
      return TANGENT_SPACE_T::RowsAtCompileTime;
    // if constexpr (std::is_base_of_v<Eigen::MatrixBase<TANGENT_SPACE_T>,TANGENT_SPACE_T>)
  }

  static constexpr const std::size_t  kN = compute_kN(); // DoF (or dim of the tangent space)

  // maps the idx to the component name. e.g. component[0] returns "x" etc..
  static constexpr const std::array<const char*, sizeof...(ORDERRED_COMPONENT_NAMEs)> components {
      ORDERRED_COMPONENT_NAMEs...};

  using key_t= KEY_T; // might be a vector, SE2 SO3 etc..
  using tangent_space_t = TANGENT_SPACE_T;
  
  // trivial meta ? true or false
  static constexpr bool kTrivialMeta = std::is_same_v<key_t,tangent_space_t>;

  // type enunciation to play well with type_trait convention
  // using type = KeyMeta<DerivedKeyMeta, KEY_T,TANGENT_SPACE_T, NAME, DIM, ORDERRED_COMPONENT_NAMEs...>;
  using type = KeyMeta<DerivedKeyMeta, NAME, KEY_T,TANGENT_SPACE_T, ORDERRED_COMPONENT_NAMEs...>;

  // retrieve components statically
  // TODO: move to printer class (single responsibility principle)
  template <const char* COMPONENT>
  static double get_component(const key_t & keyvalue)
  {
    return DerivedKeyMeta::template get_component_impl<COMPONENT>(keyvalue);
  }

  // dynamic version
  static double get_component(const char* component, const key_t & keyvalue)
  {
    return DerivedKeyMeta::get_component_impl(component,keyvalue);
  }

  // TODO: add some meta about the covariance of the key once feature visibility is improved
};

// Euclidian space: the tangent space manifold is the space of key
template<typename DerivedKeyMeta, const char* NAME, typename KEY_T, const char... ORDERRED_COMPONENT_NAMEs[]>
using EuclidKeyMeta = KeyMeta<DerivedKeyMeta,NAME,KEY_T,KEY_T,ORDERRED_COMPONENT_NAMEs...>;

//------------------------------------------------------------------//
//                       MeasureMeta Template                       //
//------------------------------------------------------------------//
template <typename DerivedMeasureMeta,
          typename MEASURE_T,
          const char* NAME,
          std::size_t DIM,
          const char*... ORDERRED_COMPONENT_NAMEs>
struct MeasureMeta
{
  inline static constexpr const char* kMeasureName {
      NAME};   // "linear-translation" "rigid-transformation2"
               // "rigid-transformation3"
  static constexpr std::size_t kM {DIM};

  // maps the idx to the component name. e.g. component[0] returns "dx" etc..
  static constexpr std::array<const char*, sizeof...(ORDERRED_COMPONENT_NAMEs)> components
      = std::array<const char*, sizeof...(ORDERRED_COMPONENT_NAMEs)> {ORDERRED_COMPONENT_NAMEs...};

  using type = MeasureMeta<MEASURE_T, DerivedMeasureMeta, NAME, DIM, ORDERRED_COMPONENT_NAMEs...>;

  // measure type : is often a vector, but could be also manifold such SE(n) type (e.g. from scan matcher in pose graphs problems)
  using measure_t = MEASURE_T;

  // Get a subset of themeasure by component (e.g. 'y' of a absolute position measurement, or 't' of a SE2 measure)
  // This necessarily defers to the derived class
  template <const char* COMPONENT>
  static auto get_component(const measure_t & measure)
  {
    // NOTE: not tested yet
    // std::format("a");
    static_assert( 
        (
         (*COMPONENT == *ORDERRED_COMPONENT_NAMEs)  || 
          ...
        )
        //, std::format("a")
        );
    // return static_cast<DerivedMeasureMeta*>(this)->template get_component_impl<COMPONENT>(measure);
    return DerivedMeasureMeta::template get_component_impl<COMPONENT>(measure);
  }
  // dynamic version
  static double get_component(const char* component, const measure_t & keyvalue)
  {
    return DerivedMeasureMeta::get_component_impl(component,keyvalue);
  }
};
// }   // namespace

#endif
