#ifndef META_H_
#define META_H_

#include <array>

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

#endif
