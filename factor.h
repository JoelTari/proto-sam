#include <array>
#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

/* template <auto X> */
/* struct ValueIdentity{ */
/*   static constexpr auto value = X; */
/* }; */
/* template<auto X> */
/* inline constexpr auto ValueIdentity_v = ValueIdentity<X>::value; */

// assert the validity of variables size list against the total aggregate
// dimension variable
//       also, a variable size cannot be zero
template <int EXP_DIM_TOTAL_T, typename T>
constexpr bool is_valid_varsSizes(T slist)
{
  // check if sizes elements are in valid range
  int sum = 0;
  for (const auto & e : slist) {
    sum += e;
    if (e <= 0)
      return false;
  }
  // check if the sum of the sizes is coherent with total dimension size
  if (sum != EXP_DIM_TOTAL_T)
    return false;
  // checks are coherent
  return true;
}

template <int EXP_AGGR_DIM_VAR, typename T>
constexpr bool is_valid_varIdxRanges(T ranges)
{
  return true;
}

template <size_t NB_VARS_T>
constexpr std::array<std::array<int, 2>, NB_VARS_T>
generate_indexes_ranges(const std::array<int, NB_VARS_T> & varsSizes)
{
  std::array<std::array<int, 2>, NB_VARS_T> result{};

  int idx = 0;
  int i   = 0;
  for (const auto & vsize : varsSizes) {
    result[i] = std::array<int, 2>{idx, idx + vsize - 1};
    i++;
    idx = idx + vsize;
  }
  return result;
}

// metafunction (in a programming sense) about the meta (in a math sense) to
// describe the strutural dimensions of a factor
template <int                                NB_VARS_T,
          int                                VAR_TOTAL_DIM_T,
          const std::array<int, NB_VARS_T> & VAR_SIZES_T,
          int                                MES_DIM_T>
struct FactorMetaInfo {
  /// Number of groups of variables of the factor (ex: {x1,x2} -> 2)
  static constexpr auto numberOfVars{NB_VARS_T};
  /// Aggregate dimension of the variable(s) combined (ex: SO2 odom -> 6)
  ///  = dimension of the factor's (aggregate) variable vector
  static constexpr auto aggrVarDim{VAR_TOTAL_DIM_T};
  /// Array of the respective dimension of each variable (ex: SO2 odom -> [3,3])
  static constexpr auto varsSizes{VAR_SIZES_T};
  /// Array of the idx start & end of each variable in the aggregate factor
  /// variable array. (ex: SO2 odom -> [[0,2],[3,5]])
  /* static constexpr auto varIdxRanges {VAR_IDX_RANGES_T}; */
  /// Dimension of the measurement vector (ex: SO2 odom -> 3, bearing-only -> 1)
  static constexpr auto mesDim{MES_DIM_T};

  // automation attempt
  /* static constexpr std::array<int[2], NB_VARS_T> varIdxRanges2 =
   * generate_indexes_ranges(varsSizes); */
  static constexpr auto varIdxRanges = generate_indexes_ranges(varsSizes);

  // The meta data stored statically is expressive in nature to serve
  // effiencitly all runtime requirements without overhead. Howerver, the above
  // metadata is filled-in by an user who wants to instantiate a custom factor
  // type. Coherence issues might arise, if for example, the total sum of
  // variable sizes does not equal the total dimension.
  // This motivates the asserts.
  static_assert(is_valid_varsSizes<aggrVarDim>(varsSizes),
                "FACTOR META ASSERT: the list of variable dimension is not "
                "coherent with the total dimension");
};

template <typename Derived, typename META_INFO_T>
class BaseFactor
{
public:
  // access meta info through meta_t type
  using meta_t = META_INFO_T;
  // jacobian matrix type
  using jacobian_matrix_t
      = Eigen::Matrix<double, META_INFO_T::aggrVarDim, META_INFO_T::mesDim>;
  // measure vector type
  using measure_vector_t = Eigen::Matrix<double, META_INFO_T::mesDim, 1>;
  // measure covariance type
  using measure_covariance_matrix_t
      = Eigen::Matrix<double, META_INFO_T::mesDim, META_INFO_T::mesDim>;
  // state vector type
  using state_vector_t = Eigen::Matrix<double, META_INFO_T::aggrVarDim, 1>;

  // the actual measurement made
  const measure_covariance_matrix_t mes_covariance;
  const measure_vector_t            mes_vector;

  jacobian_matrix_t jacobian;

  state_vector_t linearization_point;

  const std::array<std::string, META_INFO_T::numberOfVars> variables;

  const std::map<std::string, int> variable_position
      = link_variables_to_state_vector_idx();

  // constructor
  BaseFactor(
      const std::array<std::string, META_INFO_T::numberOfVars> & variable_names)
      : variables(variable_names)
  {
    std::cout << "creating factor with meta ";
    for (const auto & varname : this->variables) std::cout << varname << " ";
    std::cout << "\n";
  }

  // TODO call static polymorphic methods here

private:
  std::map<std::string, int> link_variables_to_state_vector_idx()
  {
    std::map<std::string, int> m;
    int                        i = 0;
    for (const auto & e : this->variables) m[e] = i++;
    return m;
  };
};

// purely helper
template <size_t S>
std::string stringify_array_of_strings(
    const std::array<std::string, S> & array_of_variable_names)
{
  std::stringstream ss;
  for (const auto & str : array_of_variable_names) ss << str;
  return ss.str();
}
