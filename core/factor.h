#ifndef SAM_FACTOR_H_
#define SAM_FACTOR_H_

#include "config.h"

#include <array>
#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <initializer_list>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

// assert the validity of variables size list against the total aggregate
// dimension variable
//       also, a variable size cannot be zero
/**
 * @brief Assert the coherence of variables size list VS total aggregate
 * dimension variable. The sum of the former must equal the later.
 *
 * @tparam EXPECTED_VAR_TOTAL_DIM
 * @tparam T
 * @param slist
 *
 * @return
 */
template <int EXPECTED_VAR_TOTAL_DIM, typename T>
constexpr bool IsSizesOfVarsValid(T slist)
{
  // check if sizes elements are in valid range
  int sum = 0;
  for (const auto& e : slist)
  {
    sum += e;
    if (e <= 0) return false;
  }
  // check if the sum of the sizes is coherent with total dimension size
  if (sum != EXPECTED_VAR_TOTAL_DIM) return false;
  // checks are coherent
  return true;
}

template <int EXPECTED_AGGR_DIM_VAR, typename T>
constexpr bool IsVarIdxRangesValid(T ranges)
{
  // TODO: complete
  return true;
}

/**
 * @brief Given the sizes of the variables, infer the start and end positions of
 * each variable in the joint variable array
 *
 * @tparam NB_VARS
 * @param sizes_of_vars
 *
 * @return
 */
template <size_t NB_VARS>
static constexpr std::array<std::array<int, 2>, NB_VARS>
    GenerateIndexesRanges(const std::array<int, NB_VARS>& sizes_of_vars)
{
  std::array<std::array<int, 2>, NB_VARS> result {};

  int idx = 0;
  int i   = 0;
  for (const auto& vsize : sizes_of_vars)
  {
    result[i] = std::array<int, 2> {idx, idx + vsize - 1};
    i++;
    idx = idx + vsize;
  }
  return result;
}

/**
 * @brief metafunction holding the structure of a factor type
 *
 * @tparam NB_VARS
 * @tparam VAR_TOTAL_DIM
 * @tparam VAR_SIZES
 * @tparam NB_VARS
 */
template <int                             NB_VARS,
          int                             VAR_TOTAL_DIM,
          const std::array<int, NB_VARS>& VAR_SIZES,
          int                             MES_DIM>
struct FactorMetaInfo
{
  /// Number of groups of variables of the factor (ex: {x1,x2} -> 2)
  static constexpr int kNumberOfVars {NB_VARS};
  /// Aggregate dimension of the variable(s) combined (ex: SE2 odom -> 6)
  ///  = dimension of the factor's (aggregate) variable vector
  static constexpr int kAggrVarDim {VAR_TOTAL_DIM};
  /// Array of the respective dimension of each variable (ex: SE2 odom -> [3,3])
  static constexpr auto kVarsSizes {VAR_SIZES};
  /// Array of the idx start & end of each variable in the aggregate factor
  /// variable array. (ex: SO2 odom -> [[0,2],[3,5]])
  /* static constexpr auto varIdxRanges {VAR_IDX_RANGES_T}; */
  /// Dimension of the measurement vector (ex: SO2 odom -> 3, bearing-only -> 1)
  static constexpr int kMesDim {MES_DIM};
  // Number of scalar elements in the lhs (matrix measurement)
  static constexpr int kNbScalarElements {kMesDim*kAggrVarDim};

  // for each variable, the range in the state. Ex: SE2 : -> [[0,2],[3,5]]
  // this will work in conjunction with the variable_position & variable_range
  // members in the factor class std::array<std::array<int, 2>, NB_VARS_T>
  static constexpr std::array<std::array<int, 2>, kNumberOfVars> kVarIdxRanges {
      GenerateIndexesRanges<kNumberOfVars>(kVarsSizes)};

  // The meta data stored statically is expressive in nature to serve
  // effiencitly all runtime requirements without overhead. Howerver, the above
  // metadata is filled-in by an user who wants to instantiate a custom factor
  // type. Coherence issues might arise, if for example, the total sum of
  // variable sizes does not equal the total dimension.
  // This motivates the asserts.
  static_assert(IsSizesOfVarsValid<kAggrVarDim>(kVarsSizes),
                "FACTOR META ASSERT: the list of variable dimension is not "
                "coherent with the total dimension");
};

/**
 * @brief
 *
 * @tparam Derived  Static polymorphic trick
 * @tparam META_INFO_T meta information (dimensions of various entities)
 * @tparam FACTOR_TYPE_NAME[] the name of the factor type (odometry,
 * range-bearing, initialPose etc...)
 */
template <typename Derived, typename META_INFO_T, const char FACTOR_CATEGORY[]>
class BaseFactor
{
  public:
  // access meta info through Meta_t type
  using Meta_t = META_INFO_T;
  // jacobian matrix type
  using jacobian_matrix_t
      = Eigen::Matrix<double, Meta_t::kAggrVarDim, Meta_t::kMesDim>;
  // using jacobian_matrices_t
  //   = std::array<Eigen::Matrix, std::size_t _Nm>
  // measure vector type
  using measure_vector_t = Eigen::Matrix<double, Meta_t::kMesDim, 1>;
  // measure covariance type
  using measure_covariance_matrix_t
      = Eigen::Matrix<double, Meta_t::kMesDim, Meta_t::kMesDim>;
  // state vector type
  using state_vector_t = Eigen::Matrix<double, Meta_t::kAggrVarDim, 1>;
  // array of string: variable keys
  using var_keys_t = std::array<std::string, Meta_t::kNumberOfVars>;

  // the type of factor (eg odometry, range bearing, linear)
  static constexpr const char* kFactorCategory = FACTOR_CATEGORY;

  // factor id
  const std::string factor_id;

  // the actual measurement made
  const measure_covariance_matrix_t mes_covariance;
  const measure_vector_t            mes_vector; // z

  // NOTE: it would be advantageous to split into bloc jacobian matrices (one of each key), but that would be a tuple difficult to implement (blocs may not have the same number of cols)
    // TODO: use a combination constexpr & tuple_cat
  jacobian_matrix_t A; 
  // jacobian_matrices_t AA;
  measure_vector_t b; // dont confuse with mes_vector: z

  state_vector_t linearization_point;

  const var_keys_t variables;

  const std::map<std::string, int> variable_position
      = LinkVariablesToStateVectorIdx();

  /**
   * @brief Base Factor consturctor
   *
   * @param factor_id str id of the factor (eg "f0")
   * @param variable_names array str of the variables (or keys) (eg ["x2","l5"])
   */
  BaseFactor(const std::string& factor_id, const var_keys_t& variable_names, const measure_vector_t measure, const measure_covariance_matrix_t & covariance)
      : variables(variable_names)
      , factor_id(factor_id)
      , mes_vector(measure)
      , mes_covariance(covariance)
  {
#if ENABLE_DEBUG_TRACE
    std::cout << "Create " << this->kFactorCategory << " factor " << factor_id
              << " with variables : ";
    for (const auto& varname : this->variables) std::cout << varname << " ";
    std::cout << "\n";
#endif
  }

  std::tuple<jacobian_matrix_t,measure_vector_t> compute_A_b()
  {
    // in linear it would just be a getter
    // in nonlinear, set_linearization_point must occur before
    return static_cast<Derived*>(this)->compute_A_b_impl();
  }

  // for the nonlinears
  void set_linearization_point(const state_vector_t & lin_point)
  {
    linearization_point = lin_point;
  }

  private:
  std::map<std::string, int> LinkVariablesToStateVectorIdx()
  {
    std::map<std::string, int> m;
    int                        i = 0;
    for (const auto& e : this->variables) m[e] = i++;
    return m;
  };
};

/**
 * @brief Returns a string given an array of string
 *
 * @tparam S
 * @param array_of_variable_names
 * @param separator
 * @param ""
 *
 * @return
 */
template <size_t S>
std::string StringifyArrayOfStrings(
    const std::array<std::string, S>& array_of_variable_names,
    const std::string&                separator = ",")
{
  std::stringstream ss;
  // for (const auto & str : array_of_variable_names)
  for (int i = 0; i < S; i++)
  {
    if (i) ss << separator;
    ss << array_of_variable_names[i];
  }
  return ss.str();
  // can be tested with:
  // auto test_arr = std::array<std::string,4>({"x45","x448","x85","Xs8"});
  // std::cout << stringify_array_of_strings(test_arr) << "\n";
}

#if ENABLE_DEBUG_TRACE
template <typename FACTOR_T>
void ShortPrintFactorInfo(const FACTOR_T& factor)
{
  std::cout << FACTOR_T::kFactorCategory << " " << factor.factor_id
            << " :: Scope : " << StringifyArrayOfStrings(factor.variables);
}
#endif

#endif
