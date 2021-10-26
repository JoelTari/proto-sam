#ifndef SAM_BOOKKEEPER_H_
#define SAM_BOOKKEEPER_H_

#include <map>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

// WARNING: must still decide if the Bookkeeper should be templated in the meta
// info (for now, no, it would be too complicated)

struct KeyInfo
{
  int dim;
  // int order; // NOTE: feature order not active
  std::vector<std::string> factors_id;
};

// mostly copied from the factor meta (known at compiled-time)
// and run time info (name of the vars)
struct FactorInfo
{
  int                      factor_aggr_mes_dim;
  int                      factor_aggr_key_dim;
  std::vector<std::string> keys;   // NOTE: assumed to be coherent with the
};

/**
 * @brief Infos on the SAM system
 */
struct SystemInfo
{
  int aggr_dim_keys;
  int aggr_dim_mes;
  int number_of_keys;
  int number_of_factors;
};

class Bookkeeper
{
  public:
  void add_key(const std::string& key, int key_dimension)
  {
    KeyInfo new_key_info;
    new_key_info.dim        = key_dimension;
    this->key_to_infos[key] = new_key_info;
  }
  void add_factor_id_to_key(const std::string& key,
                            const std::string& factor_id)
  {
    this->key_to_infos[key].factors_id.push_back(factor_id);
  }

  void add_factor(const std::string&              factor_id,
                  int                             aggr_key_dim,
                  int                             aggr_mes_dim,
                  const std::vector<std::string>& keys)
  {
    FactorInfo factor_info;
    factor_info.factor_aggr_key_dim  = aggr_key_dim;
    factor_info.factor_aggr_mes_dim  = aggr_mes_dim;
    factor_info.keys                 = keys;
    this->factor_to_infos[factor_id] = factor_info;
  };

  void update_system_info(int d_aggr_dim_keys,
                          int d_aggr_dim_mes,
                          int d_num_keys,
                          int d_num_factors)
  {
    // NOTE: whats could happen if
    // TODO: -> manage those cases (via tests ? run time checks ?)
    system_info_.aggr_dim_keys += d_aggr_dim_keys;
    system_info_.aggr_dim_mes += d_aggr_dim_mes;
    system_info_.number_of_keys += d_num_keys;
    system_info_.number_of_factors += d_num_factors;
  }

  KeyInfo getKeyInfos(const std::string& key) const
  {
    if (auto it {this->key_to_infos.find(key)};
        it != std::end(this->key_to_infos))
    {
      return it->second;
    }
    else
    {
      throw 1;   // TODO: consistent failure management std::optinal ??
    }
  }
  FactorInfo getFactorInfos(const std::string& factor_id) const
  {
    if (auto it {factor_to_infos.find(factor_id)};
        it != std::end(this->factor_to_infos))
    {
      return it->second;
    }
    else
    {
      throw 1;   // TODO: consistent failure management
    }
  }
  SystemInfo getSystemInfos() const { return this->system_info_; }

  bool factor_id_exists(const std::string& factor_id) const
  {
    // NOTE: ugly code ?
    if (auto it {this->factor_to_infos.find(factor_id)};
        it != std::end(this->factor_to_infos))
    {
      return true;
    }
    else
    {
      // key was not found
      return false;
    }
  }

  #define ENABLE_DEBUG 1
  #if ENABLE_DEBUG
  bool are_dimensions_consistent() const
    {
      // TODO: check if everything is right: dimensions does add up etc...
      return true;
    }
  #endif

  private:
  SystemInfo system_info_;
  std::unordered_map<std::string, KeyInfo>    key_to_infos; // TODO: map better ? 
  std::unordered_map<std::string, FactorInfo> factor_to_infos;
    // PERFORMANCE: I have way more lookups than insert/delete, so I choose unordered_map
    // PERFORMANCE: unordered has more overhead however

  // std::vector< std::string > ordered_keys; // NOTE: feature order not active
};


#endif
