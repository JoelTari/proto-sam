#ifndef SAM_BOOKKEEPER_H_
#define SAM_BOOKKEEPER_H_

#include <map>
#include <string>
#include <string_view>
#include <vector>

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
  std::vector<std::string> keys;   // NOTE: assumed to be coherent with the
                                   // TODO: add stuff as needed here
};

/**
 * @brief Infos on the SAM system
 */
struct SystemInfo
{
  int number_of_keys;
  int aggr_dim_keys;
  int aggr_dim_mes;
  int number_of_factors;
};

class Bookkeeper
{
  public:
  void add_key();
  void add_factor();

  KeyInfo getKeyInfos(const std::string& key) const
  {
    if (auto it {key_to_infos.find(key)}; it != std::end(this->key_to_infos))
    {
      return it->second;
    }
    else
    {
      throw;   // TODO: consistent failure management
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
      throw;   // TODO: consistent failure management
    }
  }
  SystemInfo getSystemInfos() const { return this->system_info_; }

  bool factor_id_exists(const std::string& factor_id) const
  {
    if (auto it {this->factor_to_infos.find(factor_id)};
        it != std::end(this->factor_to_infos))
    {
      // Use `structured binding` to get the key
      // and value.
      const auto& [key, value] {*it};

      // Grab either the key or value stored in the pair.
      // The key is stored in the 'first' variable and
      // the 'value' is stored in the second.
      const auto& mkey {it->first};
      const auto& mvalue {it->second};

      // That or just grab the entire pair pointed
      // to by the iterator.
      const auto& pair {*it};

      return true;
    }
    else
    {
      // key was not found
      return false;
    }
  }


  private:
  SystemInfo system_info_;

  // std::vector< std::string > ordered_keys; // NOTE: feature order not active

  std::map<std::string, KeyInfo>    key_to_infos;
  std::map<std::string, FactorInfo> factor_to_infos;
};


#endif
