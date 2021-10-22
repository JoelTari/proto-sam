#ifndef SAM_BOOKKEEPER_H_
#define SAM_BOOKKEEPER_H_

#include <vector>
#include <string>
#include <map>

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
    std::vector<std::string> keys; // NOTE: assumed to be coherent with the 
    // TODO: add stuff as needed here

};

struct Bookkeeper{

  void add_key();
  void add_factor();

  std::pair<std::string, int> getKeyInfos();


  // TODO: below should be private

  int number_of_keys;
  int aggr_dim_keys;
  int aggr_dim_mes;
  int number_of_factors;

  // std::vector< std::string > ordered_keys; // NOTE: feature order not active

  std::map<std::string, KeyInfo> key_to_infos;
  std::map<std::string, FactorInfo> factor_to_infos;

};


#endif
