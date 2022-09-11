#pragma once

#include "system/config.h"
#include "marginal/marginal.h"

#if ENABLE_JSON_OUTPUT
#include <json/json.h>

template <typename SAM_SYS>
class SystemJsonify
{
  public:
  static Json::Value jsonify_graph(const SAM_SYS& sys)
  {
    Json::Value json_graph;
    json_graph["header"]    = jsonify_header(sys.get_system_infos(),sys.nbSequence);
    // FIX: header:  take OptStats, OptimOpts, SolverStats, Header(seq, sys name (cosntexpr), agent id )
    json_graph["factors"]   = jsonify_factors(sys.get_all_factors());
    json_graph["marginals"] = jsonify_marginals(sys.get_marginals());

    return json_graph;
  }

  private:
  static Json::Value jsonify_header(const typename SAM_SYS::system_info_t& sysinfo, int nbSequence)
    // FIX: header:  take OptStats, OptimOpts, SolverStats, Header(seq, sys label (cosntexpr), agent id )
  {
    Json::Value json_header;
    json_header["robot_id"]  = sysinfo.agent_id;
    json_header["seq"]       = nbSequence;
    json_header["base_unit"] = 0.15;
    Json::Value quadratic_errors;
    for (auto qerr : sysinfo.quadratic_error) quadratic_errors.append(qerr);
    json_header["quadratic_errors"] = quadratic_errors;
    json_header["Rnnz"]             = sysinfo.Rnnz;
    json_header["Hnnz"]             = sysinfo.Hnnz;
    // some compile time information (system agnostic)
    using def_t                           = sam::definitions::CompiledDefinitions;
    json_header["bla"]                    = def_t::blas;
    json_header["bla_vendor_mkl"]         = def_t::bla_vendor_mkl;
    json_header["aggressively_optimised"] = def_t::optimised;
    json_header["openmp"]                 = def_t::openmp;
    json_header["timer"]                  = def_t::timer;
    json_header["json_output"]            = def_t::json_output;
    json_header["runtime_checks"]         = def_t::runtime_checks;
    json_header["debug_trace"]            = def_t::debug_trace;
    // TODO: variable order  :  "variable_order"
    return json_header;
  }

  static Json::Value jsonify_factors(const typename SAM_SYS::Wrapped_Factor_t& wrapped_factors)
  {
    // input: a tuple of vectors of factors

    return std::apply(
        [](const auto&... wfactors)
        {
          Json::Value json_factors;
          // jsonify one type after another, append the result in json_factor
          (jsonify_map_of_factors(wfactors, json_factors), ...);
          return json_factors;
        },
        wrapped_factors);
  }

  static Json::Value
      jsonify_marginals(const typename SAM_SYS::Marginals_t::Marginals_Data_t& marginals_data_tuple)
  {
    return std::apply(
        [](const auto&... map_of_wmarginals)
        {
          Json::Value json_marginals;
          // jsonify one type after another, append the result in json_marginals
          (jsonify_map_of_marginals(map_of_wmarginals, json_marginals), ...);
          return json_marginals;
        },
        marginals_data_tuple);
  }


  //------------------------------------------------------------------//
  //                           lower level                            //
  //------------------------------------------------------------------//
  template <typename MAP_OF_WMARG_T>
  static void jsonify_map_of_marginals(const MAP_OF_WMARG_T& map_of_wmarginals,
                                       Json::Value&          json_marginals_out)
  {
    using wrapped_marginal_t = typename MAP_OF_WMARG_T::mapped_type;
    using marginal_t         = typename wrapped_marginal_t::Marginal_t;
    using keymeta_t          = typename marginal_t::KeyMeta_t;
    for (const auto& [key_id, wrapped_marginal] : map_of_wmarginals)
    {
      Json::Value json_marginal;
      // auto marg_hist = pair.second;
      json_marginal["var_id"]   = key_id;
      json_marginal["category"] = keymeta_t::kKeyName;
      Json::Value json_mean;
      for (std::size_t i = 0; i < keymeta_t::components.size(); i++)
      {
        // TODO: component name could be given statically so that the string comparaison would occur
        // at compile time (micro opt)
        json_mean[keymeta_t::components[i]]
            = keymeta_t::get_component(keymeta_t::components[i], wrapped_marginal.marginal.mean);
      }
      json_marginal["mean"] = json_mean;
      // covariance
      if (wrapped_marginal.marginal.covariance.has_value())
      {
        json_marginal["covariance"] = jsonify_marginal_covariance(wrapped_marginal.marginal);
      }
      // historic marginals (from the init point given to the solver to)
      //  If system is linear, there is only one historic mean
      Json::Value historic_marginals;
      for (std::size_t j = 0; j < wrapped_marginal.marginal_histories.size(); j++)
      {
        Json::Value ith_json_marginal;
        Json::Value ith_json_mean;
        for (std::size_t i = 0; i < keymeta_t::components.size(); i++)
        {
          ith_json_mean[keymeta_t::components[i]]
              = keymeta_t::get_component(keymeta_t::components[i],
                                         wrapped_marginal.marginal_histories[j].mean);
        }
        ith_json_marginal["mean"] = ith_json_mean;
        if (wrapped_marginal.marginal_histories[j].covariance.has_value())
        {
          ith_json_marginal["covariance"]
              = jsonify_marginal_covariance(wrapped_marginal.marginal_histories[j]);
        }
        historic_marginals.append(ith_json_marginal);
      }
      json_marginal["historic_marginals"] = historic_marginals;
      // TODO: historic covariances
      // put in result
      json_marginals_out.append(json_marginal);
    }
  }

  template <typename MARG_T>
  static Json::Value jsonify_marginal_covariance(const MARG_T& marg)
  {
    Json::Value json_cov;
    auto [sigma, rot] = ::sam::Marginal::get_visual_2d_covariance(marg.covariance.value());
    json_cov["sigma"].append(sigma.first);
    json_cov["sigma"].append(sigma.second);
    json_cov["rot"] = rot;
    return json_cov;
  }

  template <typename VECTOR_FACTORS_T>
  static void jsonify_map_of_factors(const VECTOR_FACTORS_T& wfactors,
                                     Json::Value&            json_factors_out)
  {
    using wrapped_factor_t = typename VECTOR_FACTORS_T::value_type;
    using factor_t         = typename VECTOR_FACTORS_T::value_type::Factor_t;

    for (const auto& wfactor : wfactors)
    {
      const auto& factor = wfactor.factor;
      Json::Value json_factor;
      json_factor["factor_id"] = factor.factor_id;
      json_factor["type"]      = factor_t::kFactorLabel;   // TODO: rename type to label
      json_factor["measure"]   = factor_t::kMeasureName;
      // vars_id
      Json::Value json_vars_id;
      for (const auto& key_id : factor.get_array_keys_id()) json_vars_id.append(key_id);
      json_factor["vars_id"] = json_vars_id;

      auto wfactor_data = wfactor.get_current_point_data();

      json_factor["MaP_residual"] = wfactor_data.norm;
      Json::Value json_norms;
      // TODO: reverse vector, or use iterator
      for (const auto& norm : wfactor.norm_history) json_norms.append(norm);
      json_factor["previous_norms"] = json_norms;
      // report the measurement value, component by component
      Json::Value json_measure;   // TODO: component name could be given statically so that the
                                  // string comparaison would occur at compile time (micro opt)
      for (std::size_t i = 0; i < factor_t::kMeasureComponentsName.size(); i++)
      {
        json_measure[factor_t::kMeasureComponentsName[i]]
            = factor_t::MeasureMeta_t::get_component(factor_t::kMeasureComponentsName[i], factor.z);
      }
      json_factor["measure"] = json_measure;
      // Append at the json list of factors
      json_factors_out.append(json_factor);
    }
  }
};
#endif   // ENABLE_JSON_OUTPUT
