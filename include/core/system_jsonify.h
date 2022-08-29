#pragma once

#include "core/config.h"
#include "utils/utils.h"
#include "utils/tuple_patterns.h"

// TODO:  wrap with #if ENABLE_JSON_OUTPUT ?? I think yes

template <typename SAM_SYS>
class SystemJsonify
{
  public: 
    static Json::Value jsonify_graph(const SAM_SYS & sys)
    {
      Json::Value json_graph;
      json_graph["header"] = jsonify_header(sys.get_system_infos());
      json_graph["factors"] = jsonify_factors(sys.get_factors_histories());
      json_graph["marginals"] = jsonify_marginals(sys.get_marginals_histories());

      // std::cout << jsonify_factors(sys.get_factors_histories()) << std::endl;
      // std::cout << jsonify_header(sys.get_system_infos()) << std::endl;

      return json_graph;
    }

  private:
    
    static Json::Value jsonify_header(const typename SAM_SYS::system_info_t & sysinfo )
    {
      Json::Value json_header;
      json_header["robot_id"] = sysinfo.agent_id;
      json_header["seq"] = 0; // TODO:
      json_header["base_unit"] = 0.15;
      Json::Value quadratic_errors;
      for (auto qerr : sysinfo.quadratic_error)
        quadratic_errors.append(qerr);
      json_header["quadratic_errors"] = quadratic_errors;
      json_header["Rnnz"] = sysinfo.Rnnz;
      json_header["Hnnz"] = sysinfo.Hnnz;
      // some compile time information
      using def_t = sam::definitions::CompiledDefinitions;
      json_header["bla"] = def_t::blas;
      json_header["bla_vendor_mkl"] = def_t::bla_vendor_mkl;
      json_header["aggressively_optimised"] = def_t::optimised;
      json_header["openmp"] = def_t::openmp;
      json_header["timer"] = def_t::timer;
      json_header["json_output"] = def_t::json_output;
      json_header["runtime_checks"] = def_t::runtime_checks;
      json_header["debug_trace"] = def_t::debug_trace;
      // TODO: variable order  :  "variable_order"
      return json_header;
    }

    static Json::Value jsonify_factors(const typename SAM_SYS::factors_histories_t & factors_histories_list)
    {
      // input: a tuple of vectors of factors
      // TODO: use std::apply

      Json::Value json_factors;
      sam_tuples::for_each_in_const_tuple(
      factors_histories_list.factors_histories_container, 
      [&json_factors](const auto & map_factor_history, auto NIET)
      {
        using factor_history_t = typename std::remove_const_t<std::decay_t<decltype(map_factor_history)>>::mapped_type;
        using factor_t = typename factor_history_t::Factor_t;
        
        for (const auto & pair : map_factor_history)
        {
          const auto & factor_h = pair.second;
          Json::Value json_factor;
          json_factor["factor_id"] = factor_h.factor_id;
          json_factor["type"]      = factor_history_t::kFactorLabel; // TODO: rename type to label
          json_factor["measure"]   = factor_history_t::kMeasureName;
          // vars_id
          Json::Value json_vars_id;
          for (const auto & key_id : factor_h.vars_id) json_vars_id.append(key_id);
          json_factor["vars_id"] = json_vars_id;
          json_factor["MAPerror"] = factor_h.norms.back();           // TODO: rename norm
          Json::Value json_norms;
          for (const auto & norm: factor_h.norms) json_norms.append(norm);
          json_factor["norms"]= json_norms;
          // report the measurement value, component by component
          Json::Value json_measure; // TODO: component name could be given statically so that the string comparaison would occur at compile time (micro opt)
          for (std::size_t i=0;i< factor_t::kMeasureComponentsName.size(); i++ )
          {
            json_measure[factor_t::kMeasureComponentsName[i]] 
              = factor_t::MeasureMeta_t::get_component(factor_t::kMeasureComponentsName[i],factor_h.z);
          }
          json_factor["measure"] = json_measure;
          // Append at the json list of factors
          json_factors.append(json_factor);
        }
      });

      return json_factors;
    }

    static Json::Value jsonify_marginals(const typename SAM_SYS::marginals_histories_container_t & marginals_history_list)
    {
      // TODO: std::apply
      Json::Value json_marginals;
      sam_tuples::for_each_in_const_tuple(
      marginals_history_list.marginal_history_tuple,
      [&json_marginals](const auto & map_marginal_histories, auto NIET)
      {
        using marginal_history_t = typename std::remove_const_t<std::decay_t<decltype(map_marginal_histories)>>::mapped_type;
        using marginal_t = typename marginal_history_t::Marginal_t;
        using keymeta_t = typename marginal_t::KeyMeta_t;
        for (const auto & pair : map_marginal_histories)
        {
          Json::Value json_marginal;
          auto marg_hist = pair.second;
          json_marginal["var_id"] = marg_hist.var_id;
          json_marginal["category"] = keymeta_t::kKeyName;
          Json::Value json_mean;
          for (std::size_t i =0; i< keymeta_t::components.size();i++)
          {
            // TODO: component name could be given statically so that the string comparaison would occur at compile time (micro opt)
            json_mean[keymeta_t::components[i]]  
              = keymeta_t::get_component(keymeta_t::components[i],marg_hist.iterative_means.back());
          }
          json_marginal["mean"] = json_mean;
          // iterative means
          Json::Value iterative_means;
          for (std::size_t j = 0 ; j < marg_hist.iterative_means.size(); j++)
          {
            Json::Value ith_json_mean;
            for (std::size_t i =0; i< keymeta_t::components.size();i++)
            {
              ith_json_mean[keymeta_t::components[i]] 
                = keymeta_t::get_component(keymeta_t::components[i],marg_hist.iterative_means[j]);
            }
            iterative_means.append(ith_json_mean);
          }
          json_marginal["iterative_means"] = iterative_means;
          // iterative covariance
          json_marginal["covariance"]["sigma"].append(std::get<0>(marg_hist.iterative_covariances.back())[0]);
          json_marginal["covariance"]["sigma"].append(std::get<0>(marg_hist.iterative_covariances.back())[1]);
          json_marginal["covariance"]["rot"]= std::get<1>(marg_hist.iterative_covariances.back());
          Json::Value iterative_covariances;
          for (std::size_t j=0; j< marg_hist.iterative_covariances.size();j++)
          {
            Json::Value covariance;
            covariance["sigma"].append(std::get<0>(marg_hist.iterative_covariances[j])[0]);
            covariance["sigma"].append((std::get<0>(marg_hist.iterative_covariances[j])[1]));
            covariance["rot"] = std::get<1>(marg_hist.iterative_covariances[j]);
            iterative_covariances.append(covariance);
          }
          json_marginal["iterative_covariances"] = iterative_covariances;
          json_marginals.append(json_marginal);
        }
      });
      return json_marginals;
    }


};
