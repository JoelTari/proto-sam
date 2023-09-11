/* 
 * Copyright 2023 AKKA Technologies and LAAS-CNRS (joel.tari@akka.eu) 
 * 
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by 
 * the European Commission - subsequent versions of the EUPL (the "Licence"); 
 * You may not use this work except in compliance with the Licence. 
 * You may obtain a copy of the Licence at: 
 * 
 * https://joinup.ec.europa.eu/software/page/eupl 
 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the Licence is distributed on an "AS IS" basis, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the Licence for the specific language governing permissions and 
 * limitations under the Licence. 
 */
 
#include "anchor2d/anchor2d.h"
#include "relative-matcher-2d/relative-matcher-2d.h"
#include "system/sam-system.h"

#include <fstream>
#include <iostream>

// using Solver_t = typename sam::Inference::SolverSparseQR;
// using Solver_t = typename sam::Inference::SolverSparseNaive;
// using Solver_t = typename sam::Inference::SolverSparseSimplicialLLT;
// using Solver_t = typename sam::Inference::SolverSparsePardisoLLT;
// using Solver_t = typename sam::Inference::SolverSPQR;
using Solver_t          = typename sam::Inference::SolverSparseSupernodalLLT;
using InferenceSystem_t = typename sam::Inference::
    SparseSystem<Solver_t, sam::Factor::Anchor2d, sam::Factor::RelativeMatcher2d>;

//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  std::string argId;
  if (argc > 1) { argId = argv[1]; }
  else { argId = "unnamed"; }

  // 
  sam_utils::JSONLogger::Instance().beginSession(
      "mainstdin",
      argId,
      sam_utils::currentDateTime());   // second argument: some sort of runtime prefix: date,
                                       // dataset ID etc.. program name pb_id (e.g. M3500) date

  Json::Value json_meta;
  json_meta["program"] = __FILE__ ;
  json_meta["date"] = sam_utils::currentDateTime();
  if (argc > 2)
  {
    json_meta["hash commit"] = argv[2];
  }
  json_meta["problem id"] = argId;
  json_meta["threads available"] = Eigen::nbThreads();
  sam_utils::JSONLogger::Instance().add_custom_field("meta", json_meta);
    
  // capture standard input
  Json::Value rootJson; 
  std::cin >> rootJson;
  // alternative : getting a filename as argument
  // std::ifstream file = std::ifstream(argv[3]);
  // file >> rootJson;

#if ENABLE_DEBUG_TRACE
  std::cout << "\n\n Declaring a sam system:\n";
#endif

  {

    PROFILE_FUNCTION();
    auto syst   = InferenceSystem_t(argId);
    int  fcount = 0;

    {
      PROFILE_SCOPE("integrates factors");
      std::vector<std::string> vfid;
      std::vector<Eigen::Vector2d> vz;
      std::vector<Eigen::Matrix2d> vSigma;
      std::vector<std::array<std::string,2>> vKeysid;
      for (const auto& mesJson : rootJson)
      {
        if (mesJson["type"] == "anchor")
        {
          Eigen::Vector2d     z {mesJson["value"]["x"].asDouble(), mesJson["value"]["y"].asDouble()};
          std::vector<double> cov_vect;
          for (const auto& c : mesJson["covariance"]) cov_vect.push_back(c.asDouble());
          Eigen::Map<Eigen::Matrix2d> Sigma(cov_vect.data());
          std::stringstream           fid;   // factor id
          fid << "f" << fcount;
          syst.register_new_factor<sam::Factor::Anchor2d>(fid.str(),
                                                          z,
                                                          Sigma,
                                                          {mesJson["vars_id"][0].asCString()},false);
          fcount++;
        }
        else if (mesJson["type"] == "linear-translation")
        {
          Eigen::Vector2d z {mesJson["value"]["dx"].asDouble(), mesJson["value"]["dy"].asDouble()};
          std::vector<double> cov_vect;
          for (const auto& c : mesJson["covariance"]) cov_vect.push_back(c.asDouble());
          Eigen::Map<Eigen::Matrix2d> Sigma(cov_vect.data());
          std::stringstream           fid;
          fid << "f" << fcount;
          vfid.push_back(fid.str());
          vz.push_back(z);
          vSigma.push_back(Sigma);
          vKeysid.push_back({mesJson["vars_id"][0].asCString(), mesJson["vars_id"][1].asCString()});
          // syst.register_new_factor<sam::Factor::RelativeMatcher2d>(
          //     fid.str(),
          //     z,
          //     Sigma,
          //     {mesJson["vars_id"][0].asCString(), mesJson["vars_id"][1].asCString()},false);
          fcount++;
        }
        else
          throw std::runtime_error("measure type not supported");
      }
      syst.register_factors_in_bundle<sam::Factor::RelativeMatcher2d>(vfid, vz, vSigma, vKeysid);
    }

    try
    {
      // ::sam::Inference::OptimOptions<::sam::Inference::SolverSparseNaive> opts(false);
      syst.sam_optimise(/* opts */);
    }
    catch (const char* e)
    {
      std::cerr << "SLAM algorithm failed. Reason: " << e << '\n';
    }

    Json::Value json_graph = SystemJsonify<InferenceSystem_t>::jsonify_graph(syst);
    sam_utils::JSONLogger::Instance().writeGraph(json_graph);

  }
  // print logger
  std::cout << sam_utils::JSONLogger::Instance().out();
  return 0;
}
