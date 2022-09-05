// #include "core/marginal.h"
#include "core/sam-system.h"
#include "anchor2d/anchor2d.h"
#include "relative-matcher-2d/relative-matcher-2d.h"

#include<iostream>
#include<fstream>

//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  // std::cout << "nb of threads : " << Eigen::nbThreads() << '\n';
  std::string argId;
  if (argc > 1) { argId = argv[1]; }
  else
  {
    argId = "unnamed";
  }
  std::stringstream session_name;
  // session_name << "mainstdin.cpp_sam_" << argId; 
  // std::string result_filename
  //     = sam_utils::currentDateTime() + "_results_" + argId + ".json";
  sam_utils::JSONLogger::Instance().beginSession("mainstdin", argId, sam_utils::currentDateTime()); // second argument: some sort of runtime prefix: date, dataset ID etc..
                                                                                       // program name
                                                                                       // pb_id (e.g. M3500)
                                                                                       // date

  Json::Value rootJson;   // the desired 'container' (to be filled)
                          // initialized as null
  
  // read measurements inputs
  if (argc > 2)
  {
    PROFILE_SCOPE("read file", sam_utils::JSONLogger::Instance());
    // getting a filename as argument
    std::ifstream file = std::ifstream(argv[2]);
    file >> rootJson;
    // Json::CharReaderBuilder rbuilder;
    // std::unique_ptr<Json::CharReader> const reader(rbuilder.newCharReader());
  }
  else
  {
    // or receiving the json directly in std input
    // The parse is not necessary here
    std::cin >> rootJson;
  }

#if ENABLE_DEBUG_TRACE
  std::cout << "\n\n Declaring a sam system:\n";
#endif

  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());   // TODO: remove those calls
  using system_t = typename sam::System::SamSystem<sam::Factor::Anchor2d, sam::Factor::RelativeMatcher2d>;
  auto syst   = sam::System::SamSystem<sam::Factor::Anchor2d, sam::Factor::RelativeMatcher2d>(argId);
  int  fcount = 0;

  {
    PROFILE_SCOPE("integrates factors", sam_utils::JSONLogger::Instance());
    for (const auto& mesJson : rootJson)   // TODO: template it
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
                                               {mesJson["vars_id"][0].asCString()});
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
        syst.register_new_factor<sam::Factor::RelativeMatcher2d>(
            fid.str(),
            z,
            Sigma,
            {mesJson["vars_id"][0].asCString(), mesJson["vars_id"][1].asCString()});
        fcount++;
      }
      else
        throw std::runtime_error("measure type not supported");
    }
  }

  // std::this_thread::sleep_for(std::chrono::seconds(1));

  try
  {
    syst.sam_optimise();
  }
  catch (const char* e)
  {
    std::cerr << "SLAM algorithm failed. Reason: " << e << '\n';
  }
  
  Json::Value json_graph = SystemJsonify<system_t>::jsonify_graph(syst);
  sam_utils::JSONLogger::Instance().writeGraph(json_graph);

  // sam_utils::JSONLogger::Instance().endSession();
  std::cout << sam_utils::JSONLogger::Instance().out();
  return 0;
}
