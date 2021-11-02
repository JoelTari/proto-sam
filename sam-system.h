#ifndef SAM_SYSTEM_H_
#define SAM_SYSTEM_H_

#include "bookkeeper.h"
#include "config.h"
#include "utils.h"

#include <functional>
// #include "definitions.h"

#include <eigen3/Eigen/Dense>
#include <tuple>
#include <type_traits>
#include <vector>

namespace SAM
{
  template <typename FACTOR_T,
            typename... FACTORS_Ts>   // I need at least one type of factor
  class SamSystem
  {
    public:
    SamSystem() {}

    template <typename FT, typename... Args>
    void register_new_factor(const std::string&             factor_id,
                             const typename FT::var_keys_t& keys,
                             Args&&... args)
    {
      static_assert(
          std::is_same_v<FT,
                         FACTOR_T> || (std::is_same_v<FT, FACTORS_Ts> || ...),
          "This type of factor doesnt exist ");

      // check if factor id exists already
      // TODO: consistent management failure (throw ? return value false ?
      // std::optional ?)
      if (this->bookkeeper_.factor_id_exists(factor_id)) return;


      // recursively find, at compile time, the corresponding container (amongst
      // the ones in the tuple) to emplace back the factor FT
      place_factor_in_container<0, FT>(factor_id,
                                       keys,
                                       std::forward<Args>(args)...);
    }

    template <std::size_t I = 0, typename FT, typename... Args>
    void place_factor_in_container(const std::string&             factor_id,
                                   const typename FT::var_keys_t& keys,
                                   Args&&... args)
    {
      // beginning of static recursion (expanded at compile time)
      if constexpr (I == S_)
        return;
      else
      {
        // if this is the type we are looking for, emplace back in
        if constexpr (std::is_same_v<FT, factor_type_in_tuple_t<I>>)
        {
          // At this point the factor is pushed in the system so we should
          // update bookkeeper
          //  1. for each key
          //      - check if the keys exists if key doesn't exist, add it (and
          //      it's meta)
          //      - add this factor id to the keyinfo list
          //  2. add an element in FactorInfo

          // for each key of this factor.   keys.size() is same as
          // FT::Meta_t::kNumberOfVars
          for (std::size_t i = 0; i < keys.size(); i++)
          {
            // check if the key exists
            try
            {
              this->bookkeeper_.getKeyInfos(keys[i]);
            }
            catch (int e)
            {
              // add the key, the variable size is accessed via the factor Meta
              //              in the same order of the
              this->bookkeeper_.add_key(keys[i], FT::Meta_t::kVarsSizes[i]);
            }
            // each key has a list of factors_id that it is connected, so add
            // this factor_id to it
            this->bookkeeper_.add_factor_id_to_key(keys[i], factor_id);
          }
          // add the factor_id with its infos in the bookkeeper
          // last argument is a conversion from std::array to std::vector
          this->bookkeeper_.add_factor(factor_id,
                                       FT::Meta_t::kAggrVarDim,
                                       FT::Meta_t::kMesDim,
                                       {keys.begin(), keys.end()});


          std::get<I>(this->all_factors_tuple_)
              .emplace_back(factor_id, keys, std::forward<Args>(args)...);

// Debug consistency check of everything
#if ENABLE_RUNTIME_CONSISTENCY_CHECKS
          // 1. checking if the bookkeeper is consistent with itself (systemInfo
          // vs whats on the std::maps)
          if (!this->bookkeeper_.are_dimensions_consistent())
            std::cerr << "Bookkeeper internally not consistent.";
          // 2. checking if the bookkeeper is consistent with the tuples of
          // vector holding the factors
          if (!this->is_system_consistent())
            std::cerr << "SAM system is not consistent.";
#endif
        }
        // recursion :  compile time call
        place_factor_in_container<I + 1, FT>(factor_id,
                                             keys,
                                             std::forward<Args>(args)...);
      }
    }

    // // this works for 1 type of std::vector<FT>, but not for the whole tuple
    // template <std::size_t I=0, typename FT>
    // void for_each_factor( std::function< void(const FT & func) >)
    // {
    //     for (const auto& factor : std::get<I>(this->all_factors_tuple_))
    //     {
    //       func(factor);
    //     }
    // }

    // TODO: can we do it by adding a lambda inside the for{} ?
    template <std::size_t I = 0>
    void loop_over_factors()
    {
      if constexpr (I == S_)
        return;
      else
      {
        for (const auto& factor : std::get<I>(this->all_factors_tuple_))
        {
          // ShortPrintFactorInfo(factor);
        }
        // compile-time recursion
        loop_over_factors<I + 1>();
      }
    }


    void smooth_and_map()
    {
      // scoped timer
      PROFILE_FUNCTION();

      SystemInfo      system_infos = this->bookkeeper_.getSystemInfos();
      int             M            = system_infos.aggr_dim_mes;
      int             N            = system_infos.aggr_dim_keys;
      Eigen::MatrixXd A(M, N);
      Eigen::VectorXd b(M);
      // loop the factors, and fill A and b
      //
      // solve  A,b
      auto Xmap  = A.colPivHouseholderQr().solve(b);
      auto Xmap2 = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

      // declarer la matrice d'info H


      // declarer/reutiliser un pt de linearisation

      // tant que le delta est signicatif
      //      si l'ordering peut changer, mettre la matrice d'info H = At.A a
      //      zero mettre le vecteur d'info At.b  a  zero iterer sur les
      //      facteurs en linearisant et remplissant la mat info resoudre dX* =
      //      inv(At.A)At.b  , on a le MAP mettre a jour le pt de linearisation


      //     REMARQUES:
      // Le bookkeeper intervient bcp dans cette fonction, principalement pour,
      // a chaque pas de linearisation savoir ou a quel idx doit aller la sortie
      // Ai, AiT.bi (sachant qu'ils peuvent etre decoupe en morceaux)
      //
      // ex: soit Ai \in (2,5) et Hi \in (5,5) et bi \in (2,2)  repesentant 2
      // var X3 et l1 de taille respective 3 et 2.
      //     soit H \in (50,50) la matrice d'info du pb complet pour laquelle X3
      //     occupe 15,16,17 et L1 occupe 40,41
      //
      //    alors:
      //      Hi[0:2,0:2] doit s'additionner dans H[15:17,15:17]
      //      Hi[3:4,3:4] doit s'additionner dans H[40:41,40:41]
      //      Hi[0:2,3:4] doit s'additionner dans H[15:17,40:41] (reciproquement
      //      pour les transposees de Hi et H) Et pour AiT.bi, les idx [0:2]
      //      vont dans b[15:17],
      //                          et  [3:4] vont dans b[40:41]
      //
      //  Chaque factor nous donne le lien   state (ex: x3)  -> idxes dans Hi /
      //  Ait.bi, ces liens d'idx sont constants Le bookkeeper donne le lien
      //  state (ex: x3)  -> idxes dans H / At.b | ces liens sont fonction de
      //  l'ordre
      //                                                                              et donc peuvent changer


      //    REMARQUES 2:
      //  il faut gerer l'ordering, et modifier en consequence le bookkeeping
    }

    // TODO: remove, too specific
    // Eigen::VectorXd mean_;
    // Eigen::MatrixXd covariance_;

#if ENABLE_DEBUG_TRACE
    /**
     * @brief dummy iteration over tuple of std::vector
     */
    template <size_t I = 0>
    void IterFactorInfos()
    {
      if constexpr (I == S_)
        return;
      else
      {
        std::cout << factor_type_in_tuple_t<I>::kFactorCategory
                  << "  -- Number of elements registered : "
                  << std::get<I>(this->all_factors_tuple_).size() << "\n";

        // if this collection is not empty, print the details of the factors
        if (std::get<I>(this->all_factors_tuple_).size() > 0)
        {
          for (const auto& factor : std::get<I>(this->all_factors_tuple_))
          {
            std::cout << "\t";
            ShortPrintFactorInfo(factor);
            std::cout << "\n";
          }
        }
        // recursion :  compile time call
        IterFactorInfos<I + 1>();
      }
    }
#endif

#if ENABLE_RUNTIME_CONSISTENCY_CHECKS
    bool is_system_consistent()
    {
      // TODO:
      return true;
    }
#endif

    private:
    Bookkeeper bookkeeper_;

    // there's at least one factor, the rest are expanded
    std::tuple<std::vector<FACTOR_T>, std::vector<FACTORS_Ts>...>
        all_factors_tuple_;

    // how many different types of factor there are
    constexpr static const size_t S_
        = std::tuple_size<decltype(all_factors_tuple_)>::value;


    /**
     * @brief Gets the factor type of the Ith tuple element. Use case: get some
     * static info about the factors such as :
     * - factor type name of 0th factor collection :
     * `factor_type_in_tuple_t<0>::kFactorCategory`
     * - some meta (dimensions) about the Ith factors collection:
     * `factor_type_in_tuple_t<I>::Meta_t::kMesDim`.
     *
     * The advantage is that it doesn't matter if the std::vector is not holding
     * any factor (or many)
     *
     * @tparam I
     */
    template <size_t I>
    using factor_type_in_tuple_t = typename std::
        tuple_element<I, decltype(all_factors_tuple_)>::type::value_type;
  };

};   // namespace SAM
#endif
