#ifndef SAM_SYSTEM_H_
#define SAM_SYSTEM_H_

#include "definitions.h"
// #include "odom_vel_factor.h"
// #include "range_bearing_factor.h"
#include "bookkeeper.h"
#include <type_traits>

namespace SAM
{

/***
 * Mainly a class to keep track of associations between variables and index positions in matrix representations
***/
class Bookkeeper
{
};


template <typename FACTOR_T, typename ...FACTORS_Ts>  // I need at least one type of factor
class SamSystem
{
public:

    SamSystem()
    {
    }

    // TODO: Is std::size_t I=0 necessary ?
    template <std::size_t I = 0, typename FT>
    void register_new_factor(const FT & factor)
    {
        static_assert(   std::is_same_v<FT, FACTOR_T> || 
                         (std::is_same_v<FT, FACTORS_Ts> || ...)
                         , "This type of factor doesnt exist ");

        place_factor_in_container(factor);

    }

    template <std::size_t I = 0, typename FT>
    void place_factor_in_container(const FT & factor)
    {
        // beginning of static recursion (expanded at compile time)
        if constexpr(I == S_)
            return;
        else
        {
            //if constexpr( std::is_same_v<FT, typename std::tuple_element<I,decltype(this->all_factors_tuple_)>::type::value_type > )    
            if constexpr( std::is_same_v<FT, factor_type_in_tuple_t<I> > )    
            {
                std::get<I>(this->all_factors_tuple_).push_back(factor);
            }
            // recursion :  compile time call
            place_factor_in_container<I+1,FT>(factor);
        }
    }


    void smooth_and_map()
    {
        // declarer la matrice d'info H

        // declarer/reutiliser un pt de linearisation

        // tant que le delta est signicatif
        //      si l'ordering peut changer, mettre la matrice d'info H = At.A a zero
        //      mettre le vecteur d'info At.b  a  zero
        //      iterer sur les facteurs en linearisant et remplissant la mat info
        //      resoudre dX* = inv(At.A)At.b  , on a le MAP
        //      mettre a jour le pt de linearisation


        //     REMARQUES:
        // Le bookkeeper intervient bcp dans cette fonction, principalement pour, a chaque pas de linearisation 
        // savoir ou a quel idx doit aller la sortie Ai, AiT.bi (sachant qu'ils peuvent etre decoupe en morceaux)
        //
        // ex: soit Ai \in (2,5) et Hi \in (5,5) et bi \in (2,2)  repesentant 2 var X3 et l1 de taille respective 3 et 2.
        //     soit H \in (50,50) la matrice d'info du pb complet pour laquelle X3 occupe 15,16,17 et L1 occupe 40,41     
        //
        //    alors:  
        //      Hi[0:2,0:2] doit s'additionner dans H[15:17,15:17] 
        //      Hi[3:4,3:4] doit s'additionner dans H[40:41,40:41] 
        //      Hi[0:2,3:4] doit s'additionner dans H[15:17,40:41] (reciproquement pour les transposees de Hi et H)
        //      Et pour AiT.bi, les idx [0:2] vont dans b[15:17],
        //                          et  [3:4] vont dans b[40:41]
        //
        //  Chaque factor nous donne le lien   state (ex: x3)  -> idxes dans Hi / Ait.bi, ces liens d'idx sont constants
        //  Le bookkeeper donne le lien        state (ex: x3)  -> idxes dans H / At.b | ces liens sont fonction de l'ordre
        //                                                                              et donc peuvent changer


        //    REMARQUES 2:
        //  il faut gerer l'ordering, et modifier en consequence le bookkeeping 
    }

    Eigen::VectorXd mean_;
    Eigen::MatrixXd covariance_;


private:
    Bookkeeper bookkeeper_;

    // there's at least one factor, the rest are expanded
    std::tuple<std::vector<FACTOR_T> , std::vector<FACTORS_Ts>...> all_factors_tuple_ ;

    // how many different types of factor there are
    constexpr static const size_t   S_ = std::tuple_size<decltype(all_factors_tuple_)>::value;

    template <size_t I>
    using factor_type_in_tuple_t = typename std::tuple_element<I,decltype(all_factors_tuple_)>::type::value_type ; 
};

};
#endif
