#include <core/factor.h>

class Clique
{
    /**
    * @brief vars id
    *        dimension is deduced easily at higher level
    */
    std::vector<std::string> vars_id;

    /**
    * @brief factors id
    *        dimension is deduced easily at higher level
    */
    std::vector<std::string> factors_id;

    /**
    * @brief factors
    */
    std::vector<BaseFactor> factors;

    /**
    * @brief belief made of the multiplication of factors beliefs
    */
    DenseLinearBelief internal_belief;

    /**
    * @brief Constructor
    *       TODO: check that it works with different kind of factors
    *       TODO: e.g. SE2 motion, SE2 pose prior 
    *
    * @param std::vector
    */
    Clique(const std::vector<BaseFactor> )

    // TODO: Might need this later
    // get_Posterior_Mean();
};
