#include <core/connected_clique.h>

class CliqueUndirectedTree
{

    /**
    * @brief map of connected cliques
    *       Neighbor relations are stored inside the 
    */
    std::unordered_map<ConnectedClique*>;

    /**
    * @brief add a new leaf in the tree
    *
    * @param new_clique
    */
    void add_leaf(const ConnectedClique* const new_clique);

    /**
    * @brief remove a leaf
    *
    * @param ConnectedClique
    */
    void remove_leaf(const ConnectedClique );

    /**
    * @brief insert a new factor in an existing clique
    *
    * @param factor
    * @param factor_scope
    */
    void add_factor_in_clique(const BaseFactor * factor, std::vector<std::string> factor_scope );

    // TODO: Reshuffle only when doing a loop closure (needed feature: subtree extraction)

    // TODO: queries subtree by variable (not easy)
    // TODO:   - query a minimal path between 2 variables (give back 2 cliques)
    // TODO:   - query a minimal path between n variables (give back all the cliques)
};
