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
