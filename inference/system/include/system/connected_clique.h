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
 
#include <core/clique.h>
#include <core/DenseLinearBelief.h>


class ConnectedClique
{
    /**
    * @brief Is the clique ready (all incomming separators received)
    *
    * @return 
    */
  bool isReady();

    /**
    * @brief Pointers to separators. 
    *       TODO: allow runtime modifications
    *         
    */
  std::vector<ConnectedClique*> neighbors;

    /**
    * @brief Received separators
    */
  std::vector<
    std::map<ConnectedClique*,DenseLinearBelief>> incomming_separators;


    /**
    * @brief compute an outgoing message for a neighbor
    *       This implies that the other incomming message are availabe
    *
    * @return 
    */
  DenseLinearBelief send_message(const ConnectedClique* ptr_connected_clique);

    /**
    * @brief Get incomming messages from a subset of cliques neighbors
    *
    * @param neighbors
    *
    * @return 
    */
  std::vector<DenseLinearBeliefs> get_incomming_messages_from(const std::vector<ConnectedClique*> & neighbors);
  
    /**
    * @brief Routine to get all incomming messages/separators
    *
    * @return 
    */
  DenseLinearBelief get_all_incomming_messages();

    /**
    * @brief Get final belief. The clique must be ready.
    *
    * @return 
    */
  DenseLinearBelief get_final_belief();


    /**
    * @brief Update belief (e.g. after relinearisation)
    *
    * @return 
    */
  DenseLinearBelief update_belief();

  
};



/**
  * @brief Root Adapter
  *        A connected clique is chosen as the root to allow
  *        forward and, optionaly, backward message passing.
  */
RootAdapter
{
  RootAdapter(const ConnectedClique* ptr_clique)
    : clique(connected_clique)
    { }

  /**
  * @brief Pointer to a Connected Clique that is upgraded as root
  *        The adapter does not have ownership of the clique
  */
  ConnectedClique* clique; // the clique that is root
  
  
  /**
    * @brief Forward-only message passing
    *         The result are contained within the clique.
    *         Only the root is ready by the end.
    */
  void forward_message_passing();

  /**
  * @brief Full message passing (forward and backward)
  */
  void full_message_passing();

};
