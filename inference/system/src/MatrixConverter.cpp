#include "system/MatrixConverter.hpp"

using namespace sam::Inference::MatrixConverter;


Eigen::SparseMatrix<int> Sparse::Semantic::spyHessian(const Eigen::SparseMatrix<int>& spy_jacobian)
{
  PROFILE_FUNCTION();
  return spy_jacobian.transpose() * spy_jacobian;
}

Eigen::SparseMatrix<int> Sparse::Semantic::spyHessian(const Keys_Affectation_t& keys_affectation, const std::size_t nnz) 
{
  PROFILE_FUNCTION();
  std::vector<Eigen::Triplet<int>> triplets;
  triplets.reserve(nnz);
  // loop keys affectations
  for(const auto & [key,dispatch] : keys_affectation)
  {
    std::size_t i = dispatch.natural_semantic_idx;
    triplets.emplace_back(i,i,1);
    for (const auto & neigh : dispatch.neighbours)
    {
      std::size_t j = keys_affectation.find(neigh)->second.natural_semantic_idx;
      triplets.emplace_back(i,j,1);
    }
  }
  Eigen::SparseMatrix<int> H(keys_affectation.size(),keys_affectation.size());
  H.setFromTriplets(triplets.begin(),triplets.end());

  return H;
}
