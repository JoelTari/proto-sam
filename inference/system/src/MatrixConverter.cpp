#include "system/MatrixConverter.hpp"

using namespace sam::Inference::MatrixConverter;


Eigen::SparseMatrix<int> Sparse::Semantic::spyHessian(const Eigen::SparseMatrix<int>& spy_jacobian)
{
  PROFILE_FUNCTION();
  return spy_jacobian.transpose() * spy_jacobian;
}

Eigen::SparseMatrix<int> Sparse::Semantic::spyHessian(const DispatchContainer_t& keys_affectation, const std::size_t nnz) 
{
  PROFILE_FUNCTION();
  // note that the nnz input can be given as an overshoot (e.g. it might be easier to give a greater number such as the nnz of jacobian, all it does is preallocate a bit more memory than necessary)
  std::vector<Eigen::Triplet<int>> triplets;
  triplets.reserve(nnz);
  // loop keys affectations
  for(const auto & dispatch : keys_affectation.get<2>()) // random_access iter of bmi
    // performance: (M3500) looping over the hashed_unique vs random_access
    // didn't change (about 0.355 ms)
  {
    std::size_t i = dispatch.natural_semantic_idx;
    triplets.emplace_back(i,i,1);
    for (const auto & neigh : dispatch.neighbours)
    {
      std::size_t j = keys_affectation.find(neigh)->natural_semantic_idx;
      triplets.emplace_back(i,j,1);
    }
  }
  Eigen::SparseMatrix<int> H(keys_affectation.size(),keys_affectation.size());
  H.setFromTriplets(triplets.begin(),triplets.end());

  return H;
}
