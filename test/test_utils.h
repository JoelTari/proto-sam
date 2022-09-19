#pragma once

#include <gtest/gtest.h>
#include <random>

template <std::size_t N, typename MAT_T>
MAT_T sample_nmv_u_vector(std::default_random_engine& generator)
{
  std::normal_distribution<double> dist {0, 1};
  MAT_T                            X;
  for (int i = 0; i < N; i++) X[i] = dist(generator);
  return X;
}


template <typename TUP_Aiks>
void EXPECT_TUPLE_OF_MATRIX_APPROX(const TUP_Aiks& expected, const TUP_Aiks& value, double p)
{
  // tup zip pattern
  std::apply(
      [&](const auto&... val)
      {
        std::apply(
            [&](const auto&... exp)
            {
              //  (EXPECT_TRUE(exp.isApprox(val,p)) , ...  ) ;
              EXPECT_TRUE((exp.isApprox(val, p) && ...));
            },
            expected);
      },
      value);
}

template <typename VEC_bi, typename TUP_Aiks>
void EXPECT_bi_Aiks(const std::tuple<VEC_bi, TUP_Aiks>& expected,
                    const std::tuple<VEC_bi, TUP_Aiks>& value,
                    double                              p)
{
  auto expected_bi   = std::get<0>(expected);
  auto value_bi      = std::get<0>(value);
  auto expected_Aiks = std::get<1>(expected);
  auto value_Aiks    = std::get<1>(value);
  EXPECT_TRUE(value_bi.isApprox(expected_bi, p));
  if (!value_bi.isApprox(expected_bi, p))
  {
    std::cerr << "Expected:\n" << expected_bi << '\n';
    std::cerr << "Value:\n" << value_bi << '\n';
  }
  EXPECT_TUPLE_OF_MATRIX_APPROX(expected_Aiks, value_Aiks, p);
}


template <typename VEC_bi, typename TUP_Aiks>
void print(const std::tuple<VEC_bi, TUP_Aiks>& biAiks)
{
  std::cout << "bi : \n" << std::get<0>(biAiks).transpose() << '\n';
  std::apply([](const auto&... Aik) { ((std::cout << "Ai_k:\n"
                                                  << Aik << '\n'),
                                       ...); },
             std::get<1>(biAiks));
}

template <typename Key_Meta>
void EXPECT_KEY_APPROX(const std::string&              id,
                       const typename Key_Meta::key_t& exp,
                       const typename Key_Meta::key_t& val,
                       double                          p = 1)
{
  EXPECT_TRUE(exp.isApprox(val, p));
  if (!val.isApprox(exp, p))
  {
    std::cerr << "Expected key     " << id << ": " << Key_Meta::stringify_key_oneliner(exp) << '\n';
    std::cerr << "Actual key Value " << id << ": " << Key_Meta::stringify_key_oneliner(val) << '\n';
  }
}

template <typename FT>
struct PreRegistrationBundle
{
  std::vector<std::string>                          vfid         = {};
  std::vector<typename FT::measure_t>               vmeasure     = {};
  std::vector<typename FT::measure_cov_t>           vmeasure_cov = {};
  std::vector<std::array<std::string, FT::kNbKeys>> vkeys_id     = {};

  void push(const std::string&                          fid,
            const typename FT::measure_t&               m,
            const typename FT::measure_cov_t&           S,
            const std::array<std::string, FT::kNbKeys>& keys)
  {
    this->vfid.push_back(fid);
    this->vmeasure.push_back(m);
    this->vmeasure_cov.push_back(S);
    this->vkeys_id.push_back(keys);
    this->dsize++;
    this->isempty = false;
  }

  bool        isempty = true;
  bool        empty() const { return this->isempty; }
  std::size_t dsize = 0;
  std::size_t size() const { return this->dsize; }
};


// use-case one collection per time tk
// e.g. landmark slam: one factor for odometry, then several for landmark obs
template <typename FT, typename... FTs>
struct PreRegistrationBundleCollection
{
  using tuple_t = std::tuple<PreRegistrationBundle<FT>, PreRegistrationBundle<FTs>...>;
  std::vector<std::tuple<PreRegistrationBundle<FT>, PreRegistrationBundle<FTs>...>>
      vector_of_collection;
};
