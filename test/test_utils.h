#ifndef SAM_TEST_UTILS_H_
#define SAM_TEST_UTILS_H_

#include <gtest/gtest.h>
#include <random>

template <std::size_t N, typename MAT_T>
MAT_T sample_nmv_u_vector(std::default_random_engine& generator)
{
  std::normal_distribution<double> dist {0, 1};
  MAT_T         X;
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
    std::cout << "Expected:\n" << expected_bi << '\n'; 
    std::cout << "Value:\n" << value_bi << '\n'; 
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

template<typename Key_t>
void EXPECT_KEY_APPROX(const std::string & id, const Key_t & exp , const Key_t & val, double p=1)
{
  EXPECT_TRUE( exp.isApprox(val, p)  );
}



#endif
