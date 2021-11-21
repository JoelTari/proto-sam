#ifndef SAM_TUPLE_PATTERNS_H_
#define SAM_TUPLE_PATTERNS_H_

#include <tuple>
#include <utility>

namespace sam_tuples
{
  // index sequence pattern


  //------------------------------------------------------------------//
  //                           reduce tuple                           //
  //------------------------------------------------------------------//
  template <typename ARRAY, typename FUNC>
  auto reduce_variadically(const ARRAY& my_array, FUNC f)
  {
    // or add the folowing template argument :  Indices =
    // std::make_index_sequence< std::tuple_size_v<ARRAY> >
    //  and use Indices{} instead of N

    // WOW !!!!
    if constexpr (std::is_invocable_v<FUNC, ARRAY>)   // HACK:
      return f(my_array);
    else
    {
      std::make_index_sequence<std::tuple_size_v<ARRAY>> N {};
      return f(my_array, N);
    }
  }
  // // typical use : sum, multiply, make_tuple
  // template <typename ARRAY, std::size_t I0, std::size_t... I>
  // static auto core_function(const ARRAY& array, std::index_sequence<I0, I...>)
  // {
  //   static_assert(std::tuple_size_v<ARRAY> == 1 + sizeof...(I));
  //   static_assert(I0 == 0);
  //   // toute la logique basee template doit etre ici
  //   // si notre expression est cachee sous un tuple:
  //   //       return std::get<I0>(my_tuple).something_value()  + (
  //   //       std::get<I>(my_tuple).something_value() + ... );
  //   return array[I0] + (array[I] + ...);
  // }
  // template <typename ARRAY>
  // static auto wrapper_function(const ARRAY& array)
  // {
  //   std::make_index_sequence<std::tuple_size_v<ARRAY>> N {};
  //   // auto truc = std::function<typename _Signature>;
  //   return core_function(array, N);
  // }
  // // call
  // reduce_variadically(A, &wrapper_function<std::array<double, 4>>);
  // // or same with a lambda (c++20 required)
  // auto lambda_result = []<typename ARRAY, std::size_t I0, std::size_t... I>(
  //     const ARRAY& array,
  //     std::index_sequence<I0, I...>)
  // {
  //   static_assert(I0 == 0, "not 0");
  //   // return std::get<I0>(my_tuple).something_value(array[I0])  + (
  //   // std::get<I>(my_tuple).something_value(array[I]) + ... );
  //   return array[I0] + (array[I] + ...);
  // };
  // reduce_variadically(A, lambda_result); 

  //------------------------------------------------------------------//
  //                           foreach tup                            //
  //------------------------------------------------------------------//
  namespace detail
  {
    template <typename T, typename F, std::size_t... Is>
    void for_each(T&& t, F f, std::index_sequence<Is...>)
    {
      auto l = {(f(std::get<Is>(t), Is), 0)...};
    }
  }   // namespace detail

  template <typename... Ts, typename F>
  void for_each_in_tuple(std::tuple<Ts...> const& t, F f)
  {
    detail::for_each(t, f, std::make_index_sequence<sizeof...(Ts)> {});
  }
  
  // //------------------------------------------------------------------//
  // //                std::apply (permits lambda c++17)                 //
  // //------------------------------------------------------------------//
  // std::apply([](auto... e) { ((std::cout << e << '\n'), ...); int a = 1+1; }, my_tuple);
  // std::apply([](auto... e) { ((customAction(e)), ...);  std::cout<< '\n'; }, my_tuple);
  // std::apply([](auto... e) { auto l = [](auto e){ std::cout << e << '\t';}; ((l(e)), ...);  std::cout<< '\n'; }, my_tuple);

  // int i;
  // std::apply([i](auto...e){ if (i)},my_tuple);

}   // namespace sam_tuples
#endif
