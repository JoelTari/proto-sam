#ifndef SAM_TUPLE_PATTERNS_H_
#define SAM_TUPLE_PATTERNS_H_

#include <tuple>
#include <type_traits>
#include <utility>

namespace sam_tuples
{
  // index sequence pattern


  //------------------------------------------------------------------//
  //             Reduce an array by zipping with a tuple              //
  //        and producing some variadic logic (embedded in f)         //
  //------------------------------------------------------------------//
  template <typename ARRAY, typename FUNC, typename... Args>
  auto reduce_array_variadically(const ARRAY& my_array, FUNC f, Args... args)
  {
    // WOW !!!!
    if constexpr (std::is_invocable_v<FUNC, ARRAY, Args...>)   // HACK:
      return f(my_array, args...);
    else
    {
      std::make_index_sequence<std::tuple_size_v<ARRAY>> N {};
      return f(my_array, args..., N);
    }
  }

  //------------------------------------------------------------------//
  //        Reduce a tuple with variadic logic (embedded in f)        //
  //------------------------------------------------------------------//
  template <typename FUNC, typename TUP>
  auto reduce_tuple_variadically(const TUP& my_tuple, FUNC f)
  {
    // WOW !!!!
    if constexpr (std::is_invocable_v<FUNC, TUP>)   // HACK:
      return f(my_tuple);
    else
    {
      std::make_index_sequence<std::tuple_size_v<TUP>> N {};
      return f(my_tuple, N);
    }
  }

  //------------------------------------------------------------------//
  //        reduce statically a tuple type with variadic logic        //
  //            logic (embedded in f). Should be called by            //
  //                    explicitely instanciating                     //
  //          the TUP template argument (see lambda example)          //
  //------------------------------------------------------------------//
  template <typename TUP, typename FUNC>
  constexpr auto reduce_static_tuple_variadically(FUNC f)
  {
    if constexpr (std::is_invocable_v<FUNC>)   // HACK:
      return f();
    else
    {
      std::make_index_sequence<std::tuple_size_v<TUP>> N {};
      return f(N);
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
  //
  // static example:
  // auto cumul2 =reduce_static_tuple_variadically< tt_tuple_t >([]<std::size_t I0, std::size_t...
  // I>(std::index_sequence<I0, I...>)
  // {
  //   return std::tuple_element_t<I0, tt_tuple_t>::value + (std::tuple_element_t<I,
  //   tt_tuple_t>::value + ...);
  // }
  // );
  // std::cout << "reduce static tuple variadically : " << cumul2 << '\n';

  //------------------------------------------------------------------//
  //                           foreach tup                            //
  //------------------------------------------------------------------//
  // WARNING: try with lambda and regular, didn't work
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

  //------------------------------------------------------------------//
  //         foreach tup : varidically style, and array input         //
  //------------------------------------------------------------------//
  template <typename FUNC, typename ARRAY>
  auto foreach_tup_variadically(const ARRAY& my_array, FUNC f)
  {
    // WOW !!!!
    if constexpr (std::is_invocable_v<FUNC, ARRAY>)   // HACK:
      f(my_array);
    else
    {
      std::make_index_sequence<std::tuple_size_v<ARRAY>> N {};
      f(my_array, N);
    }
  }


  //------------------------------------------------------------------//
  //                std::apply (permits lambda c++17)                 //
  //          std::apply should be the prefer way for tuple           //
  //                            traversing                            //
  //------------------------------------------------------------------//
  // std::apply([](auto... e) { ((std::cout << e << '\n'), ...); int a = 1+1; }, my_tuple);
  // std::apply([](auto... e) { ((customAction(e)), ...);  std::cout<< '\n'; }, my_tuple);
  // std::apply([](auto... e) { auto l = [](auto e){ std::cout << e << '\t';}; ((l(e)), ...);
  // std::cout<< '\n'; }, my_tuple);

//------------------------------------------------------------------//
//                       extend tuple by type                       //
//------------------------------------------------------------------//
template <typename, typename>
struct tuple_type_cat;
template <typename... First, typename... Second>
struct tuple_type_cat<std::tuple<First...>, std::tuple<Second...>> {
    using type = std::tuple<First..., Second...>;
};

//------------------------------------------------------------------//
//          define a tuple by catting a type member of the          //
//                           input types                            //
//------------------------------------------------------------------//
template< typename ...Ts>
struct cat_tuple_in_depth;

template<typename T>
struct cat_tuple_in_depth<T>
{
  using type = std::tuple<typename T::underlyingT>; // WARNING: weakness here: use macro ?
};

template<typename T,typename ...Ts>
struct cat_tuple_in_depth<T,Ts...>
{
  using type = tuple_type_cat<std::tuple<typename T::underlyingT>, typename cat_tuple_in_depth<Ts...>::type >;
};
// extract tuple template argument specialisation
template<typename...Ts>
struct cat_tuple_in_depth< std::tuple<Ts...> >: cat_tuple_in_depth<Ts...>
{};


// template <std::size_t IS_U_IN_T_V, class U, class...T>
// struct tuple_type_uniq_cat;
// // IS_U_IN_T_V   <-  (std::is_same_v<U,T> || ... ) 
// // if type U already in 
// template <class U, class ...T>
// struct tuple_type_uniq_cat<0, std::tuple<U>,std::tuple<T...>>
// {
//     using type = std::tuple<T...,U>;
// }; 
// template <class U, class ...T>
// struct tuple_type_uniq_cat<1, std::tuple<U>,std::tuple<T...>>
// {
//     using type = std::tuple<T...>;
// }; 


//------------------------------------------------------------------//
//           filter out duplicate template argument types           //
//         and output type a tuple of this filtered set of          //
//                              types                               //
//------------------------------------------------------------------//
template<typename ...Ts>
struct tuple_filter_duplicate;
// unitype specialisation, wow !
template<typename T>
struct tuple_filter_duplicate<T>
{
  using type = typename std::tuple<T>;
};
// recurse
template<typename TNEW,typename Ts0,typename ...Ts>
struct tuple_filter_duplicate<TNEW,Ts0,Ts...>
{
  // check if the new type TNEW exists in the set
   static constexpr std::size_t B= std::is_same_v<TNEW,Ts0> || ( std::is_same_v<TNEW,Ts> || ... );
   // using type = typename std::conditional_t<B, std::tuple<Ts...>, std::tuple<TNEW,Ts...>>;
   using type = typename std::conditional_t<B,typename tuple_filter_duplicate<Ts0,Ts...>::type, std::tuple<TNEW,Ts0,Ts...>>;
   // using recurs_type = typename std::tuple<Ts...>;
};

// // NOTE: the duplicates are eleminated from left to right: the first "int" is gone (play with the initializer list)
// inline static constexpr tuple_filter_duplicate<int,std::array<int,2>, double, int,const char*>::type AAAA {{1,2},-3.1654,45,"jk"};
// // NOTE: Obvious advise is then to use a typedef/using after instanciating
// using my_filtered_tuple_t = tuple_filter_duplicate<int,std::array<int,2>, double, int,const char*>;

// TODO: specialize tuple_filter_duplicate if given argument is tuple ?
// tuple argument specialisation (-> extract what's inside the tuple, and filter duplicate)
template<typename ...Ts>
struct tuple_filter_duplicate< std::tuple<Ts...> >: tuple_filter_duplicate<Ts...>{};

}   // namespace sam_tuples
#endif
