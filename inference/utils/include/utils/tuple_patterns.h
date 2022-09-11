#ifndef SAM_TUPLE_PATTERNS_H_
#define SAM_TUPLE_PATTERNS_H_

#include <tuple>
#include <type_traits>
#include <utility>

namespace sam_tuples
{

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
  auto reduce_tuple_variadically(TUP&& my_tuple, FUNC f)
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

  //------------------------------------------------------------------//
  //                           foreach tup                            //
  //------------------------------------------------------------------//
  // WARNING: try with lambda and regular, didn't work
  namespace detail
  {
    template <typename T, typename F, std::size_t... Is>
    void for_each(T&& t, F f, std::index_sequence<Is...>)
    {
      // NOTE: unfortunately, Is, the second arg of f, wont be considered constexpr inside f
      auto l = {(f(std::get<Is>(t), Is), 0)...};
    }
  }   // namespace detail

  template <typename... Ts, typename F>
   void for_each_in_tuple(std::tuple<Ts...> & t, F f)
  {
    detail::for_each(t, f, std::make_index_sequence<sizeof...(Ts)> {});
  }

  template <typename... Ts, typename F>
  void for_each_in_const_tuple(std::tuple<Ts...> const& t, F f)
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
//                  type version of std::tuple_cat                  //
//------------------------------------------------------------------//
template <typename... Tups>
using tuple_cat_t = decltype(std::tuple_cat(std::declval<Tups>()...));

//------------------------------------------------------------------//
//          flatten a two layer tuple into a single tuple           //
//    Output tuple is composed of the concatenation of the types in //
//                        the deepest layer                         //
//------------------------------------------------------------------//
// unspecialised structure (useless in itself but has to be declared)
template <typename ...DeepestTups>
struct flattened_tuples;
template <typename DeepestTup, typename... DeepestTups>
struct flattened_tuples<std::tuple<DeepestTup, DeepestTups...>>
{
  using type = tuple_cat_t<DeepestTup, DeepestTups...>;
};
template <typename Tup>
using flattened_tuples_t = typename flattened_tuples<Tup>::type;


//------------------------------------------------------------------//
//           filter out duplicate template argument types           //
//         and output type a tuple of this filtered set of          //
//                              types                               //
//------------------------------------------------------------------//
template<typename ...ts>
struct tuple_filter_duplicate;
template<typename ...ts> // not sure 1/2
using tuple_filter_duplicate_t =typename tuple_filter_duplicate<ts...>::type; // not sure 2/2
// unitype specialisation, wow !
template<typename t>
struct tuple_filter_duplicate<t>
{
  static constexpr std::size_t size = 1;
  using type = typename std::tuple<t>;
};
// recurse
template<typename tnew,typename ts0,typename ...ts>
struct tuple_filter_duplicate<tnew,ts0,ts...>
{
  // check if the new type tnew exists in the set
   static constexpr std::size_t b= std::is_same_v<tnew,ts0> || ( std::is_same_v<tnew,ts> || ... );
   // using type = typename std::conditional_t<b, std::tuple<ts...>, std::tuple<tnew,ts...>>;
   using type = typename std::conditional_t<b,typename tuple_filter_duplicate<ts0,ts...>::type, std::tuple<tnew,ts0,ts...>>;
   // using recurs_type = typename std::tuple<ts...>;
  static constexpr std::size_t size = std::tuple_size_v<type>;
};

// // note: the duplicates are eleminated from left to right: the first "int" is gone (play with the initializer list)
// inline static constexpr tuple_filter_duplicate<int,std::array<int,2>, double, int,const char*>::type aaaa {{1,2},-3.1654,45,"jk"};
// // note: obvious advise is then to use a typedef/using after instanciating
// using my_filtered_tuple_t = tuple_filter_duplicate<int,std::array<int,2>, double, int,const char*>;

// todo: specialize tuple_filter_duplicate if given argument is tuple ?
// tuple argument specialisation (-> extract what's inside the tuple, and filter duplicate)
template<typename ...Ts>
struct tuple_filter_duplicate< std::tuple<Ts...> >: tuple_filter_duplicate<Ts...>{};







//------------------------------------------------------------------//
//                         constexpr index                          //
//                                                                  //
//                                                                  //
//        Use this when manipulating looping several tuples         //
//           (of the same size), all the others patterns            //
//           (including std::apply) only loop one tuple.            //
//        So instead, constexpr_for() just iterate the tuple        //
//                  idx, allowing several usage of                  //
//              std::get<idx>(a_tuple) at each level.               //
//------------------------------------------------------------------//
template <typename Integer, Integer ...I, typename F>
constexpr void constexpr_for_each(std::integer_sequence<Integer, I...>, F &&func)
{
    (func(std::integral_constant<Integer, I>{}) , ...);
}

template <auto N, typename F>
constexpr void constexpr_for(F &&func)
{
    if constexpr (N > 0)
        constexpr_for_each(std::make_integer_sequence<decltype(N), N>{}, std::forward<F>(func));
}

}   // namespace sam_tuples
#endif
