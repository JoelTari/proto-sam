#include <array>
#include <iostream>

/* template <typename T, */
/*           size_t                   N, */
/*           const std::array<T, N> & arr, */
/*           typename Fn, */
/*           size_t Index = 0> */
/* void iterate_static_array(Fn && fn) */
/* { */
/*   static constexpr T e = arr[Index]; */
/*   fn.template        operator()<e>(); */
/*   if constexpr (Index + 1 < N) { */
/*     iterate_static_array<T, N, arr, Fn, Index + 1>(std::forward<Fn>(fn)); */
/*   } */
/* } */

/* int main() */
/* { */
/*   static constexpr std::array<int, 10> values = {1, 2, 3, 4, 5, 6, 7, 8, 9,
 * 10}; */
/*   iterate_static_array<int, 10, values>( */
/*       []<int e>() { std::cout << "compile-time e = " << e << "\n"; }); */
/* } */

template <typename Derived, typename T>
class BaseTest
{
public:
  T foo;

  virtual void bar() { static_cast<Derived *>(this)->bar(); }
};

template <typename Derived, typename T>
class StationaryTest : public BaseTest<Derived, T>
{
public:
  void bar()
  {
    do_spec();
    std::cout << this->foo << std::endl;
  }

  void do_spec() { static_cast<Derived *>(this)->do_spec(); }
};

// impl from base
class Odom : public BaseTest<Odom, int>
{
public:
  void bar()
  {
    // do something
  }
};

// impl from StationaryTest
class Bearing : public StationaryTest<Bearing, int>
{
public:
  void do_spec()
  {
    // do something
  }
};

int main()
{
  auto odom    = Odom();
  auto bearing = Bearing();
  bearing.do_spec();
}
