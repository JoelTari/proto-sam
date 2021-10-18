#include <array>
#include <iostream>

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
