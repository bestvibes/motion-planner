#include <iostream>
#include <Eigen/Dense>
#include <exception>
#include <tuple>
#include <map>
#include <experimental/array>
#include <string>
#include <range/v3/all.hpp> 
// #include "utilities.hpp"
// #include "cost.hpp"
// #include "constraint.hpp"

using namespace std;
using namespace Eigen;
template<int N>
using Point = Array<double, 1, N, RowMajor>;

std::tuple<int, std::string, float> create_a_tuple() {
	  return {20, std::string("baz"), 1.2f};
}

constexpr int factorial(int i)
{
    return (i > 1) ? i * factorial(i - 1) : 1;
}

constexpr int safe_factorial( int i )
{
	    return (i < 0) ?        // error condition
						 throw std::exception(): // error reporting
						 factorial(i);       // real computation
}


std::map<std::string, int> get_map()
{
    return {
        { "hello", 1 },
        { "world", 2 },
        { "it's",  3 },
        { "me",    4 },
    };
}

template<typename ...Args> auto sum(Args ...args) 
{ 
    return (args + ... + 0); 
}

template<typename ...Args> auto sum2(Args ...args) 
{ 
    return (args + ...);
}

int main()
{

	std::array<int, 6> v1 { { 6, 2, 3, 4, 5, 6 } };
	auto v2 = v1;
	auto v3 = v1;


	auto add = [](auto x, auto y, auto z){return x+y+z;};


	auto results = ranges::view::zip_with(add, v1, v2, v3);
	std::cout << typeid(results).name() << std::endl;
	ranges::for_each(results, [](const auto& f){std::cout << f << std::endl;});

	// std::vector<int > v2(6);
	// std::array<int, 6> v2;
	// v2.fill(1);

	// auto foo =  ranges::view::zip_with([](auto a, int b){return a+b;}, v1, v2);;
	// ranges::for_each(foo, [](auto f){std::cout << f << std::endl;});
	
  //note the count return is a numeric type
  //like int or long -- auto below make sure
  //it matches the implementation
  // auto c = ranges::count( v, 6 );
  // cout << "vector:   " << c << "\n";
  //
  // std::array<int, 6> a { { 6, 2, 3, 4, 5, 6 } };
  // c = ranges::count( a, 6 );
  // cout << "array:    " << c << "\n";
	// std::array<double, 6> x;
	// x.fill(2.0);
  //
	// std::array<double, 3> goal{{ 4.0, 4.0, 4.0 }};;
  //
	// traj_opt::constraint::Goal_Constraint<3> goal_const(goal, 0);
  //
	// auto foo = goal_const(x.data());
  //
	// for (auto f:foo){
	// 	std::cout<< f<< std::endl; 
	// }
  //
	// const double* x_arr= x.data();
  //
	// traj_opt::cost::Control_Cost<2, 3> cost(1, 1, 1);
	// traj_opt::cost::Control_Cost<2, 3> cost2(1, 1, 1);
  //
	// std::vector<traj_opt::cost::Eval_F_Func> cost_funcs{cost, cost2}; 
  //
	// traj_opt::cost::Sum_Cost sum_cost(cost_funcs);
  //
	// auto f = sum_cost(x_arr);
	// std::cout << f<< std::endl;
}
