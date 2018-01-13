#include <iostream>
#include <Eigen/Dense>
#include <exception>
#include <tuple>
#include <map>
#include <experimental/array>
#include <string>
#include <range/v3/all.hpp> 
// #include "utilities.hpp"
#include "cost.hpp"
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

	// std::array<double, 6> a_arr; a_arr.fill(1);
	// std::array<double, 6> b_arr; b_arr.fill(2);
	// std::array<double, 6> c_arr; c_arr.fill(3);
  //
	// auto c = ranges::view::concat(a_arr, b_arr, c_arr);
  //
	// ranges::for_each(c, [](auto r){std::cout << r << std::endl;});
	// // ranges::view::all(a_arr);
  //
	// auto two_out = [](int a, int b, int c){return std::make_tuple(a+b, b+c);};
  //
	// auto ziped = ranges::view::zip_with(two_out, a_arr, b_arr, c_arr);
  //
	// for (auto [a, b]:ziped){
	// 	std::cout << a << std::endl;
	// 	std::cout << b << std::endl;
	// }


	// auto squares = view::transform(view::ints(1), [](int i){return i*i;});
  //
	// for (int i: squares|view::take(20))
	// 	std::cout << i << ' ';
  //
	// std::cout <<'\n' ;

	
	// auto rng = vi | view::remove_if([] (int i){return i %2 == 1;})
	// 							| view::transform([] (int i){return i;});
	//


	// ranges::for_each(foo, [](auto r){std::cout << r << std::endl;});
	// std::cout << typeid((foo)).name() << std::endl;;

	// int sum = ranges::accumulate(view::ints(1, 10), [](int i){
	// 		return ranges::yield_from(view::repeat_n(i, i));} 

	

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
	std::array<double, 6> x;
	x.fill(2.0);
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
	const double* x_arr= x.data();

	// std::vector<const double> v(x_arr, x_arr+6);

	// auto rng = ranges::view::all(x_arr);

	traj_opt::cost::Control_Cost<2, 3> cost(1, 1, 1);
	traj_opt::cost::Control_Cost<2, 3> cost2(1, 1, 1);


	std::vector<traj_opt::cost::Eval_F_Func> cost_funcs{cost, cost2}; 
	traj_opt::cost::Sum_Cost sum_cost(cost_funcs);
	auto f = sum_cost(x_arr);
	std::cout << f<< std::endl;
}
