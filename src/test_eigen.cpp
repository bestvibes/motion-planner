#include <iostream>
#include <Eigen/Dense>
#include <exception>
#include <tuple>
#include <map>
#include <experimental/array>
#include <string>

#include "utilities.hpp"

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
	    cout << sum(1, 2, 3, 4, 5, 6, 7) << "\n";
				cout << sum2(1, 2, 3, 4, 5, 6, 7) << "\n";
}
