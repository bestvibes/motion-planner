#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
template<int N>
using Point = Array<double, 1, N, RowMajor>;
// main function
int main(int argc, const char** argv)
{
	const double x[3] = {1.0, 2.0, 3.0};
	double* y = const_cast<double*>(x);
	Map<Point<3>> x_arr(y, 1, 3);
	std::cout << x_arr << std::endl;
	return 0;
}
