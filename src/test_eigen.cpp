#include <iostream>
#include <Eigen/Dense>

// main function
int main(int argc, const char** argv)
{
	using namespace Eigen;
	using Point = Array<double, 1, 3, RowMajor>;
	const double x[3] = {1.0, 2.0, 3.0};
	double* y = const_cast<double*>(x);
	Map<Point> x_arr(y, 1, 3);
	std::cout << x_arr << std::endl;
	return 0;
}
