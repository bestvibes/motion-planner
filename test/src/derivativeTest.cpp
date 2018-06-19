#include <array>
#include <cmath>
#include <cassert>
#include <functional>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "trajectoryOptimization/derivative.hpp"

using namespace trajectoryOptimization::derivative;

class derivativeTest : public::testing::Test {
	protected:
		const unsigned numberVariables = 4;
		const double x[4] = {2, 3, 4, 5};
		const VectorToDoubleFunction vectorToDoubleFn = [](const double* x) {
			return x[0] + 2*x[1] + 3*std::pow(x[2], 2) + x[0]*x[1];
		};
		const VectorToVectorFunction vectorToVectorFn = [&](const double* x) {
			std::vector<double> output(numberVariables);
			output[0] = x[0] + 2*x[1] + 3*std::pow(x[2], 2) + x[0]*x[1];
			output[1] = x[0] * x[1] * x[2] * x[3];
			output[2] = 0;
			output[3] = x[3] - x[2] - x[1] - x[0];
			return output;
		};
};

TEST_F(derivativeTest, partialOfVectorToDoubleFunction) {
	auto getPartialDerivative = GetPartialDerivativeOfVectorToDoubleFunction(vectorToDoubleFn, numberVariables);
	int partialIndex = 0;
	double partialDerivative;

	partialDerivative = getPartialDerivative(x, partialIndex);
	EXPECT_DOUBLE_EQ(partialDerivative, 1 + x[1]);

	partialIndex++;
	partialDerivative = getPartialDerivative(x, partialIndex);
	EXPECT_DOUBLE_EQ(partialDerivative, 2 + x[0]);

	partialIndex++;
	partialDerivative = getPartialDerivative(x, partialIndex);
	EXPECT_DOUBLE_EQ(partialDerivative, 6 * x[2]);

	partialIndex++;
	partialDerivative = getPartialDerivative(x, partialIndex);
	EXPECT_DOUBLE_EQ(partialDerivative, 0);
}

TEST_F(derivativeTest, gradientOfVectorToDoubleFunction) {
	auto getGradient = GetGradientOfVectorToDoubleFunction(vectorToDoubleFn, numberVariables);
	std::vector<double> gradient = getGradient(x);
	EXPECT_THAT(gradient, testing::ElementsAre(1 + x[1], 2 + x[0], 6 * x[2], 0));
}

TEST_F(derivativeTest, partialOfVectorToVectorFunction) {
	auto getPartialDerivative = GetPartialDerivativeOfVectorToVectorFunction(vectorToVectorFn, numberVariables);
	int variableIndex = 0;
	std::vector<double> partialDerivative;

	const std::vector<double> expectedAtOutputRow0 = {1 + x[1], 2 + x[0], 6 * x[2], 0};
	const std::vector<double> expectedAtOutputRow1 = {x[1]*x[2]*x[3], x[0]*x[2]*x[3], x[0]*x[1]*x[3], x[0]*x[1]*x[2]};
	const std::vector<double> expectedAtOutputRow2 = {0, 0, 0, 0};
	const std::vector<double> expectedAtOutputRow3 = {-1, -1, -1, 1};

	partialDerivative = getPartialDerivative(x, variableIndex);
	EXPECT_THAT(partialDerivative, testing::ElementsAre(expectedAtOutputRow0[variableIndex],
														expectedAtOutputRow1[variableIndex],
														expectedAtOutputRow2[variableIndex],
														expectedAtOutputRow3[variableIndex]));

	variableIndex++;
	partialDerivative = getPartialDerivative(x, variableIndex);
	EXPECT_THAT(partialDerivative, testing::ElementsAre(expectedAtOutputRow0[variableIndex],
														expectedAtOutputRow1[variableIndex],
														expectedAtOutputRow2[variableIndex],
														expectedAtOutputRow3[variableIndex]));

	variableIndex++;
	partialDerivative = getPartialDerivative(x, variableIndex);
	EXPECT_THAT(partialDerivative, testing::ElementsAre(expectedAtOutputRow0[variableIndex],
														expectedAtOutputRow1[variableIndex],
														expectedAtOutputRow2[variableIndex],
														expectedAtOutputRow3[variableIndex]));

	variableIndex++;
	partialDerivative = getPartialDerivative(x, variableIndex);
	EXPECT_THAT(partialDerivative, testing::ElementsAre(expectedAtOutputRow0[variableIndex],
														expectedAtOutputRow1[variableIndex],
														expectedAtOutputRow2[variableIndex],
														expectedAtOutputRow3[variableIndex]));
}

TEST_F(derivativeTest, jacobianColumnsOfVectorToVectorFunction) {
	auto getJacobianColumns = GetJacobianColumnsOfVectorToVectorFunction(vectorToVectorFn, numberVariables);
	unsigned variableIndex = 0;

	const std::vector<double> expectedAtOutputRow0 = {1 + x[1], 2 + x[0], 6 * x[2], 0};
	const std::vector<double> expectedAtOutputRow1 = {x[1]*x[2]*x[3], x[0]*x[2]*x[3], x[0]*x[1]*x[3], x[0]*x[1]*x[2]};
	const std::vector<double> expectedAtOutputRow2 = {0, 0, 0, 0};
	const std::vector<double> expectedAtOutputRow3 = {-1, -1, -1, 1};

	std::vector<std::vector<double>> jacobianColumns = getJacobianColumns(x);

	EXPECT_THAT(jacobianColumns[variableIndex], testing::ElementsAre(expectedAtOutputRow0[variableIndex],
																		expectedAtOutputRow1[variableIndex],
																		expectedAtOutputRow2[variableIndex],
																		expectedAtOutputRow3[variableIndex]));

	variableIndex++;
	EXPECT_THAT(jacobianColumns[variableIndex], testing::ElementsAre(expectedAtOutputRow0[variableIndex],
																		expectedAtOutputRow1[variableIndex],
																		expectedAtOutputRow2[variableIndex],
																		expectedAtOutputRow3[variableIndex]));

	variableIndex++;
	EXPECT_THAT(jacobianColumns[variableIndex], testing::ElementsAre(expectedAtOutputRow0[variableIndex],
																		expectedAtOutputRow1[variableIndex],
																		expectedAtOutputRow2[variableIndex],
																		expectedAtOutputRow3[variableIndex]));

	variableIndex++;
	EXPECT_THAT(jacobianColumns[variableIndex], testing::ElementsAre(expectedAtOutputRow0[variableIndex],
																		expectedAtOutputRow1[variableIndex],
																		expectedAtOutputRow2[variableIndex],
																		expectedAtOutputRow3[variableIndex]));
}

TEST_F(derivativeTest, sparsityPatternOfVectorToVectorFunction) {
	auto getSparsityPattern = GetSparsityPatternOfVectorToVectorFunction(vectorToVectorFn, numberVariables);

	std::vector<int> expectedJacobianRows = {0, 1, 3, 0, 1, 3, 0, 1, 3, 1, 3};
	std::vector<int> expectedJacobianCols = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3};

	const auto [jacobianRows, jacobianCols] = getSparsityPattern();

	EXPECT_THAT(jacobianRows, testing::ContainerEq(expectedJacobianRows));
	EXPECT_THAT(jacobianCols, testing::ContainerEq(expectedJacobianCols));
}

TEST_F(derivativeTest, jacobianOfVectorToVectorFunctionUsingSparsityPattern) {
	auto getSparsityPattern = GetSparsityPatternOfVectorToVectorFunction(vectorToVectorFn, numberVariables);
	const auto [jacobianRows, jacobianCols] = getSparsityPattern();
	auto getJacobian = GetJacobianOfVectorToVectorFunctionUsingSparsityPattern(vectorToVectorFn,
																				numberVariables,
																				jacobianRows,
																				jacobianCols);

	const std::vector<double> expectedOutput = {1 + x[1],
												x[1]*x[2]*x[3],
												-1,
												2 + x[0],
												x[0]*x[2]*x[3],
												-1,
												6 * x[2],
												x[0]*x[1]*x[3],
												-1,
												x[0]*x[1]*x[2],
												1 };

	std::vector<double> jacobian = getJacobian(x);

	EXPECT_THAT(jacobian, testing::ContainerEq(expectedOutput));
}

TEST_F(derivativeTest, jacobianAndSparsityPatternOfVectorToVectorFunction) {
	const auto [jacobianRows, jacobianCols, getJacobian] = getSparsityPatternAndJacobianFunctionOfVectorToVectorFunction(vectorToVectorFn, numberVariables);

	const std::vector<double> expectedOutput = {1 + x[1],
												x[1]*x[2]*x[3],
												-1,
												2 + x[0],
												x[0]*x[2]*x[3],
												-1,
												6 * x[2],
												x[0]*x[1]*x[3],
												-1,
												x[0]*x[1]*x[2],
												1 };

	std::vector<double> jacobian = getJacobian(x);

	EXPECT_THAT(jacobian, testing::ContainerEq(expectedOutput));
}
 
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
