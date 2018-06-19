#include <array>
#include <cassert>
#include <gtest/gtest.h>
#include "trajectoryOptimization/cost.hpp"
#include "trajectoryOptimization/utilities.hpp"

using namespace trajectoryOptimization::cost;
using namespace trajectoryOptimization::utilities;

TEST(costTest, controlSquare_isZero_whenControlsAreAllZero) { 
	const unsigned numberOfPoints = 3;
	const unsigned pointDimension = 3;
	const unsigned controlDimension = 1;
	const auto trajectoryDimension = numberOfPoints * pointDimension;
	std::array<double, trajectoryDimension> trajectoryWithZeroControl;
	const double * trajectoryWithZeroControl_RawDouble = trajectoryWithZeroControl.data();
	trajectoryWithZeroControl.fill(0);

	auto getControlSquareSum = GetControlSquareSum(numberOfPoints,
													pointDimension,
													controlDimension);
	EXPECT_EQ(0, getControlSquareSum(trajectoryWithZeroControl_RawDouble));
}


TEST(costTest, controlSquareIsTwelveWhenControlsAreThreeTwo) {
	using namespace ranges;
	const unsigned numberOfPoints = 3;
	const unsigned pointDimension = 3;
	const unsigned controlDimension = 1;
	const auto trajectoryDimension = numberOfPoints * pointDimension;
	std::vector<double> controlTwoPoint = {{0, 0, 2}};
	auto trajectoryWithControlTwo = createTrajectoryWithIdenticalPoints(
																	numberOfPoints,
																	controlTwoPoint);
	assert(trajectoryWithControlTwo.size() == trajectoryDimension);
	const double* trajectoryWithControlTwo_rawDouble
								= trajectoryWithControlTwo.data();


	auto getControlSquareSum = GetControlSquareSum(numberOfPoints,
													pointDimension,
													controlDimension);
	EXPECT_EQ(12, getControlSquareSum(trajectoryWithControlTwo_rawDouble));
}

TEST(costTest, controlSquareIsSixPointSevenFiveWhenControlsAreThreeOnePointFive) {
	using namespace ranges;
	const unsigned numberOfPoints = 3;
	const unsigned pointDimension = 3;
	const unsigned controlDimension = 1;
	const auto trajectoryDimension = numberOfPoints * pointDimension;
	std::vector<double> controlTwoPoint = {{0, 0, 1.5}};
	auto trajectoryWithControlTwo = createTrajectoryWithIdenticalPoints(
																	numberOfPoints,
																	controlTwoPoint);
	assert(trajectoryWithControlTwo.size() == trajectoryDimension);
	const double* trajectoryWithControlTwo_rawDouble
								= trajectoryWithControlTwo.data();


	auto getControlSquareSum = GetControlSquareSum(numberOfPoints,
													pointDimension,
													controlDimension);
	EXPECT_DOUBLE_EQ(6.75, getControlSquareSum(trajectoryWithControlTwo_rawDouble));
}

TEST(costTest, controlSquareIs24WhenControlsAreThreeTwoTwo) { 
	using namespace ranges;
	const unsigned numberOfPoints = 3;
	const unsigned pointDimension = 4;
	const unsigned controlDimension = 2;
	const auto trajectoryDimension = numberOfPoints * pointDimension;
	std::vector<double> controlTwoTwo = {{0, 0, 2, 2}};
	auto trajectoryWithControlTwoTwo = createTrajectoryWithIdenticalPoints(
																		 numberOfPoints,
																		 controlTwoTwo);
	assert(trajectoryWithControlTwoTwo.size() == trajectoryDimension);
	const double* trajectoryWithControlTwoTwo_ptr
								= trajectoryWithControlTwoTwo.data();
	auto getControlSquareSum = GetControlSquareSum(numberOfPoints,
													pointDimension,
													controlDimension);
	EXPECT_EQ(24, getControlSquareSum(trajectoryWithControlTwoTwo_ptr));
}
 
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
