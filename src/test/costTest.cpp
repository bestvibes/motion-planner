#include <array>
#include <cassert>
#include "gtest/gtest.h"
#include "cost.hpp"
// #include <range/v3/all.hpp> 
#include <range/v3/view.hpp> 

using namespace trajectoryOptimization::cost;

TEST(costTest, controlSquare_IsZero_WhenControlsAreAllZero) { 
	const unsigned numberOfPoints = 3;    
	const unsigned pointDimension = 3;  
	const unsigned controlDimension = 1; 
	const auto trajectoryDimension = numberOfPoints * pointDimension;
	std::array<double, trajectoryDimension> trajectoryWithZeroControl;  
	const double * trajectoryWithZeroControl_RawDouble = trajectoryWithZeroControl.data();
	trajectoryWithZeroControl.fill(0);

	auto getControlSquareSum = GetControlSquareSum<numberOfPoints,
																										 pointDimension,
																										 controlDimension>();
	EXPECT_EQ(0, getControlSquareSum(trajectoryWithZeroControl_RawDouble));
}


TEST(costTest, controlSquare_IsTwelve_WhenContorlsAreThreeTwo) { 
	using namespace ranges;
	const unsigned numberOfPoints = 3;    
	const unsigned pointDimension = 3;  
	const unsigned controlDimension = 1; 
	const auto trajectoryDimension = numberOfPoints * pointDimension;
	std::array<double, pointDimension> singlePoint = {{0, 0, 2}};
	auto singlePointWithControlTwo = view::all(singlePoint);
	auto trajectorWithControlTwo_Range = singlePointWithControlTwo
																			 | view::cycle
									           					 | view::take(trajectoryDimension);
	std::vector<double> trajectoryWithControlTwo
											= yield_from(trajectorWithControlTwo_Range);
	assert(trajectoryWithControlTwo.size() == trajectoryDimension);
	const double* trajectoryWithControlTwo_rawDouble
								= trajectoryWithControlTwo.data();


	auto getControlSquareSum = GetControlSquareSum<numberOfPoints,
																										 pointDimension,
																										 controlDimension>();
	EXPECT_EQ(12, getControlSquareSum(trajectoryWithControlTwo_rawDouble));
}

 
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
