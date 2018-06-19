#include <gtest/gtest.h> 
#include <gmock/gmock.h>
#include <vector>
#include <range/v3/view.hpp>
#include "trajectoryOptimization/utilities.hpp"
;
using namespace testing;
using namespace trajectoryOptimization::utilities;

class utilitiesTest:public::Test{
	protected:
		const unsigned pointDimension = 5;
		const unsigned positionDimension = 2; 
		const unsigned velocityDimension = positionDimension ; 
		const unsigned controlDimension = pointDimension - 2* positionDimension;
		std::vector<double> point0 = {0, 0, 3, 4, 5};
		std::vector<double> point1 = {1.5, 2, 3.5, 5, 1};
		std::vector<double> trajectory;
		const double* trajectoryPointer;
	
		virtual void SetUp() {
			trajectory = yield_from(view::concat(point0, point1));
			trajectoryPointer = trajectory.data();
		}
};

TEST_F(utilitiesTest, getPointOne){
	const unsigned timeIndex = 1; 
	auto trajectoryPoint1 = getTrajectoryPoint(trajectoryPointer,
																						 timeIndex,
																						 pointDimension);

	EXPECT_THAT(trajectoryPoint1, point1);
}


TEST_F(utilitiesTest, getPointZero){
	const unsigned timeIndex = 0; 
	auto trajectoryPoint1 = getTrajectoryPoint(trajectoryPointer,
																						 timeIndex,
																						 pointDimension);
	EXPECT_THAT(trajectoryPoint1, point0);
}

TEST_F(utilitiesTest, getPointOnePositionVelocityControl){
	const unsigned timeIndex = 1; 
	auto trajectoryPoint1 = getTrajectoryPoint(trajectoryPointer,
																						 timeIndex,
																						 pointDimension);

	const auto& [position, velocity, control] = 
		getPointPositionVelocityControl(trajectoryPoint1,
																		positionDimension,
																		velocityDimension,
																		controlDimension);

	EXPECT_THAT(position, ElementsAre(1.5, 2));
	EXPECT_THAT(velocity, ElementsAre(3.5, 5));
	EXPECT_THAT(control, ElementsAre(1));
}


TEST_F(utilitiesTest, getPointZeroPositionVelocityControl){
	const unsigned timeIndex = 0; 
	auto trajectoryPoint1 = getTrajectoryPoint(trajectoryPointer,
																						 timeIndex,
																						 pointDimension);

	const auto& [position, velocity, control] = 
		getPointPositionVelocityControl(trajectoryPoint1,
																		positionDimension,
																		velocityDimension,
																		controlDimension);

	EXPECT_THAT(position, ElementsAre(0, 0));
	EXPECT_THAT(velocity, ElementsAre(3, 4));
	EXPECT_THAT(control, ElementsAre(5));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
