#include <gtest/gtest.h> 
#include <gmock/gmock.h>
#include <range/v3/view.hpp>
#include "utilities.hpp"
#include "constraint.hpp"

using namespace trajectoryOptimization::constraint;
using namespace trajectoryOptimization::utilities;
using namespace testing;
using namespace ranges;

TEST(kinematicGoalConstraintTest, ZerosWhenReachingGoal){
	const unsigned numberOfPoints = 3;    
	const unsigned pointDimension = 3;  
	const unsigned kinematicDimension = 2;
	const unsigned goalTimeIndex = 0; 
	auto trajectoryWithOnesQV =  createTrajectoryWithIdenticalPoints(
															 numberOfPoints,
															 {1, 1, 0});
	const double* trajectoryWithOnesQV_ptr = trajectoryWithOnesQV.data();

	const std::vector<double> kinematicGoal = {1, 1};
	auto getToKinematicGoalSquare =
		GetToKinematicGoalSquare<numberOfPoints,
														 pointDimension,
														 kinematicDimension,
														 goalTimeIndex>(kinematicGoal);

	auto  toGoalSquares = getToKinematicGoalSquare(trajectoryWithOnesQV_ptr);
	EXPECT_THAT(toGoalSquares, ElementsAre(0, 0));
}

TEST(kinematicGoalConstraintTest, increasingKinematicValues){
	const unsigned numberOfPoints = 2;    
	const unsigned pointDimension = 6;  
	const unsigned kinematicDimension = 4;
	const unsigned goalTimeIndex = 1; 

	const unsigned trajectoryDimension = numberOfPoints * pointDimension;
	auto point1 = {0, 0, 0, 0, 2, 3};
	auto point2 = {2, 3, 4, 5, 6, 7};

	std::vector<double> trajectory = yield_from(view::concat(point1, point2));
	assert (trajectory.size() == trajectoryDimension);
	std::vector<double> kinematicGoal = {{-1, -1, -1, -1}};

	const double* trajectoryPtr = trajectory.data();

	auto getToKinematicGoalSquare =
		GetToKinematicGoalSquare<numberOfPoints,
														 pointDimension,
														 kinematicDimension,
														 goalTimeIndex>(kinematicGoal);

	auto  toGoalSquares = getToKinematicGoalSquare(trajectoryPtr);
	EXPECT_THAT(toGoalSquares, ElementsAre(9, 16, 25, 36));
}


TEST(stackedConstriantsTest, twoKinmaticGoalConstraints){
	const unsigned numberOfPoints = 2;    
	const unsigned pointDimension = 6;  
	const unsigned kinematicDimension = 4;
	const unsigned goalOneTimeIndex = 0; 
	const unsigned goalTwoTimeIndex = 1; 

	const unsigned trajectoryDimension = numberOfPoints * pointDimension;
	auto point1 = {0, 0, 0, 0, 2, 3};
	auto point2 = {2, 3, 4, 5, 6, 7};

	std::vector<double> trajectory = yield_from(view::concat(point1, point2));
	const double* trajectoryPtr = trajectory.data();
	assert (trajectory.size() == trajectoryDimension);
	std::vector<double> kinematicGoalOne = {{1, 2, 3, 4}};
	std::vector<double> kinematicGoalTwo = {{-1, -1, -1, -1}};

	auto getToGoalOneSquare =
		GetToKinematicGoalSquare<numberOfPoints,
														 pointDimension,
														 kinematicDimension,
														 goalOneTimeIndex>(kinematicGoalOne);

	auto getToGoalTwoSquare =
		GetToKinematicGoalSquare<numberOfPoints,
														 pointDimension,
														 kinematicDimension,
														 goalTwoTimeIndex>(kinematicGoalTwo);

	std::vector<constraintFunction> twoGoalConstraintFunctions =
																	{getToGoalOneSquare, getToGoalTwoSquare};
	auto stackConstriants = StackConstriants(twoGoalConstraintFunctions);

	std::vector<double> squaredDistanceToTwoGoals =
											stackConstriants(trajectoryPtr);

	EXPECT_THAT(squaredDistanceToTwoGoals,
							ElementsAre(1, 4, 9, 16, 9, 16, 25, 36));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
