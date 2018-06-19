#include <gtest/gtest.h> 
#include <gmock/gmock.h>
#include <range/v3/view.hpp>
#include <functional>
#include "trajectoryOptimization/utilities.hpp"
#include "trajectoryOptimization/dynamic.hpp"
#include "trajectoryOptimization/constraint.hpp"

using namespace trajectoryOptimization::constraint;
using namespace trajectoryOptimization::utilities;
using namespace trajectoryOptimization::dynamic; 
using namespace testing;
using namespace ranges;

class kinematicGoalConstraintTest:public::Test{
	protected:
		const unsigned numberOfPoints = 2;    
		const unsigned pointDimension = 6;  
		const unsigned kinematicDimension = 4;
		std::vector<double> trajectory;
	
		virtual void SetUp(){
			auto point1 = {0, 0, 0, 0, 2, 3};
			auto point2 = {2, 3, 4, 5, 6, 7};
			trajectory = yield_from(view::concat(point1, point2));
			assert (trajectory.size() == numberOfPoints*pointDimension);
		}
};


TEST_F(kinematicGoalConstraintTest, ZerosWhenReachingGoal){
	const unsigned goalTimeIndex = 1; 
	std::vector<double> kinematicGoal = {{2, 3, 4, 5, 6, 7}};
	const double* trajectoryPtr = trajectory.data();
	auto getToKinematicGoalSquare =
		GetToKinematicGoalSquare(numberOfPoints,
								 pointDimension,
								 kinematicDimension,
								 goalTimeIndex,
								 kinematicGoal);

	auto  toGoalSquares = getToKinematicGoalSquare(trajectoryPtr);
	EXPECT_THAT(toGoalSquares, ElementsAre(0, 0, 0, 0));

}

TEST_F(kinematicGoalConstraintTest, increasingKinematicValues){
	const unsigned goalTimeIndex = 1; 
	std::vector<double> kinematicGoal = {{-1, -1, -1, -1}};
	const double* trajectoryPtr = trajectory.data();
	auto getToKinematicGoalSquare =
		GetToKinematicGoalSquare(numberOfPoints,
								 pointDimension,
								 kinematicDimension,
								 goalTimeIndex,
								 kinematicGoal);

	auto  toGoalSquares = getToKinematicGoalSquare(trajectoryPtr);
	EXPECT_THAT(toGoalSquares, ElementsAre(9, 16, 25, 36));
}

TEST_F(kinematicGoalConstraintTest, twoKinematicGoalConstraints){
	const unsigned goalOneTimeIndex = 0; 
	const unsigned goalTwoTimeIndex = 1; 

	std::vector<double> kinematicGoalOne = {{1, 2, 3, 4}};
	std::vector<double> kinematicGoalTwo = {{-1, -1, -1, -1}};

	const double* trajectoryPtr = trajectory.data();

	auto getToGoalOneSquare =
		GetToKinematicGoalSquare(numberOfPoints,
								 pointDimension,
								 kinematicDimension,
								 goalOneTimeIndex,
								 kinematicGoalOne);

	auto getToGoalTwoSquare =
		GetToKinematicGoalSquare(numberOfPoints,
								 pointDimension,
								 kinematicDimension,
								 goalTwoTimeIndex,
								 kinematicGoalTwo);


	std::vector<ConstraintFunction> twoGoalConstraintFunctions =
												{getToGoalOneSquare, getToGoalTwoSquare};
	auto stackConstriants = StackConstriants(trajectory.size(), twoGoalConstraintFunctions);

	std::vector<double> squaredDistanceToTwoGoals =
											stackConstriants(trajectoryPtr);

	EXPECT_THAT(squaredDistanceToTwoGoals,
							ElementsAre(1, 4, 9, 16, 9, 16, 25, 36));
}

class blockDynamic:public::Test{
	protected:
		const unsigned numberOfPoints = 3;    
		const unsigned pointDimension = 6;  
		const unsigned kinematicDimension = 4;
		unsigned positionDimension;
		unsigned velocityDimension;
		const double dt = 0.5;
		std::vector<double> trajectory;
		const double* trajectoryPtr;
	
		virtual void SetUp(){
			std::vector<double> point1 = {0, 0, 3, 4, 1, 2};
			std::vector<double> point2 = {1.5, 2, 3.5, 5, 2, 4};
			std::vector<double> point3 = {2.5, 3, 4.5, 6, 3, 5};
			positionDimension = kinematicDimension/2;
			velocityDimension = positionDimension;
			trajectory = yield_from(view::concat(point1, point2, point3));
			assert (trajectory.size() == numberOfPoints*pointDimension);
			trajectoryPtr = trajectory.data();
		}
};

TEST_F(blockDynamic, oneTimeStepViolation){
	const unsigned timeIndex = 0;
	DynamicFunction blockDynamics = BlockDynamics;
	auto getKinematicViolation = GetKinematicViolation(blockDynamics,
														pointDimension,
														positionDimension,
														timeIndex, 
														dt);
	std::vector<double> kinematicViolation = getKinematicViolation(trajectoryPtr);
	EXPECT_THAT(kinematicViolation,
							ElementsAre(-0.125, -0.25, -0.25, -0.5));
}

TEST_F(blockDynamic, twoTimeStepsViolation){
	const unsigned timeIndexZero = 0;
	const unsigned timeIndexOne = 1;
	DynamicFunction blockDynamics = BlockDynamics;
	auto getTime0KinematicViolation = GetKinematicViolation(blockDynamics,
															pointDimension,
															positionDimension,
															timeIndexZero, 
															dt);

	auto getTime1KinematicViolation = GetKinematicViolation(blockDynamics,
															pointDimension,
															positionDimension,
															timeIndexOne, 
															dt);

	std::vector<ConstraintFunction> twoStepConstraintFunctions = {getTime0KinematicViolation,
																	getTime1KinematicViolation};

	auto getStackConstriants = StackConstriants(trajectory.size(), twoStepConstraintFunctions);

	auto twoStepKinematicViolations = getStackConstriants(trajectoryPtr);

	EXPECT_THAT(twoStepKinematicViolations,
							ElementsAre(-0.125, -0.25, -0.25, -0.5, -1, -1.75, -0.25, -1.25));
}

TEST_F(blockDynamic, twoTimeStepsViolationUsingApplyFunction){
	const unsigned timeIndexZero = 0;
	const unsigned timeIndexOne = 1;
	const unsigned startTimeIndex = timeIndexZero;
	const unsigned numTimePoints = 3;
	const unsigned endTimeIndex = startTimeIndex + numTimePoints - 1;
	const unsigned constraintIndex = 0;
	DynamicFunction blockDynamics = BlockDynamics;

	std::vector<ConstraintFunction> twoStepConstraintFunctions;

	twoStepConstraintFunctions = applyKinematicViolationConstraints(twoStepConstraintFunctions,
																	blockDynamics,
																	pointDimension,
																	positionDimension,
																	startTimeIndex,
																	endTimeIndex,
																	dt);

	auto getStackConstriants = StackConstriants(trajectory.size(), twoStepConstraintFunctions);
	auto twoStepKinematicViolations = getStackConstriants(trajectoryPtr);

	EXPECT_THAT(twoStepKinematicViolations,
							ElementsAre(-0.125, -0.25, -0.25, -0.5, -1, -1.75, -0.25, -1.25));
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
