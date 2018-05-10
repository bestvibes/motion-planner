#include <gtest/gtest.h> 
#include <gmock/gmock.h>
#include <range/v3/view.hpp>
#include <functional>
#include "utilities.hpp"
#include "dynamic.hpp"
#include "constraint.hpp"

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
	auto stackConstriants = StackConstriants(twoGoalConstraintFunctions);

	std::vector<double> squaredDistanceToTwoGoals =
											stackConstriants(trajectoryPtr);

	EXPECT_THAT(squaredDistanceToTwoGoals,
							ElementsAre(1, 4, 9, 16, 9, 16, 25, 36));
}

TEST_F(kinematicGoalConstraintTest, GradientZerosWhenReachingGoal){
	const unsigned goalTimeIndex = 1; 
	const unsigned constraintIndex = 0;
	std::vector<double> kinematicGoal = {{2, 3, 4, 5, 6, 7}};
	const double* trajectoryPtr = trajectory.data();
	auto getToKinematicGoalSquareGradient =
		GetToKinematicGoalSquareGradient(numberOfPoints,
								 pointDimension,
								 kinematicDimension,
								 goalTimeIndex,
								 kinematicGoal);

	auto  toGoalSquaresGradient = getToKinematicGoalSquareGradient(trajectoryPtr);

	auto getToKinematicGoalSquareGradientIndices = GetToKinematicGoalSquareGradientIndices(pointDimension,
																							kinematicDimension,
																							goalTimeIndex);
	auto [numConstraints, gradientRows, gradientCols] = getToKinematicGoalSquareGradientIndices(constraintIndex);


	EXPECT_THAT(toGoalSquaresGradient, ElementsAre(0, 0, 0, 0));
	EXPECT_EQ(numConstraints, kinematicDimension);
	EXPECT_THAT(gradientRows, ElementsAre(0, 1, 2, 3));
	EXPECT_THAT(gradientCols, ElementsAre(6, 7, 8, 9));

}

TEST_F(kinematicGoalConstraintTest, increasingKinematicValuesGradient){
	const unsigned goalTimeIndex = 1;
	const unsigned constraintIndex = 0;
	std::vector<double> kinematicGoal = {{-1, -1, -1, -1}};
	const double* trajectoryPtr = trajectory.data();
	auto getToKinematicGoalSquareGradient =
		GetToKinematicGoalSquareGradient(numberOfPoints,
										 pointDimension,
										 kinematicDimension,
										 goalTimeIndex,
										 kinematicGoal);

	auto toGoalSquaresGradient = getToKinematicGoalSquareGradient(trajectoryPtr);

	auto getToKinematicGoalSquareGradientIndices = GetToKinematicGoalSquareGradientIndices(pointDimension,
																							kinematicDimension,
																							goalTimeIndex);
	auto [numConstraints, gradientRows, gradientCols] = getToKinematicGoalSquareGradientIndices(constraintIndex);

	EXPECT_THAT(toGoalSquaresGradient, ElementsAre(6, 8, 10, 12));
	EXPECT_EQ(numConstraints, kinematicDimension);
	EXPECT_THAT(gradientRows, ElementsAre(0, 1, 2, 3));
	EXPECT_THAT(gradientCols, ElementsAre(6, 7, 8, 9));
}

TEST_F(kinematicGoalConstraintTest, twoKinematicGoalConstraintGradients){
	const unsigned goalOneTimeIndex = 0; 
	const unsigned goalTwoTimeIndex = 1;
	const unsigned constraintIndex = 0;

	std::vector<double> kinematicGoalOne = {{1, 2, 3, 4}};
	std::vector<double> kinematicGoalTwo = {{-1, -1, -1, -1}};

	const double* trajectoryPtr = trajectory.data();

	auto getToGoalOneSquareGradient =
		GetToKinematicGoalSquareGradient(numberOfPoints,
								 pointDimension,
								 kinematicDimension,
								 goalOneTimeIndex,
								 kinematicGoalOne);

	auto getToGoalTwoSquareGradient =
		GetToKinematicGoalSquareGradient(numberOfPoints,
								 pointDimension,
								 kinematicDimension,
								 goalTwoTimeIndex,
								 kinematicGoalTwo);


	std::vector<ConstraintGradientFunction> twoGoalConstraintGradientFunctions =
												{getToGoalOneSquareGradient, getToGoalTwoSquareGradient};
	auto stackConstriantGradients = StackConstriantGradients(twoGoalConstraintGradientFunctions);

	std::vector<double> squaredDistanceGradientToTwoGoals =
											stackConstriantGradients(trajectoryPtr);


	auto getToKinematicGoalSquareGradientIndicesOne = GetToKinematicGoalSquareGradientIndices(pointDimension,
																							kinematicDimension,
																							goalOneTimeIndex);
	auto getToKinematicGoalSquareGradientIndicesTwo = GetToKinematicGoalSquareGradientIndices(pointDimension,
																							kinematicDimension,
																							goalTwoTimeIndex);
	std::vector<ConstraintGradientIndicesFunction> getToKinematicGoalSquareGradientIndices = {getToKinematicGoalSquareGradientIndicesOne,
																						getToKinematicGoalSquareGradientIndicesTwo};
	auto stackedKinematicGoalSquareGradientIndices = StackConstriantGradientIndices(getToKinematicGoalSquareGradientIndices);

	auto [numConstraints, gradientRows, gradientCols] = stackedKinematicGoalSquareGradientIndices(constraintIndex);

	EXPECT_THAT(squaredDistanceGradientToTwoGoals,
							ElementsAre(-2, -4, -6, -8, 6, 8, 10, 12));
	EXPECT_EQ(numConstraints, 2 * kinematicDimension);
	EXPECT_THAT(gradientRows, ElementsAre(0, 1, 2, 3, 4, 5, 6, 7));
	EXPECT_THAT(gradientCols, ElementsAre(0, 1, 2, 3, 6, 7, 8, 9));
}

TEST_F(kinematicGoalConstraintTest, twoKinematicGoalConstraintGradientsUsingApplyFunction){
	const unsigned goalOneTimeIndex = 0; 
	const unsigned goalTwoTimeIndex = 1;
	const unsigned constraintIndex = 0;

	std::vector<double> kinematicGoalOne = {{1, 2, 3, 4}};
	std::vector<double> kinematicGoalTwo = {{-1, -1, -1, -1}};

	const double* trajectoryPtr = trajectory.data();

	std::vector<ConstraintFunction> twoGoalConstraintFunctions;
	std::vector<ConstraintGradientFunction> twoGoalConstraintGradientFunctions;
	std::vector<ConstraintGradientIndicesFunction> getToKinematicGoalSquareGradientIndices;

	std::tie(twoGoalConstraintFunctions, twoGoalConstraintGradientFunctions, getToKinematicGoalSquareGradientIndices) =
		applyGetToKinematicGoalSquareConstraint(twoGoalConstraintFunctions,
												twoGoalConstraintGradientFunctions,
												getToKinematicGoalSquareGradientIndices,
												numberOfPoints,
												 pointDimension,
												 kinematicDimension,
												 goalOneTimeIndex,
												 kinematicGoalOne);

	std::tie(twoGoalConstraintFunctions, twoGoalConstraintGradientFunctions, getToKinematicGoalSquareGradientIndices) =
		applyGetToKinematicGoalSquareConstraint(twoGoalConstraintFunctions,
												twoGoalConstraintGradientFunctions,
												getToKinematicGoalSquareGradientIndices,
												numberOfPoints,
												 pointDimension,
												 kinematicDimension,
												 goalTwoTimeIndex,
												 kinematicGoalTwo);

	auto stackConstriantGradients = StackConstriantGradients(twoGoalConstraintGradientFunctions);
	std::vector<double> squaredDistanceGradientToTwoGoals =
											stackConstriantGradients(trajectoryPtr);

	auto stackedKinematicGoalSquareGradientIndices = StackConstriantGradientIndices(getToKinematicGoalSquareGradientIndices);
	auto [numConstraints, gradientRows, gradientCols] = stackedKinematicGoalSquareGradientIndices(constraintIndex);

	EXPECT_THAT(squaredDistanceGradientToTwoGoals,
							ElementsAre(-2, -4, -6, -8, 6, 8, 10, 12));
	EXPECT_EQ(numConstraints, 2 * kinematicDimension);
	EXPECT_THAT(gradientRows, ElementsAre(0, 1, 2, 3, 4, 5, 6, 7));
	EXPECT_THAT(gradientCols, ElementsAre(0, 1, 2, 3, 6, 7, 8, 9));
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

	auto getStackConstriants = StackConstriants(twoStepConstraintFunctions);

	auto twoStepKinematicViolations = getStackConstriants(trajectoryPtr);

	EXPECT_THAT(twoStepKinematicViolations,
							ElementsAre(-0.125, -0.25, -0.25, -0.5, -1, -1.75, -0.25, -1.25));
}

TEST_F(blockDynamic, oneTimeStepViolationGradient){
	const unsigned timeIndex = 0;
	const unsigned constraintIndex = 0;
	DynamicFunction blockDynamics = BlockDynamics;
	auto getKinematicViolationGradient = GetKinematicViolationGradient(blockDynamics,
																		pointDimension,
																		positionDimension,
																		timeIndex, 
																		dt);
	std::vector<double> kinematicViolationGradient = getKinematicViolationGradient(trajectoryPtr);

	auto getKinematicViolationGradientIndices = GetKinematicViolationGradientIndices(pointDimension,
																						positionDimension,
																						timeIndex);
	auto [numConstraints, kinematicViolationGradientRows, kinematicViolationGradientCols] = getKinematicViolationGradientIndices(constraintIndex);
																	

	std::vector<double> expectedPointGradient = {-1.0, -0.25, 1.0, -0.25};
	std::vector<double> expectedGradient = view::cycle(expectedPointGradient) | view::take(16);
	EXPECT_THAT(kinematicViolationGradient,
					ElementsAreArray(expectedGradient));
	EXPECT_EQ(numConstraints, 4);
	EXPECT_THAT(kinematicViolationGradientRows, ElementsAreArray({0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3}));
	EXPECT_THAT(kinematicViolationGradientCols, ElementsAreArray({0, 2, 6, 8, 1, 3, 7, 9, 2, 4, 8, 10, 3, 5, 9, 11}));
}

TEST_F(blockDynamic, twoTimeStepsViolationGradient){
	const unsigned timeIndexZero = 0;
	const unsigned timeIndexOne = 1;
	const unsigned constraintIndex = 0;
	DynamicFunction blockDynamics = BlockDynamics;
	auto getTimeZeroKinematicViolationGradient = GetKinematicViolationGradient(blockDynamics,
															pointDimension,
															positionDimension,
															timeIndexZero, 
															dt);

	auto getTimeOneKinematicViolationGradient = GetKinematicViolationGradient(blockDynamics,
															pointDimension,
															positionDimension,
															timeIndexOne, 
															dt);

	std::vector<ConstraintGradientFunction> twoStepConstraintGradientFunctions = {getTimeZeroKinematicViolationGradient,
																	getTimeOneKinematicViolationGradient};

	auto getStackConstriantGradients = StackConstriantGradients(twoStepConstraintGradientFunctions);
	auto twoStepKinematicViolationGradients = getStackConstriantGradients(trajectoryPtr);

	auto getKinematicViolationGradientIndicesZero = GetKinematicViolationGradientIndices(pointDimension,
																							positionDimension,
																							timeIndexZero);
	auto getKinematicViolationGradientIndicesOne = GetKinematicViolationGradientIndices(pointDimension,
																							positionDimension,
																							timeIndexOne);
	std::vector<ConstraintGradientIndicesFunction> getKinematicViolationGradientIndices = {getKinematicViolationGradientIndicesZero,
																						getKinematicViolationGradientIndicesOne};
	auto stackedKinematicViolationGradientIndices = StackConstriantGradientIndices(getKinematicViolationGradientIndices);

	auto [numConstraints, gradientRows, gradientCols] = stackedKinematicViolationGradientIndices(constraintIndex);

	std::vector<double> expectedPointGradient = {-1.0, -0.25, 1.0, -0.25};
	std::vector<double> expectedGradient = view::cycle(expectedPointGradient) | view::take(32);
	EXPECT_THAT(twoStepKinematicViolationGradients,
					ElementsAreArray(expectedGradient));
	EXPECT_EQ(numConstraints, 8);
	EXPECT_THAT(gradientRows, ElementsAreArray({0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7}));
	EXPECT_THAT(gradientCols, ElementsAreArray({0, 2, 6, 8, 1, 3, 7, 9, 2, 4, 8, 10, 3, 5, 9, 11, 6, 8, 12, 14, 7, 9, 13, 15, 8, 10, 14, 16, 9, 11, 15, 17}));
}

TEST_F(blockDynamic, twoTimeStepsViolationGradientUsingApplyFunction){
	const unsigned timeIndexZero = 0;
	const unsigned timeIndexOne = 1;
	const unsigned startTimeIndex = timeIndexZero;
	const unsigned numTimePoints = 3;
	const unsigned endTimeIndex = startTimeIndex + numTimePoints - 1;
	const unsigned constraintIndex = 0;
	DynamicFunction blockDynamics = BlockDynamics;

	std::vector<ConstraintFunction> twoStepConstraintFunctions;
	std::vector<ConstraintGradientFunction> twoStepConstraintGradientFunctions;
	std::vector<ConstraintGradientIndicesFunction> getKinematicViolationGradientIndices;

	std::tie(twoStepConstraintFunctions, twoStepConstraintGradientFunctions, getKinematicViolationGradientIndices) =
		applyKinematicViolationConstraints(twoStepConstraintFunctions,
											twoStepConstraintGradientFunctions,
											getKinematicViolationGradientIndices,
											blockDynamics,
											pointDimension,
											positionDimension,
											startTimeIndex,
											endTimeIndex,
											dt);

	auto getStackConstriantGradients = StackConstriantGradients(twoStepConstraintGradientFunctions);
	auto twoStepKinematicViolationGradients = getStackConstriantGradients(trajectoryPtr);

	auto stackedKinematicViolationGradientIndices = StackConstriantGradientIndices(getKinematicViolationGradientIndices);

	auto [numConstraints, gradientRows, gradientCols] = stackedKinematicViolationGradientIndices(constraintIndex);

	std::vector<double> expectedPointGradient = {-1.0, -0.25, 1.0, -0.25};
	std::vector<double> expectedGradient = view::cycle(expectedPointGradient) | view::take(32);
	EXPECT_THAT(twoStepKinematicViolationGradients,
					ElementsAreArray(expectedGradient));
	EXPECT_EQ(numConstraints, 8);
	EXPECT_THAT(gradientRows, ElementsAreArray({0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7}));
	EXPECT_THAT(gradientCols, ElementsAreArray({0, 2, 6, 8, 1, 3, 7, 9, 2, 4, 8, 10, 3, 5, 9, 11, 6, 8, 12, 14, 7, 9, 13, 15, 8, 10, 14, 16, 9, 11, 15, 17}));
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
