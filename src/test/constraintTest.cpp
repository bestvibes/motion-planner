#include <gtest/gtest.h> 
#include <range/v3/view.hpp>
#include "utilities.hpp"
#include "constraint.hpp"

using namespace trajectoryOptimization::constraint;
using namespace trajectoryOptimization::utilities;
TEST(constraintTest, toGoalDistanceSquare_isZero_whenReachingGoal){
	const unsigned numberOfPoints = 3;    
	const unsigned pointDimension = 3;  
	auto trajectory_QA_areOnes = createTrajectoryWithIdenticalPoints(
															 numberOfPoints,
															 {1, 1, 0});
	const double* trajectory_QA_areOnes_ptr = trajectory_QA_areOnes.data();

	ASSERT_THAT

	// EXPECT_EQ(0, -1);
	// EXPECT_EQ(0, getToGoalDistanceSquareSum(trajectory_QA_areOnes));
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
