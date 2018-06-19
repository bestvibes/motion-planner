#include <gtest/gtest.h> 
#include <gmock/gmock.h>
#include "trajectoryOptimization/dynamic.hpp"

using namespace testing;
using namespace trajectoryOptimization::dynamic;
TEST(blockDynamic, controlZero){
	dvector position = {1, 2};
	dvector velocity = {0, 0};
	dvector control =  {0, 0};

	auto acceleration = BlockDynamics(position.data(), position.size(), velocity.data(), velocity.size(), control.data(), control.size()); 
	EXPECT_DOUBLE_EQ(acceleration[0], 0);
	EXPECT_DOUBLE_EQ(acceleration[1], 0);
}

TEST(blockDynamic, controlOneTwo){
	dvector position = {1, 2};
	dvector velocity = {1, 2};
	dvector control =  {1, 2};

	auto acceleration = BlockDynamics(position.data(), position.size(), velocity.data(), velocity.size(), control.data(), control.size()); 
	EXPECT_DOUBLE_EQ(acceleration[0], 1);
	EXPECT_DOUBLE_EQ(acceleration[1], 2);
}

TEST(forward, zeroVelocityAndControl){
	dvector position = {1, 2};
	dvector velocity = {0, 0};
	dvector acceleration =  {0, 0};
	const int dt = 1.0;

	auto [nextPosition, nextVelocity] = stepForward(position, velocity, acceleration, dt);

	EXPECT_THAT(position, nextPosition);
	EXPECT_THAT(velocity, nextVelocity);
}

TEST(forward, noneZeroVelocityZeroControl){
	dvector position = {1, 2};
	dvector velocity = {1, 2};
	dvector acceleration =  {0, 0};
	const double dt = 0.5;

	auto [nextPosition, nextVelocity] = stepForward(position, velocity, acceleration, dt);

	EXPECT_THAT(nextPosition, ElementsAre(1.5, 3));
	EXPECT_THAT(nextVelocity, ElementsAre(1, 2));
}


TEST(forward, noneZeroVelocityAndNoneZeroControl){
	dvector position = {1, 2};
	dvector velocity = {1, 2};
	dvector acceleration =  {3, 5};
	const double dt = 0.5;

	auto [nextPosition, nextVelocity] = stepForward(position, velocity, acceleration, dt);

	EXPECT_THAT(nextPosition, ElementsAre(1.5, 3));
	EXPECT_THAT(nextVelocity, ElementsAre(2.5, 4.5));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
