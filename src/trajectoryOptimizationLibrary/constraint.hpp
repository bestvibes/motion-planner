#pragma once
#include <functional>
#include <cmath>
#include <iterator>
#include <functional>
#include <range/v3/all.hpp>
#include "dynamic.hpp"
#include  "utilities.hpp"

namespace trajectoryOptimization::constraint{
	using namespace ranges;
	using namespace dynamics;
	using namespace trajectoryOptimization::utilities;
	using constraintFunction = std::function<std::vector<double>(const double*)>; 

	class GetToKinematicGoalSquare{
		const unsigned numberOfPoints;
		const unsigned pointDimension; 
		const unsigned kinematicDimension;
		const unsigned goalTimeIndex;
		const std::vector<double>& kinematicGoal;
		int kinematicStartIndex;
		int kinematicEndIndex;
		public:
		GetToKinematicGoalSquare(const unsigned numberOfPoints, 
														 const unsigned pointDimension,
														 const unsigned kinematicDimension,
														 const unsigned goalTimeIndex, 
														 const std::vector<double>& kinematicGoal):
			numberOfPoints(numberOfPoints),
			pointDimension(pointDimension),
			kinematicDimension(kinematicDimension),
			goalTimeIndex(goalTimeIndex),
			kinematicGoal(kinematicGoal){
			kinematicStartIndex = goalTimeIndex * pointDimension;
		}
		auto operator()(const double* trajectoryPtr) const{
			auto differentSquare = [](auto scaler1, auto scaler2)
																{return std::pow(scaler1-scaler2 ,2);};
			std::vector<double> currentKinematics;
			auto currentKinematicsStartPtr = trajectoryPtr+kinematicStartIndex;
			std::copy_n(currentKinematicsStartPtr,
									kinematicDimension,
									std::back_inserter(currentKinematics));

			auto toKinematicGoalSquareRange =
					 view::zip_with(differentSquare, kinematicGoal, currentKinematics);

			std::vector<double> toKinematicGoalSquare =
													yield_from(toKinematicGoalSquareRange);

			return toKinematicGoalSquare;
			}
	
	};

	class GetKinematicViolation{
		DynamicFunction dynamics; 
		const unsigned pointDimension;
		const unsigned positionDimension; 
		const unsigned timeIndex;
		unsigned velocityDimension;
		unsigned controlDimension;
		int currentKinematicsStartIndex; 
		int nextKinematicsStartIndex; 

		public:
			GetKinematicViolation(DynamicFunction dynamics,
														const unsigned pointDimension,
														const unsigned positionDimension,
														const unsigned timeIndex):
				dynamics(dynamics),
				pointDimension(pointDimension),
				positionDimension(positionDimension),
				timeIndex(timeIndex){
					// assert (kinematicDimension/2 == 0);
					velocityDimension = positionDimension;
					controlDimension = pointDimension - positionDimension- velocityDimension;
					currentKinematicsStartIndex = timeIndex*pointDimension; 
					nextKinematicsStartIndex = (timeIndex+1)*pointDimension;
				}

			std::vector<double> operator () (const double* trajectoryPointer){
				auto nowPoint = getTrajectoryPoint(trajectoryPointer,
																					 timeIndex,
																					 pointDimension);
				auto nextPoint = getTrajectoryPoint(trajectoryPointer,
																						timeIndex+1,
																						pointDimension);

				const auto& [nowPosition, nowVelocity, nowControl] = 
					getPointPositionVelocityControl(nowPoint,
																					positionDimension,
																					velocityDimension,
																					controlDimension);

				const auto& [nextPosition, nextVelocity, nextControl] = 
					getPointPositionVelocityControl(nextPoint,
																					positionDimension,
																					velocityDimension,
																					controlDimension);

				auto acceleration = dynamics(nowPosition, nowVelocity, nowControl);

				// std::vector<double> nowPostion(pointDimension);
				// std::vector<double> nowVelocity(pointDimension);
				// std::vector<double> nowVelocity(pointDimension);
        //
				// std::copy_n(trajectoryPtr, trajectoryPtr+pointDimension, std::begin(nowPostion));
				return {0, 0, 0, 0};
			
			};
	
	};

	class StackConstriants{
		const std::vector<constraintFunction>& constraintFunctions;
		public:
			StackConstriants(const std::vector<constraintFunction>& constraintFunctions):
				constraintFunctions(constraintFunctions){};

			std::vector<double> operator()(const double* trajectoryPtr){

				std::vector<double> stackedConstriants;
				for (auto aFunction: constraintFunctions){
					auto constraints = aFunction(trajectoryPtr);
					std::copy(std::begin(constraints), std::end(constraints),
										std::back_inserter(stackedConstriants));
				}
			return stackedConstriants;
		}
	};
}
// 	template <unsigned NQ, unsigned NV, unsigned NU>
// 	using Dynamic_Func = std::function<Array<NQ+NV>(Array<NQ>, Array<NV>, Array<NU>)>;
// 	template<unsigned P>
// 	class Goal_Constraint{
// 			const Array<P>& goal;
// 			const int t; 
// 			int start; 
// 			int end;
// 		public:
// 			Goal_Constraint(const Array<P>& goal, const int t): goal(goal), t(t){
// 				start = t*P;
// 				end = start + P;
// 			}
//
// 			const Array<P> operator()(const double* x) const {
// 				Array<P> diff;
// 				auto goal_start = std::begin(goal);
// 				std::transform(x+start,
// 											 x+end,
// 											 goal_start,
// 											 std::begin(diff),
// 											 [](auto v1, auto v2){ return std::pow(v1-v2, 2);});
// 				return diff;
// 			};
// 	};
//
// 	template <unsigned NQ, unsigned NV, unsigned NU>
// 	class Point_Dynamic_Constraint{
// 			Dynamic_Func<NQ, NV, NU> dynamics;
// 			const int t;
// 			const double dt;
// 		public:
// 			Point_Dynamic_Constraint(Dynamic_Func<NQ, NV, NU> dynamics,
// 															 const int t, const double dt):
// 															 dynamics(dynamics), t(t), dt(dt){
// 				}
//
// 			const Array<NQ+NV> operator()(const double* x) const{
// 				auto const & [q0_arr, v0_arr, u0_arr] = get_q_v_u<NQ, NV, NU>(x, t);
// 				auto const & [q1_arr, v1_arr, u1_arr] = get_q_v_u<NQ, NV, NU>(x, t+1);
//
// 				auto qv0_rng = ranges::view::concat(q0_arr, v0_arr);
// 				auto qv1_rng = ranges::view::concat(q1_arr, v1_arr);
//
// 				auto d0_rng = ranges::view::concat(v0_arr, dynamics(q0_arr, v0_arr, u0_arr));
// 				auto d1_rng = ranges::view::concat(v1_arr, dynamics(q1_arr, v1_arr, u1_arr));
//
// 				auto grad_func = [=](auto qv0, auto qv1, auto d0, auto d1){
// 					return qv1 - qv0 - 0.5*dt*(d0+d1);
// 				};
//
// 				auto error_rng = ranges::view::zip_with(grad_func, qv0_rng, qv1_rng,
// 																						d0_rng, d1_rng);  
// 				std::vector<double> error_vec = ranges::yield_from(error_rng);
// 				Array<NQ+NV> error;
// 				std::copy_n(error_vec.begin(), NQ+NV, std::begin(error));
// 				return error;
// 			};
// 	};
