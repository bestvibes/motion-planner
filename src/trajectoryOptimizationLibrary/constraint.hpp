#pragma once
#include <functional>
#include <cassert>
#include <cmath>
#include <iterator>
#include <functional>
#include <range/v3/all.hpp>
#include "dynamic.hpp"
#include "derivative.hpp"
#include "utilities.hpp"

namespace trajectoryOptimization::constraint {
	using namespace dynamic;
	using namespace trajectoryOptimization::utilities;
	using namespace trajectoryOptimization::derivative;
	using ConstraintFunction = std::function<std::vector<double>(const double*)>;
	using ConstraintGradientFunction = std::function<std::vector<double>(const double*)>;

	class GetToKinematicGoalSquare {
		const unsigned numberOfPoints;
		const unsigned pointDimension; 
		const unsigned kinematicDimension;
		const unsigned goalTimeIndex;
		const std::vector<double> kinematicGoal;
		const unsigned kinematicStartIndex;
		const std::vector<unsigned> kinematicDimensionRange;
		public:
		GetToKinematicGoalSquare(const unsigned numberOfPoints,
								 const unsigned pointDimension,
								 const unsigned kinematicDimension,
								 const unsigned goalTimeIndex, 
								 const std::vector<double> kinematicGoal):
									numberOfPoints(numberOfPoints),
									pointDimension(pointDimension),
									kinematicDimension(kinematicDimension),
									goalTimeIndex(goalTimeIndex),
									kinematicGoal(kinematicGoal),
									kinematicStartIndex(goalTimeIndex * pointDimension),
									kinematicDimensionRange(ranges::view::ints((unsigned) 0, kinematicDimension)) {}

		std::vector<double> operator()(const double* trajectoryPtr) const {

			const auto differenceSquare = [](const auto scaler1, const auto scaler2)
												{ return std::pow(scaler1 - scaler2, 2); };
			const auto currentKinematicsStartPtr = trajectoryPtr+kinematicStartIndex;
			//TODO: control ignored?

			std::vector<double> toKinematicGoalSquare(kinematicDimension);
			std::transform(kinematicDimensionRange.begin(), kinematicDimensionRange.end(),
							toKinematicGoalSquare.begin(),
							[&] (const unsigned kinematicIndex) {
								return differenceSquare(kinematicGoal[kinematicIndex], currentKinematicsStartPtr[kinematicIndex]);
							});

			return toKinematicGoalSquare;
		}
	};

	class GetKinematicViolation {
		const DynamicFunction dynamics;
		const unsigned pointDimension;
		const unsigned positionDimension;
		const unsigned timeIndex;
		const double dt;
		const unsigned velocityDimension;
		const unsigned controlDimension;
		const int currentKinematicsStartIndex;
		const int nextKinematicsStartIndex;
		const std::vector<unsigned> positionDimensionRange;

		public:
			GetKinematicViolation(const DynamicFunction dynamics,
									const unsigned pointDimension,
									const unsigned positionDimension,
									const unsigned timeIndex,
									const double dt):
										dynamics(dynamics),
										pointDimension(pointDimension),
										positionDimension(positionDimension),
										timeIndex(timeIndex),
										dt(dt),
										velocityDimension(positionDimension),
										controlDimension(pointDimension - positionDimension - velocityDimension),
										currentKinematicsStartIndex(timeIndex * pointDimension),
										nextKinematicsStartIndex((timeIndex+1) * pointDimension),
										positionDimensionRange(ranges::view::ints((unsigned) 0, positionDimension)) {
											assert(positionDimension == velocityDimension);
										}

			std::vector<double> operator() (const double* trajectoryPointer) {

				const auto nowPosition = trajectoryPointer + currentKinematicsStartIndex;
				const auto nextPosition = trajectoryPointer + nextKinematicsStartIndex;

				const auto nowVelocity = nowPosition + positionDimension;
				const auto nextVelocity = nextPosition + positionDimension;

				const auto nowControl = nowVelocity + velocityDimension;
				const auto nextControl = nextVelocity + velocityDimension;

				const auto nowAcceleration = dynamics(nowPosition,
														positionDimension,
														nowVelocity,
														velocityDimension,
														nowControl,
														controlDimension);
				const auto nextAcceleration = dynamics(nextPosition,
														positionDimension,
														nextVelocity,
														velocityDimension,
														nextControl,
														controlDimension);

				const auto average = [](const auto val1, const auto val2) { return 0.5 * (val1 + val2); };
				const auto getViolation = [&](const auto now, const auto next, const auto dNow, const auto dNext)
						{ return (next - now) - average(dNow, dNext)*dt; };

				std::vector<double> kinematicViolation(positionDimension+velocityDimension);

				std::transform(positionDimensionRange.begin(), positionDimensionRange.end(),
								kinematicViolation.begin(),
								[nowPosition, nextPosition, nowVelocity, nextVelocity, getViolation](const auto index) {
									return getViolation(nowPosition[index], nextPosition[index], nowVelocity[index], nextVelocity[index]);
								});

				std::transform(positionDimensionRange.begin(), positionDimensionRange.end(),
								kinematicViolation.begin() + positionDimension,
								[nowVelocity, nextVelocity, nowAcceleration, nextAcceleration, getViolation](const auto index) {
									return getViolation(nowVelocity[index], nextVelocity[index], nowAcceleration[index], nextAcceleration[index]);
								});

				return kinematicViolation;
			};
	};

	class StackConstriants {
		const std::vector<ConstraintFunction>& constraintFunctions;
		unsigned numConstraints;

		public:
			StackConstriants(const unsigned numberVariablesInput,
								const std::vector<ConstraintFunction>& constraintFunctions):
				constraintFunctions(constraintFunctions),
				numConstraints(constraintFunctions.size()) {
					const auto x = std::vector<double>(numberVariablesInput, 1);
					const auto constraints = (*this)(x.data());
					numConstraints = constraints.size();
				};

			std::vector<double> operator()(const double* trajectoryPtr) {
				std::vector<double> stackedConstriants;
				stackedConstriants.reserve(numConstraints);

				for (auto const &aFunction: constraintFunctions) {
					const auto constraints = aFunction(trajectoryPtr);
					stackedConstriants.insert(stackedConstriants.end(),
												constraints.begin(),
												constraints.end());
				}
				return stackedConstriants;
			}
	};

	std::vector<ConstraintFunction> applyKinematicViolationConstraints(std::vector<ConstraintFunction> constraints,
																		const DynamicFunction blockDynamics,
							                                            const unsigned timePointDimension,
							                                            const unsigned worldDimension,
							                                            const unsigned timeIndexStart,
							                                            const unsigned timeIndexEndExclusive,
							                                            const double timeStepSize) {
			for (int timeIndex = timeIndexStart; timeIndex < timeIndexEndExclusive; timeIndex++) {
			    constraints.push_back(constraint::GetKinematicViolation(blockDynamics,
			                                                            timePointDimension,
			                                                            worldDimension,
			                                                            timeIndex,
			                                                            timeStepSize));
			}

			return constraints;
		}
}

