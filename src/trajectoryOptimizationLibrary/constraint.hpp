#pragma once
#include <functional>
#include <cassert>
#include <cmath>
#include <iterator>
#include <functional>
#include <range/v3/all.hpp>
#include "dynamic.hpp"
#include "utilities.hpp"

namespace trajectoryOptimization::constraint {
	using namespace ranges;
	using namespace dynamic;
	using namespace trajectoryOptimization::utilities;
	using ConstraintFunction = std::function<std::vector<double>(const double*)>;
	using ConstraintGradientFunction = std::function<std::vector<double>(const double*)>;
	using ConstraintGradientIndicesFunction = std::function<std::tuple<unsigned, std::vector<int>, std::vector<int>>(const unsigned)>;

	class GetToKinematicGoalSquare{
		const unsigned numberOfPoints;
		const unsigned pointDimension; 
		const unsigned kinematicDimension;
		const unsigned goalTimeIndex;
		const std::vector<double>& kinematicGoal;
		const unsigned kinematicStartIndex;
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
									kinematicGoal(kinematicGoal),
									kinematicStartIndex(goalTimeIndex * pointDimension) {}

		auto operator()(const double* trajectoryPtr) const {

			auto differenceSquare = [](auto scaler1, auto scaler2)
										{return std::pow(scaler1-scaler2 ,2);};
			std::vector<double> currentKinematics;
			auto currentKinematicsStartPtr = trajectoryPtr+kinematicStartIndex;
			std::copy_n(currentKinematicsStartPtr,
						kinematicDimension,
						std::back_inserter(currentKinematics));

			auto toKinematicGoalSquareRange =
					 view::zip_with(differenceSquare, kinematicGoal, currentKinematics);

			std::vector<double> toKinematicGoalSquare = yield_from(toKinematicGoalSquareRange);

			return toKinematicGoalSquare;
		}
	};

	class GetToKinematicGoalSquareGradient {
		const unsigned numberOfPoints;
		const unsigned pointDimension; 
		const unsigned kinematicDimension;
		const unsigned goalTimeIndex;
		const std::vector<double>& kinematicGoal;
		const unsigned kinematicStartIndex;
		public:
		GetToKinematicGoalSquareGradient(const unsigned numberOfPoints,
								 const unsigned pointDimension,
								 const unsigned kinematicDimension,
								 const unsigned goalTimeIndex, 
								 const std::vector<double>& kinematicGoal):
									numberOfPoints(numberOfPoints),
									pointDimension(pointDimension),
									kinematicDimension(kinematicDimension),
									goalTimeIndex(goalTimeIndex),
									kinematicGoal(kinematicGoal),
									kinematicStartIndex(goalTimeIndex * pointDimension) {}

		auto operator()(const double* trajectoryPtr) const {

			auto differentSquareDerivative = [](auto scaler1, auto scaler2)
										{return -2 * (scaler1-scaler2);};
			std::vector<double> currentKinematics;
			auto currentKinematicsStartPtr = trajectoryPtr+kinematicStartIndex;
			std::copy_n(currentKinematicsStartPtr,
						kinematicDimension,
						std::back_inserter(currentKinematics));

			auto toKinematicGoalSquareGradientRange =
					 view::zip_with(differentSquareDerivative, kinematicGoal, currentKinematics);

			std::vector<double> toKinematicGoalSquareGradient = yield_from(toKinematicGoalSquareGradientRange);

			return toKinematicGoalSquareGradient;
		}
	};

	class GetToKinematicGoalSquareGradientIndices {
		const unsigned numberConstraints;
		const unsigned kinematicDimension;
		const unsigned kinematicStartIndex;
		public:
		GetToKinematicGoalSquareGradientIndices(const unsigned pointDimension,
														const unsigned kinematicDimension,
														const unsigned goalTimeIndex) :
															numberConstraints(kinematicDimension),
															kinematicDimension(kinematicDimension),
															kinematicStartIndex(goalTimeIndex * pointDimension) {}

		std::tuple<unsigned, std::vector<int>, std::vector<int>> operator()(const unsigned constraintIndex) {
			const auto jacobianRowIndices = view::ints(constraintIndex, constraintIndex + kinematicDimension);
			const auto jacobianColIndices = view::ints(kinematicStartIndex, kinematicStartIndex + kinematicDimension);
			return {numberConstraints, jacobianRowIndices, jacobianColIndices};
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
										nextKinematicsStartIndex((timeIndex+1) * pointDimension) {}

			std::vector<double> operator() (const double* trajectoryPointer) {
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

				auto average = [](auto val1, auto val2) {return 0.5 * (val1 + val2);};
				auto getViolation = [=](auto now, auto next, auto dNow, auto dNext)
						{return (next - now) - average(dNow, dNext)*dt;};

				auto positionViolation = view::zip_with(getViolation,
														nowPosition,
														nextPosition,
														nowVelocity,
														nextVelocity);

				auto nowAcceleration = dynamics(nowPosition, nowVelocity, nowControl);
				auto nextAcceleration = dynamics(nextPosition, nextVelocity, nextControl);

				auto velocityViolation = view::zip_with(getViolation,
														nowVelocity,
														nextVelocity,
														nowAcceleration,
														nextAcceleration);

				std::vector<double> kinematicViolation = yield_from(view::concat(positionViolation,
																			 		velocityViolation));
				return kinematicViolation;
			};
	};

	class GetKinematicViolationGradient {
		const DynamicFunction dynamics; 
		const unsigned pointDimension;
		const unsigned positionDimension; 
		const unsigned timeIndex;
		const double dt;
		const unsigned velocityDimension;
		const unsigned controlDimension;
		const int currentKinematicsStartIndex; 
		const int nextKinematicsStartIndex; 

		public:
			GetKinematicViolationGradient(const DynamicFunction dynamics,
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
										nextKinematicsStartIndex((timeIndex+1) * pointDimension) {}

			std::vector<double> operator() (const double* trajectoryPointer) {
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

				const auto nowDerivative = [](auto now) {return -1.0;};
				const auto nextDerivative = [](auto next) {return 1.0;};
				const auto dDerivative = [=](auto d) {return -0.5*dt;};
				const auto getViolationGradient = [=](auto now, auto next, auto dNow, auto dNext)
						{return std::vector<double> {nowDerivative(now),
														dDerivative(dNow),
														nextDerivative(next),
														dDerivative(dNext)};};

				auto positionViolationGradientVectors = view::zip_with(getViolationGradient,
																		nowPosition,
																		nextPosition,
																		nowVelocity,
																		nextVelocity);
				auto positionViolationGradient = action::join(positionViolationGradientVectors);

				auto nowAcceleration = dynamics(nowPosition, nowVelocity, nowControl);
				auto nextAcceleration = dynamics(nextPosition, nextVelocity, nextControl);

				auto velocityViolationGradientVectors = view::zip_with(getViolationGradient,
																nowVelocity,
																nextVelocity,
																nowAcceleration,
																nextAcceleration);
				auto velocityViolationGradient = action::join(velocityViolationGradientVectors);

				std::vector<double> kinematicViolationGradient = view::concat(positionViolationGradient,
																	 			velocityViolationGradient);
				return kinematicViolationGradient;
			};
	};

	class GetKinematicViolationGradientIndices {
		const unsigned numberDerivatives = 4;
		const unsigned pointDimension;
		const unsigned positionDimension;
		unsigned numberConstraints;
		unsigned kinematicDimension;
		const unsigned kinematicStartIndex;
		public:
		GetKinematicViolationGradientIndices(const unsigned pointDimension,
												const unsigned positionDimension,
												const unsigned timeIndex) :
															pointDimension(pointDimension),
															positionDimension(positionDimension),
															kinematicStartIndex(timeIndex * pointDimension) {
																const unsigned velocityDimension = positionDimension;
																kinematicDimension = positionDimension + velocityDimension;
																numberConstraints = kinematicDimension;
															}

		std::tuple<unsigned, std::vector<int>, std::vector<int>> operator()(const unsigned constraintIndex) {
			const std::vector<int> jacobianRowIndices = view::for_each(view::ints(constraintIndex, constraintIndex + kinematicDimension),
														[=](auto index) {return yield_from(view::repeat_n(index, numberDerivatives));});
			std::vector<int> jacobianColIndices;
			for (unsigned i = kinematicStartIndex; i < kinematicStartIndex + kinematicDimension; i++) {
				const unsigned now = i;
				const unsigned dNow = i + positionDimension;
				const unsigned next = i + pointDimension;
				const unsigned dNext = i + pointDimension + positionDimension;

				jacobianColIndices.push_back(now);
				jacobianColIndices.push_back(dNow);
				jacobianColIndices.push_back(next);
				jacobianColIndices.push_back(dNext);
			}

			return {numberConstraints, jacobianRowIndices, jacobianColIndices};
		}
	};

	class StackConstriants{
		const std::vector<ConstraintFunction>& constraintFunctions;
		public:
			StackConstriants(const std::vector<ConstraintFunction>& constraintFunctions):
				constraintFunctions(constraintFunctions) {};

			std::vector<double> operator()(const double* trajectoryPtr) {
				std::vector<double> stackedConstriants;
				for (auto aFunction: constraintFunctions) {
					auto constraints = aFunction(trajectoryPtr);
					std::copy(std::begin(constraints), std::end(constraints),
										std::back_inserter(stackedConstriants));
				}
				return stackedConstriants;
			}
	};

	class StackConstriantGradients {
		const std::vector<ConstraintGradientFunction>& constraintGradientFunctions;
		public:
			StackConstriantGradients(const std::vector<ConstraintGradientFunction>& constraintGradientFunctions):
				constraintGradientFunctions(constraintGradientFunctions) {};

			std::vector<double> operator()(const double* trajectoryPtr) {
				std::vector<double> stackedConstriantGradients;
				for (auto aFunction: constraintGradientFunctions) {
					auto constraintGradient = aFunction(trajectoryPtr);
					std::copy(std::begin(constraintGradient), std::end(constraintGradient),
										std::back_inserter(stackedConstriantGradients));
				}
				return stackedConstriantGradients;
			}
	};

	class StackConstriantGradientIndices {
		const std::vector<ConstraintGradientIndicesFunction>& constraintGradientIndicesFunctions;
		public:
			StackConstriantGradientIndices(const std::vector<ConstraintGradientIndicesFunction>& constraintGradientIndicesFunctions):
				constraintGradientIndicesFunctions(constraintGradientIndicesFunctions) {};

			std::tuple<unsigned, std::vector<int>, std::vector<int>> operator()(const unsigned constraintIndex) {
				unsigned currentConstraintIndex = 0;
				unsigned numberConstraints;
  				std::vector<int> constraintGradientRows, constraintGradientCols;

  				int totalNumberConstraints = constraintIndex;
				std::vector<int> jacStructureRows, jacStructureCols;
				for (auto aFunction: constraintGradientIndicesFunctions) {
					std::tie(numberConstraints, constraintGradientRows, constraintGradientCols) =
																	aFunction(totalNumberConstraints);

					totalNumberConstraints += numberConstraints;
					std::copy(std::begin(constraintGradientRows), std::end(constraintGradientRows),
										std::back_inserter(jacStructureRows));
					std::copy(std::begin(constraintGradientCols), std::end(constraintGradientCols),
										std::back_inserter(jacStructureCols));
				}
				return {totalNumberConstraints, jacStructureRows, jacStructureCols};
			}
	};

	std::tuple<std::vector<ConstraintFunction>, std::vector<ConstraintGradientFunction>, std::vector<ConstraintGradientIndicesFunction>>
		applyGetToKinematicGoalSquareConstraint(std::vector<ConstraintFunction> constraints,
											std::vector<ConstraintGradientFunction> constraintGradients,
											std::vector<ConstraintGradientIndicesFunction> constraintGradientIndices,
											const unsigned numberOfPoints,
											const unsigned pointDimension,
											const unsigned kinematicDimension,
											const unsigned goalTimeIndex, 
											const std::vector<double>& kinematicGoal) {
			constraints.push_back(constraint::GetToKinematicGoalSquare(numberOfPoints,
                                                              pointDimension,
                                                              kinematicDimension,
                                                              goalTimeIndex,
                                                              kinematicGoal));
			constraintGradients.push_back(constraint::GetToKinematicGoalSquareGradient(numberOfPoints,
			                                                              pointDimension,
			                                                              kinematicDimension,
			                                                              goalTimeIndex,
			                                                              kinematicGoal));
			constraintGradientIndices.push_back(constraint::GetToKinematicGoalSquareGradientIndices(pointDimension,
                                                                        kinematicDimension,
                                                                        goalTimeIndex));

			return {constraints, constraintGradients, constraintGradientIndices};
		}

	std::tuple<std::vector<ConstraintFunction>, std::vector<ConstraintGradientFunction>, std::vector<ConstraintGradientIndicesFunction>>
		applyKinematicViolationConstraints(std::vector<ConstraintFunction> constraints,
											std::vector<ConstraintGradientFunction> constraintGradients,
											std::vector<ConstraintGradientIndicesFunction> constraintGradientIndices,
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
			    constraintGradients.push_back(constraint::GetKinematicViolationGradient(blockDynamics,
			                                                                            timePointDimension,
			                                                                            worldDimension,
			                                                                            timeIndex,
			                                                                            timeStepSize));
			    constraintGradientIndices.push_back(constraint::GetKinematicViolationGradientIndices(timePointDimension,
			                                                                        worldDimension,
			                                                                        timeIndex));
			}

			return {constraints, constraintGradients, constraintGradientIndices};
		}
}

