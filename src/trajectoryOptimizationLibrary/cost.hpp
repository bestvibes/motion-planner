#pragma once
#include <cmath>
#include <cassert>
#include <range/v3/view.hpp> 


namespace trajectoryOptimization::cost{

	class GetControlSquareSum{
		const unsigned numberOfPoints;
		const unsigned pointDimension;
		const unsigned controlDimension;
		const int trajectoryDimension; 
		const int controlStartIndex; 
		const int controlEndIndex;
		public:
			GetControlSquareSum(const unsigned numberOfPoints,
								const unsigned pointDimension,
								const unsigned controlDimension):
									numberOfPoints(numberOfPoints),
									pointDimension(pointDimension),
									controlDimension(controlDimension),
									trajectoryDimension(numberOfPoints * pointDimension),
									controlStartIndex(pointDimension - controlDimension),
									controlEndIndex(pointDimension)
								{
									assert(controlDimension<pointDimension);
								};

			double operator()(const double* trajectoryPointer) const {
				std::vector<double> trajectory(trajectoryPointer,
											 	trajectoryPointer+trajectoryDimension);

				auto squareScaler =  [](auto singleControlValue)
										 {return std::pow(singleControlValue, 2);};

				auto squareAPoint = [=](auto point) {
					return point | ranges::view::slice(controlStartIndex, controlEndIndex) // get u
								 | ranges::view::transform(squareScaler); //square
				};

				auto controlSquare = trajectory | ranges::view::chunk(pointDimension)
												  | ranges::view::transform(squareAPoint)
													| ranges::view::join;
				double controlSquareSum = ranges::accumulate(controlSquare, 0);
				return controlSquareSum;
			}  
	};

	class GetControlSquareSumGradient{
		const unsigned numberOfPoints;
		const unsigned pointDimension;
		const unsigned controlDimension;
		const int trajectoryDimension; 
		const int controlStartIndex; 
		const int controlEndIndex;
		public:
			GetControlSquareSumGradient(const unsigned numberOfPoints,
										const unsigned pointDimension,
										const unsigned controlDimension):
											numberOfPoints(numberOfPoints),
											pointDimension(pointDimension),
											controlDimension(controlDimension),
											trajectoryDimension(numberOfPoints * pointDimension),
											controlStartIndex(pointDimension - controlDimension),
											controlEndIndex(pointDimension)
										{
											assert(controlDimension<pointDimension);
										};

			std::vector<double> operator()(const double* trajectoryPointer) const {
				std::vector<double> trajectory(trajectoryPointer,
											 	trajectoryPointer+trajectoryDimension);

				auto doubleControlScaler =  [](auto singleControlValue)
										 {return 2 * singleControlValue;};

				std::vector<double> timePointWithoutControlGradient(pointDimension - controlDimension);

				auto doubleControl = [=](auto point) {
					return ranges::view::concat(timePointWithoutControlGradient, point | ranges::view::slice(controlStartIndex, controlEndIndex) // get u
								 															| ranges::view::transform(doubleControlScaler)); //square
				};

				auto controlDoubled = trajectory | ranges::view::chunk(pointDimension)
												  | ranges::view::transform(doubleControl)
													| ranges::view::join;
				return controlDoubled;
			}  
	};
}//namespace



