#pragma once
#include <cmath>
#include <cassert>
#include <range/v3/view.hpp> 


namespace trajectoryOptimization::cost{

	class GetControlSquareSum{
		const unsigned numberOfPoints;
		const unsigned pointDimension;
		const unsigned controlDimension;
		int trajectoryDimension; 
		int controlStartIndex; 
		int controlEndIndex;
		public:
			GetControlSquareSum( const unsigned numberOfPoints,
													 const unsigned pointDimension,
													 const unsigned controlDimension):
				numberOfPoints(numberOfPoints),
				pointDimension(pointDimension),
				controlDimension(controlDimension)
		{
				trajectoryDimension = numberOfPoints * pointDimension;
				assert(controlDimension<pointDimension);
				controlStartIndex = pointDimension - controlDimension;
				controlEndIndex = pointDimension;
			};
			double operator()(const double* trajectoryPointer) const {
				using namespace ranges;
			std::vector<double> trajectory(trajectoryPointer,
																		 trajectoryPointer+trajectoryDimension);

			auto squareScaler =  [](auto singleControlValue)
													 {return std::pow(singleControlValue, 2);};

			auto squareAPoint = [=](auto point) {
				return point | ranges::view::slice(controlStartIndex, controlEndIndex) // get u
										 | ranges::view::transform(squareScaler); //square
			};

			auto controlSquare = trajectory | view::chunk(pointDimension)
																		  | view::transform(squareAPoint)
																			| view::join;
			double controlSquareSum = ranges::accumulate(controlSquare, 0);
			return controlSquareSum;
			}  
	};
}//namespace



