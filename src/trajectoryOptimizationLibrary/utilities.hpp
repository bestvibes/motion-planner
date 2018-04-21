#pragma once
#include <iostream>
#include <vector>
#include <cassert>
#include <algorithm>
#include <range/v3/view.hpp>

namespace trajectoryOptimization::utilities{
	using namespace ranges;

	 std::vector<double> createTrajectoryWithIdenticalPoints(
											 unsigned numberOfPoints,
											 const std::vector<double>& singlePoint){

		auto trajectoryDimension = numberOfPoints * singlePoint.size();
		auto trajectorWithIndeticalPoints_Range = view::all(singlePoint)
																						| view::cycle
																						| view::take(trajectoryDimension);
		std::vector<double> trajectoryWithIdenticalPoints
												= yield_from(trajectorWithIndeticalPoints_Range);
		return trajectoryWithIdenticalPoints;
	 }

	 std::vector<double> getTrajectoryPoint(const double* trajectoryPointer, 
			 																		const unsigned timeIndex,
																					const unsigned pointDimension){
		 auto startIndex = trajectoryPointer + timeIndex * pointDimension; 
		 std::vector<double> point(pointDimension);
		 std::copy_n(startIndex, pointDimension, std::begin(point));
		 return point;
	 }

	  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
		  getPointPositionVelocityControl(const std::vector<double> point,
		 		 														 const unsigned positionDimension,
		 																 const unsigned velocityDimension,
		 																 const unsigned controlDimension){
		 	 const unsigned pointDimension =
		 		 							positionDimension+velocityDimension+controlDimension;
		 	 assert (point.size()==pointDimension);
			 std::vector<double> position(positionDimension);
			 std::vector<double> velocity(velocityDimension);
			 std::vector<double> control(controlDimension);

			 auto begin =std::begin(point);
			 auto positionBegin = begin;
			 auto velocityBegin = positionBegin+positionDimension; 
			 auto controlBegin = velocityBegin+velocityDimension;

			 std::copy_n(positionBegin, positionDimension, std::begin(position));
			 std::copy_n(velocityBegin, velocityDimension, std::begin(velocity));
			 std::copy_n(controlBegin, controlDimension, std::begin(control));
			 return {position, velocity, control};
		  }

}
