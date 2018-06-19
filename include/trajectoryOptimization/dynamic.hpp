#pragma once
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cassert>
#include <functional>
#include <range/v3/view.hpp>

namespace trajectoryOptimization::dynamic {
	using dvector = std::vector<double>;
	using DynamicFunction = std::function<const double*(const double*,
													const unsigned,
													const double*,
													const unsigned,
													const double*,
													const unsigned)>;
	using namespace ranges;

	const double* BlockDynamics(const double* position,
							const unsigned positionDimension,
							const double* velocity,
							const unsigned velocityDimension,
							const double* control,
							const unsigned controlDimension){
		assert(positionDimension == velocityDimension);  
		return control;
	}

	std::tuple<dvector, dvector> stepForward(const dvector& position,
											 const dvector& velocity,
											 const dvector& acceleration,
											 const double dt) {
		assert (position.size() == velocity.size()); 
		assert (position.size() == acceleration.size()); 

		const auto moveForwardDt  = [&dt](auto scaler, auto derivative){
			return scaler + derivative*dt;  
		};

		dvector nextPosition = view::zip_with(moveForwardDt, position, velocity); 
		dvector nextVelocity = view::zip_with(moveForwardDt, velocity, acceleration); 
		return {nextPosition, nextVelocity};
	}

}
