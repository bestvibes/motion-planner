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
	using DynamicFunction = std::function<dvector(const dvector&,
													const dvector&,
													const dvector&)>;
	using namespace ranges;

	dvector BlockDynamics(const dvector& position,
							const dvector& velocity,
							const dvector& control){
		assert(position.size() == velocity.size());  
		return control;
	}

	std::tuple<dvector, dvector> stepForward(const dvector& position,
											 const dvector& velocity,
											 const dvector& acceleration,
											 const double dt) {
		assert (position.size() == velocity.size()); 
		assert (position.size() == acceleration.size()); 

		auto moveForwardDt  = [dt](auto scaler, auto derivative){
			return scaler + derivative*dt;  
		};

		dvector nextPosition = view::zip_with(moveForwardDt, position, velocity); 
		dvector nextVelocity = view::zip_with(moveForwardDt, velocity, acceleration); 
		return {nextPosition, nextVelocity};
	}

}
