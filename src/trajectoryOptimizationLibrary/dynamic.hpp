#pragma once
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cassert>
#include <functional>
#include <range/v3/view.hpp>

namespace trajectoryOptimization::dynamics{
	using dvector = std::vector<double>; 
	using DynamicFunction = std::function<dvector(const dvector&,
																								const dvector&,
																								const dvector&)>;
	using namespace ranges;

	dvector block(const dvector& position,
								const dvector& velocity,
								const dvector& control){
				assert(position.size() == velocity.size());  
				return control;
	}

	std::tuple<dvector, dvector> forward(const dvector& position,
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


// namespace traj_opt{
// namespace dynamics{
// 	template <unsigned NQ, unsigned NV, unsigned NU>
// 	class Block{
// 		public:
// 			Block(const int _qd, const int _vd, const int _ud):
// 				qd(_qd),
// 				vd(_vd),
// 				ud(_ud){
// 					start = qd+vd;
// 				}
// 			//for a block, the dynamcis (aka accelation) is just the control u;
// 		const Point operator()(const Point& point) const {
// 			auto s_iter = point.begin() + start;
// 			auto e_iter = s_iter + ud; 
// 			Point u_vec(s_iter, e_iter);
// 			return u_vec;
// 		}  
// 		private:
// 			const int qd;
// 			const int vd;
// 			const int ud;
// 			int start;
// 	};
// }
// }
