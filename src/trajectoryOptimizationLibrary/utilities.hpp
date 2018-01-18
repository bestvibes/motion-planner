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

// 	template<unsigned N>
// 	using Array = std::array<double, N>;
//
// // using Point = std::valarray<const double>;
// 	template<unsigned N, unsigned P>
// 	using Traj = Eigen::Array<double, N, P, Eigen::RowMajor>;
//
// 	// template<unsigned P>
// 	// using Point = Eigen::Array<double, 1, P, Eigen::RowMajor>;
//
// 	template<unsigned P>
// 		Array<P> get_traj_point(const double* traj, const int n){
// 			const int start = n*P;
// 			Array<P> point(traj+start, traj+start+P); 
// 			return point;
// 		}
//
//
// 	template<unsigned NQ, unsigned NV, unsigned NU>
// 		std::tuple<Array<NQ>, Array<NV>, Array<NU>>
// 		get_q_v_u(const double*x, const int t){
// 			constexpr auto P = NQ+NV+NU; 
// 			Array<NQ> q_arr;
// 			Array<NV> v_arr;
// 			Array<NU> u_arr;
// 			auto q_start = x+t*P;
// 			auto q_end = q_start+NQ;
// 			auto v_end = q_end+NV;
// 			auto u_end = v_end + NU;
// 			std::copy(q_start, q_end, std::begin(q_arr));
// 			std::copy(q_end, v_end, std::begin(v_arr));
// 			std::copy(v_end, u_end, std::begin(u_arr));
// 			return {q_arr, v_arr, u_arr};
// 		}






