/*
 * =====================================================================================
 *
 *       Filename:  utilities.hpp
 *
 *    Description:  utility functions for traj optimization
 *
 *        Version:  1.0
 *        Created:  01/01/2018 04:53:36 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tao Gao, 
 *   Organization:  
 *
 * =====================================================================================
 */
#pragma once
#include <iostream>
#include <array>

namespace traj_opt{

	template<unsigned N>
	using Array = std::array<double, N>;
	// template<unsigned N, unsigned P>
	// using Traj = std::array<Point<P>, N>;

	// template<unsigned N, unsigned P>
	// Traj<N, P> double_to_traj(const double* x_c){
	// 	double* x = const_cast<double*>(x_c) ;
	// 	Traj<N, P> traj;
	// 	for (size_t r=0;r<N;r++){
	// 			auto start = x+r*P;
	// 			std::copy(start, start+P, std::begin(traj[r]));
	// 	}
	// 	return traj;
	// 	}
	template<unsigned N>
	Array<N> double_to_array(const double* x){
		Array<N> traj;
		std::copy(x, x+N, std::begin(traj));
		return traj;
		}

	template<unsigned N, unsigned P>
		Array<P> get_traj_point(Array<N*P> traj, const int n){
			const int start = n*P;
			auto begin = std::begin(traj);
			Array<P> point(begin+start, begin+start+P); 
			return point;
		}
}





