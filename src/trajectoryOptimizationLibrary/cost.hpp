/*
 * =====================================================================================
 *
 *       Filename:  cost.cpp
 *
 *    Description:  the cost function of a trajectory
 *
 *        Version:  1.0
 *        Created:  12/24/2017 10:42:04 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tao Gao
 *   Organization:  UCLA
 *
 * =====================================================================================
 */
#include <iostream>
#include <string>
#include <cmath>
#include <exception>
#include <vector>
#include <numeric>
#include <range/v3/all.hpp> 
#include "utilities.hpp"

#pragma once

namespace traj_opt::cost{


using Eval_F_Func = std::function<double(const double*)>;

template<unsigned N, unsigned P>
class Control_Cost{
		const int qd;
		const int vd;
		const int ud;
		int qvd;

	public:
		Control_Cost(const int qd, const int vd, const int ud):
			qd(qd), vd(vd), ud(ud){
				assert (P == qd+vd+ud);
				qvd = qd+vd;
		}

	double operator()(const double* X) const {
		std::vector<double> x_vec(X, X+N*P);
		auto line_func = [=](auto line) {
			return line | ranges::view::slice(qvd, qvd+ud) // get u
								  | ranges::view::transform([](auto u){return std::pow(u, 2);}); //square
		};
		auto x_arr_rng = x_vec | ranges::view::chunk(P); 
		auto u_rng = x_arr_rng|ranges::view::transform(line_func)
													|ranges::view::join;
		double f = ranges::accumulate(u_rng, 0);
		return f;
	}
};

class Sum_Cost{
	const std::vector<Eval_F_Func>& cost_funcs;
	size_t num_func;
	public:
		Sum_Cost(const std::vector<Eval_F_Func>& cost_funcs):
			cost_funcs(cost_funcs){
				num_func = cost_funcs.size();
			}

	double operator()(const double* x){
		double total = ranges::accumulate(
							cost_funcs | ranges::view::transform([x](auto f){return f(x);}),
							0);
		return total;
	}
};

}//namespace

