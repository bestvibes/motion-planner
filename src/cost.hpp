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
#include <Eigen/Dense>
#include <exception>
#include <vector>
#include <numeric>
#include "utilities.hpp"

#pragma once

namespace traj_opt::cost{


using Eval_F_Func = std::function<double(const double*)>;

template<unsigned N, unsigned P>
class Control_Cost{
		const int qd;
		const int vd;
		const int ud;

	public:
		Control_Cost(const int qd, const int vd, const int ud):
			qd(qd), vd(vd), ud(ud){
				assert (P == qd+vd+ud);
		}

	double operator()(const double* X) const {
		double* X_1d =  const_cast<double*>(X) ;
		Eigen::Map<Traj<N, P>> x_arr(X_1d, N, P);
		auto U = x_arr.block(0, qd+vd, N, ud);
		auto U2 = U.pow(2);
		double f = U2.sum();
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
		std::vector<double> costs(num_func);
		std::transform(std::begin(cost_funcs),
									 std::end(cost_funcs),
									 std::begin(costs),
									 [x](auto f){return f(x);});

		auto total = std::accumulate(std::begin(costs), std::end(costs), 0);
		return total;
	}
};

}//namespace

