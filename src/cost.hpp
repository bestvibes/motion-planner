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

#pragma once

namespace traj_opt{

namespace cost{
	// using Point = std::valarray<const double>;
	using namespace Eigen;
	using Traj = Array<double, Dynamic, Dynamic, RowMajor>;
	// using Point = Array<double, Dynamic, Dynamic, RowMajor>;
class Control_Cost{
	public:
		Control_Cost(const int _n, const int _qd, const int _vd, const int _ud):
			n(_n), qd(_qd), vd(_vd), ud(_ud){
				pd = qd + vd + ud;
				size = n*pd;
		}

	double operator()(const double* X) const {
		//FIXME: not sure whether this expression is correct;
		double* X_1d =  const_cast<double*>(X) ;
		Map<Traj> x_arr(X_1d, n, pd);
		auto U = x_arr.block(0, qd+vd, n, ud);
		auto U2 = U.pow(2);
		double f = U2.sum();
		return f;
	}
	
	private:
	const int n;
	const int qd;
	const int vd;
	const int ud;
	int pd;
	int size;

};

}
}
