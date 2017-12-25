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
#include <stdlib.h>
#include <iostream>
#include <map>
#include <string>
#include <valarray>

#pragma once

namespace traj_opt{

namespace cost{
	using Point = std::valarray<const double>;
class Control_Cost{
	public:
		Control_Cost(const int _n, const int _qd, const int _vd, const int _ud):
			n(_n), qd(_qd), vd(_vd), ud(_ud){
				pd = qd + vd + ud;
		}

	double operator()(const Point& x) const {
		auto U = x[std::slice(qd+vd, n, pd)];
		auto U2 = U^2;
		double f = U2.sum();
		return f;
	}
	
	private:
	const int n;
	const int qd;
	const int vd;
	const int ud;
	int pd;

};

}
}
