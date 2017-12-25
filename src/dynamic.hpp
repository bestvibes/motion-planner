/*
 * =====================================================================================
 *
 *       Filename:  constraint.hpp
 *
 *    Description:  constriant functions for traj_optimization
 *
 *        Version:  1.0
 *        Created:  12/24/2017 12:58:08 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tao Gao , 
 *   Organization:  UCLA
 *
 * =====================================================================================
 */
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Dense>


namespace traj_opt{
namespace dynamics{
	//maybe define it at a top level namespace ? 
	using Point =  std::vector<const double>;

	class Block{
		public:
			Block(const int _qd, const int _vd, const int _ud):
				qd(_qd),
				vd(_vd),
				ud(_ud){
					start = qd+vd;
				}
			//for a block, the dynamcis (aka accelation) is just the control u;
		const Point operator()(const Point& point) const {
			auto s_iter = point.begin() + start;
			auto e_iter = s_iter + ud; 
			Point u_vec(s_iter, e_iter);
			return u_vec;
		}  
		private:
			const int qd;
			const int vd;
			const int ud;
			int start;
	};
}
}
