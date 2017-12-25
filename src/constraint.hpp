/*
 * =====================================================================================
 *
 *       Filename:  constraint.hpp
 *
 *    Description:  constraints for trajectory optimization
 *
 *        Version:  1.0
 *        Created:  12/24/2017 05:01:27 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tao Gao 
 *   Organization:  UCLA
 *
 * =====================================================================================
 */
#include <vector>
#include <valarray>
#include <functional>

namespace traj_opt{
namespace constriant{
	using Point = std::valarray<const double>;
	using Dynamic_Func = std::function<const Point(const Point&)>;
	class Goal_Constraint{
		public:
			Goal_Constraint(const Point& goal): goal(goal){
			}

			const Point operator()(const Point& x) const {
				auto diff = x - goal; 
				auto diff2 = diff^2;
				return diff2;
			};

		private:
			const Point& goal;
	};

	class Dynamic_Constraint{
		public:
			Dynamic_Constraint(const int qd, const int vd, const int ud, const double dt, Dynamic_Func dynamics):
				qd(qd),vd(vd), ud(ud), dt(dt), dynamics(dynamics){
					pd = qd + vd + ud;
				}

			const Point operator(const Point& p0, const Point&p1){

			
			}
		private:
			const int qd;
			const int vd;
			const int ud;
			const double dt;
			int pd;
			Dynamic_Func dynamics;
	};
	
}
}

