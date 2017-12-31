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
#include <functional>
#include <Eigen/Dense>

namespace traj_opt{
namespace constriant{
	using namespace Eigen;
	using Traj = Array<double, Dynamic, Dynamic, RowMajor>;
	using Point = Array<double, 1, Dynamic, RowMajor>;
	using Dynamic_Func = std::function<const Point(const Point&)>;
	using Point_Func = std::function<const Point(const Point&, const Point&)>;

	class Goal_Constraint{
		public:
			Goal_Constraint(const Point& goal): goal(goal){
			}

			const Point operator()(const Point& x) const {
				auto diff = x - goal; 
				auto diff2 = diff.pow(2);
				return diff2;
			};

		private:
			const Point& goal;
	};

	class Point_Dynamic_Constraint{
		public:
			Point_Dynamic_Constraint(const int qd, const int vd, const int ud, const double dt, Dynamic_Func dynamics):
				qd(qd),vd(vd), ud(ud), dt(dt), dynamics(dynamics){
					pd = qd + vd + ud;
				}

			const Point operator()(const Point& p0, const Point&p1){

				auto v0 = p0.block(0, qd, 1, vd);
				auto v1 = p1.block(0, qd, 1, vd);

				auto qv0 = p0.block(0, 0, 1, qd+vd);
				auto qv1 = p1.block(0, 0, 1, qd+vd);
				auto acc0 = dynamics(p0);
				auto acc1 = dynamics(p1);

				Point d0, d1;
				d0 << v0, acc0;
				d1 << v1, acc1; 

				auto error = (qv1 - qv0) - 0.5*dt*(d0 + d1);
				return error;
			};

		private:
			const int qd;
			const int vd;
			const int ud;
			const double dt;
			int pd;
			Dynamic_Func dynamics;
	};

	class Traj_Dynamic_Constriant{
		public:
			Traj_Dynamic_Constriant(const int n, const int pd, Point_Func point_func):
		n(n), pd(pd), point_func(point_func){};
			//traj is a 2d array, row_num = n; col_num = pd
			const Point operator () (const Traj& traj) {
				Point g;   
				for (int i=0; i<n-1;i++){
					Point p0 = traj.row(i);
					Point p1 = traj.row(i+1);
					Point error = point_func(p0, p1);
					g << error;
				}
				return g;
			};

		private:
			const int n;
			const int pd;
			Point_Func point_func;
	};
}
}

