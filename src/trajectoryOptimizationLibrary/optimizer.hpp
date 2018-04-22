#pragma once
#include "coin/IpTNLP.hpp"

namespace trajectoryOptimization::optimizer {

	using namespace Ipopt;

	struct OptimizerParameters {
		int numberVariablesX;
		int numberConstraintsG;
		int numberNonzeroJacobian;
		int numberNonzeroHessian;
	};

	struct BoundsData {
		Number* xLower;
		Number* xUpper;
		Number* gLower;
		Number* gUpper;
	};

	struct StartingPointData {
		const bool initX;
		const bool initZ;
		const bool initLambda;
		Number* x;
		Number* zLower;
		Number* zUpper;
		Number* lambda;
	};

	using BoundsFunction = bool(*)(Index n, Index m, BoundsData* bounds);
	using StartingPointFunction = bool(*)(Index n, Index m, StartingPointData* startingPointData);
	using ObjectiveFunction = bool(*)(Index n, const Number* x, Number& objValue);
	using GradientFunction = bool(*)(Index n, const Number* x, Number* gradF);
	using ConstraintFunction = bool(*)(Index n, const Number* x, Index m, Number* g);
	using JacobianStructureFunction = bool(*)(Index n, Index m, Index numberElementsJacobian, Index* iRow, Index *jCol);
	using JacobianValueFunction = bool(*)(Index n, const Number* x, Index m, Index numberElementsJacobian, Number* values);
	using HessianStructureFunction = bool(*)(Index n, Index m, Index numberElementsHessian, Index* iRow, Index* jCol);
	using HessianValueFunction = bool(*)(Index n, const Number* x, Number objFactor, Index m, const Number* lambda,
										Index numberElementsHessian, Number* values);
	using FinalizerFunction = void(*)(SolverReturn status, Index n, const Number* x, const Number* zLower, const Number* zUpper,
										Index m, const Number* g, const Number* lambda,Number objValue,
										const IpoptData* ipData, IpoptCalculatedQuantities* ipCalulatedQuantities);

	class TrajectoryOptimizer : public TNLP
	{
	public:
		TrajectoryOptimizer(OptimizerParameters* optimizerParameters,
							BoundsFunction boundsFunction,
							StartingPointFunction startingPointFunction,
							ObjectiveFunction objectiveFunction,
							GradientFunction gradientFunction,
							ConstraintFunction constraintFunction,
							JacobianStructureFunction jacobianStructureFunction,
							JacobianValueFunction jacobianValueFunction,
							HessianStructureFunction hessianStructureFunction,
							HessianValueFunction hessianValueFunction,
							FinalizerFunction finalizerFunction) :
			numberVariablesX(optimizerParameters->numberVariablesX),
			numberConstraintsG(optimizerParameters->numberConstraintsG),
			numberNonzeroJacobian(optimizerParameters->numberNonzeroJacobian),
			numberNonzeroHessian(optimizerParameters->numberNonzeroHessian),
			boundsFunction(boundsFunction),
			startingPointFunction(startingPointFunction),
			objectiveFunction(objectiveFunction),
			gradientFunction(gradientFunction),
			constraintFunction(constraintFunction),
			jacobianStructureFunction(jacobianStructureFunction),
			jacobianValueFunction(jacobianValueFunction),
			hessianStructureFunction(hessianStructureFunction),
			hessianValueFunction(hessianValueFunction),
			finalizerFunction(finalizerFunction) {}
		virtual ~TrajectoryOptimizer() {}

		virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
								  Index& nnz_h_lag, IndexStyleEnum& index_style) {
			n = numberVariablesX;
			m = numberConstraintsG;
			nnz_jac_g = numberNonzeroJacobian;
			nnz_h_lag = numberNonzeroHessian;
			index_style = TNLP::C_STYLE; // 0-based indexing
			return true;
		}

		virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
									 Index m, Number* g_l, Number* g_u) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);

			BoundsData boundsData = {.xLower=x_l, .xUpper=x_u, .gLower=g_l, .gUpper=g_u};

			return boundsFunction(n, m, &boundsData);
		}

		virtual bool get_starting_point(Index n, bool init_x, Number* x,
										bool init_z, Number* z_L, Number* z_U,
										Index m, bool init_lambda,
										Number* lambda) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);

			StartingPointData startingPointData = {.initX=init_x,
												  .initZ=init_z,
												  .initLambda=init_lambda,
												  .x=x,
												  .zLower=z_L,
												  .zUpper=z_U,
												  .lambda=lambda};

			return startingPointFunction(n, m, &startingPointData);
		}

		virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
			assert(n == numberVariablesX);
			return objectiveFunction(n, x, obj_value);
		}

		virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {
			assert(n == numberVariablesX);
			return gradientFunction(n, x, grad_f);
		}

		virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);
			return constraintFunction(n, x, m, g);
		}

		virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
								Index m, Index nele_jac, Index* iRow, Index *jCol,
								Number* values) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);
			assert(nele_jac == numberNonzeroJacobian);

			if (values == NULL) {
				return jacobianStructureFunction(n, m, nele_jac, iRow, jCol);
			}
			else {
				return jacobianValueFunction(n, x, m, nele_jac, values);
			}
		}

		virtual bool eval_h(Index n, const Number* x, bool new_x,
							Number obj_factor, Index m, const Number* lambda,
							bool new_lambda, Index nele_hess, Index* iRow,
							Index* jCol, Number* values) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);
			assert(nele_hess == numberNonzeroHessian);

			if (values == NULL) {
				return hessianStructureFunction(n, m, nele_hess, iRow, jCol);
			}
			else {
				return hessianValueFunction(n, x, obj_factor, m, lambda, nele_hess, values);
			}
		}

		virtual void finalize_solution(SolverReturn status,
									   Index n, const Number* x, const Number* z_L, const Number* z_U,
									   Index m, const Number* g, const Number* lambda,
									   Number obj_value,
									   const IpoptData* ip_data,
									   IpoptCalculatedQuantities* ip_cq) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);

			finalizerFunction(status, n, x, z_L, z_U, m, g, lambda, obj_value, ip_data, ip_cq);
		}

	private:
		/**@name Methods to block default compiler methods.
		 * The compiler automatically generates the following three methods.
		 *  Since the default compiler implementation is generally not what
		 *  you want (for all but the most simple classes), we usually 
		 *  put the declarations of these methods in the private section
		 *  and never implement them. This prevents the compiler from
		 *  implementing an incorrect "default" behavior without us
		 *  knowing. (See Scott Meyers book, "Effective C++")
		 *  
		 */
		//@{
		//  TrajectoryOptimizer();
		TrajectoryOptimizer(const TrajectoryOptimizer&);
		TrajectoryOptimizer& operator=(const TrajectoryOptimizer&);
		//@}

		const int numberVariablesX;
		const int numberConstraintsG;
		const int numberNonzeroJacobian;
		const int numberNonzeroHessian;

		const BoundsFunction boundsFunction;
		const StartingPointFunction startingPointFunction;
		const ObjectiveFunction objectiveFunction;
		const GradientFunction gradientFunction;
		const ConstraintFunction constraintFunction;
		const JacobianStructureFunction jacobianStructureFunction;
		const JacobianValueFunction jacobianValueFunction;
		const HessianStructureFunction hessianStructureFunction;
		const HessianValueFunction hessianValueFunction;
		const FinalizerFunction finalizerFunction;
  };
}