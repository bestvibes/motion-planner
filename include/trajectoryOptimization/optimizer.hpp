#pragma once
#include "coin/IpTNLP.hpp"
#include <cassert>

namespace trajectoryOptimization::optimizer {

	using namespace Ipopt;

	using numberVector = std::vector<Number>;
	using indexVector = std::vector<Index>;

	using EvaluateObjectiveFunction = std::function<Number(Index n, const Number* x)>;
	using EvaluateGradientFunction = std::function<const numberVector(Index n, const Number* x)>;
	using EvaluateConstraintFunction = std::function<const numberVector(Index n, const Number* x, Index m)>;
	using GetJacobianValueFunction = std::function<const numberVector(Index n, const Number* x, Index m, Index numberElementsJacobians)>;
	using GetHessianValueFunction = std::function<const numberVector(Index n, const Number* x, const Number objFactor, Index m, const Number* lambda,
										Index numberElementsHessian)>;
	using FinalizerFunction = std::function<void(SolverReturn status, Index n, const Number* x, const Number* zLower, const Number* zUpper,
										Index m, const Number* g, const Number* lambda,Number objValue,
										const IpoptData* ipData, IpoptCalculatedQuantities* ipCalulatedQuantities)>;

	class TrajectoryOptimizer : public TNLP
	{
	public:
		TrajectoryOptimizer(const int numberVariablesX,
							const int numberConstraintsG,
							const int numberNonzeroJacobian,
							const int numberNonzeroHessian,
							const numberVector& xLowerBounds,
							const numberVector& xUpperBounds,
							const numberVector& gLowerBounds,
							const numberVector& gUpperBounds,
							const numberVector& xStartingPoint,
							// z and lambda starting point not implemented
							const EvaluateObjectiveFunction objectiveFunction,
							const EvaluateGradientFunction gradientFunction,
							const EvaluateConstraintFunction constraintFunction,
							const indexVector& jacobianStructureRows,
							const indexVector& jacobianStructureCols,
							const GetJacobianValueFunction jacobianValueFunction,
							const indexVector& hessianStructureRows,
							const indexVector& hessianStructureCols,
							const GetHessianValueFunction hessianValueFunction,
							const FinalizerFunction finalizerFunction) :
			numberVariablesX(numberVariablesX),
			numberConstraintsG(numberConstraintsG),
			numberNonzeroJacobian(numberNonzeroJacobian),
			numberNonzeroHessian(numberNonzeroHessian),
			xLowerBounds(xLowerBounds),
			xUpperBounds(xUpperBounds),
			gLowerBounds(gLowerBounds),
			gUpperBounds(gUpperBounds),
			xStartingPoint(xStartingPoint),
			objectiveFunction(objectiveFunction),
			gradientFunction(gradientFunction),
			constraintFunction(constraintFunction),
			jacobianStructureRows(jacobianStructureRows),
			jacobianStructureCols(jacobianStructureCols),
			jacobianValueFunction(jacobianValueFunction),
			hessianStructureRows(hessianStructureRows),
			hessianStructureCols(hessianStructureCols),
			hessianValueFunction(hessianValueFunction),
			finalizerFunction(finalizerFunction) {

				assert(numberVariablesX == xLowerBounds.size());
				assert(numberVariablesX == xUpperBounds.size());
				assert(numberConstraintsG == gLowerBounds.size());
				assert(numberConstraintsG == gUpperBounds.size());

				assert(numberVariablesX == xStartingPoint.size());

				assert(numberNonzeroJacobian == jacobianStructureRows.size());
				assert(numberNonzeroJacobian == jacobianStructureCols.size());

				assert(numberNonzeroHessian == hessianStructureRows.size());
				assert(numberNonzeroHessian == hessianStructureCols.size());
			}
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

			std::copy(xLowerBounds.begin(), xLowerBounds.end(), x_l);
			std::copy(xUpperBounds.begin(), xUpperBounds.end(), x_u);
			std::copy(gLowerBounds.begin(), gLowerBounds.end(), g_l);
			std::copy(gUpperBounds.begin(), gUpperBounds.end(), g_u);

			return true;
		}

		virtual bool get_starting_point(Index n, bool init_x, Number* x,
										bool init_z, Number* z_L, Number* z_U,
										Index m, bool init_lambda,
										Number* lambda) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);

			if (init_x) {
				std::copy(xStartingPoint.begin(), xStartingPoint.end(), x);
			}

			if (init_z) {
				printf("WARNING: Ipopt wants z starting point, not implemented");
			}

			if (init_lambda) {
				printf("WARNING: Ipopt wants lambda starting point, not implemented");
			}

			return true;
		}

		virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
			assert(n == numberVariablesX);
			obj_value = objectiveFunction(n, x);
			return true;
		}

		virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {
			assert(n == numberVariablesX);

			numberVector grad_output = gradientFunction(n, x);
			assert(n == grad_output.size());

			std::copy(grad_output.begin(), grad_output.end(), grad_f);
			return true;
		}

		virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);

			numberVector constraint_output = constraintFunction(n, x, m);
			assert(m == constraint_output.size());

			std::copy(constraint_output.begin(), constraint_output.end(), g);
			return true;
		}

		virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
								Index m, Index nele_jac, Index* iRow, Index *jCol,
								Number* values) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);
			assert(nele_jac == numberNonzeroJacobian);

			if (values == NULL) {
				std::copy(jacobianStructureRows.begin(), jacobianStructureRows.end(), iRow);
				std::copy(jacobianStructureCols.begin(), jacobianStructureCols.end(), jCol);
				return true;
			}
			else {
				numberVector vals = jacobianValueFunction(n, x, m, nele_jac);
				assert(nele_jac == vals.size());
				std::copy(vals.begin(), vals.end(), values);
				return true;
			}
		}

		virtual bool eval_h(Index n, const Number* x, bool new_x,
							Number obj_factor, Index m, const Number* lambda,
							bool new_lambda, Index nele_hess, Index* iRow,
							Index* jCol, Number* values) {
			assert(n == numberVariablesX);
			assert(m == numberConstraintsG);
			assert(nele_hess == numberNonzeroHessian);

			if (numberNonzeroHessian == 0) {
				return false;
			}

			if (values == NULL) {
				std::copy(hessianStructureRows.begin(), hessianStructureRows.end(), iRow);
				std::copy(hessianStructureCols.begin(), hessianStructureCols.end(), jCol);
				return true;
			}
			else {
				numberVector vals = hessianValueFunction(n, x, obj_factor, m, lambda, nele_hess);
				assert(nele_hess == vals.size());
				std::copy(vals.begin(), vals.end(), values);
				return true;
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

		const numberVector xLowerBounds;
		const numberVector xUpperBounds;
		const numberVector gLowerBounds;
		const numberVector gUpperBounds;

		const numberVector xStartingPoint;

		const EvaluateObjectiveFunction objectiveFunction;
		const EvaluateGradientFunction gradientFunction;
		const EvaluateConstraintFunction constraintFunction;

		const indexVector jacobianStructureRows;
		const indexVector jacobianStructureCols;
		const GetJacobianValueFunction jacobianValueFunction;
		
		const indexVector hessianStructureRows;
		const indexVector hessianStructureCols;
		const GetHessianValueFunction hessianValueFunction;

		const FinalizerFunction finalizerFunction;
  };
}