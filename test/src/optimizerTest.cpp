#include <gtest/gtest.h> 
#include <gmock/gmock.h>
#include <functional>
#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSolveStatistics.hpp"
#include "trajectoryOptimization/optimizer.hpp"

using namespace trajectoryOptimization::optimizer;
using namespace testing;
using namespace Ipopt;

TEST(optimizerTest, TestSampleCode) {

	const int numberVariablesX = 4;
	const int numberConstraintsG = 2;
	const int numberNonzeroJacobian = 8;
	const int numberNonzeroHessian = 10;

	const numberVector xLowerBounds = {1.0, 1.0, 1.0, 1.0};
	const numberVector xUpperBounds = {5.0, 5.0, 5.0, 5.0};
	const numberVector gLowerBounds = {25, 40.0};
	const numberVector gUpperBounds = {2e19, 40.0};

	const numberVector xStartingPoint = {1.0, 5.0, 5.0, 1.0};

	EvaluateObjectiveFunction objectiveFunction = [](Index n, const Number* x) {
		return x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
	};

	EvaluateGradientFunction gradientFunction = [](Index n, const Number* x) {
		numberVector gradF;
		gradF.push_back(x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]));
		gradF.push_back(x[0] * x[3]);
		gradF.push_back(x[0] * x[3] + 1);
		gradF.push_back(x[0] * (x[0] + x[1] + x[2]));
		return gradF;
	};

	EvaluateConstraintFunction constraintFunction = [](Index n, const Number* x, Index m) {
		numberVector g;
		g.push_back(x[0] * x[1] * x[2] * x[3]);
		g.push_back(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
		return g;
	};

	indexVector jacStructureRows = {0, 0, 0, 0, 1, 1, 1, 1};
	indexVector jacStructureCols = {0, 1, 2, 3, 0, 1, 2 ,3};

	GetJacobianValueFunction jacobianValueFunction = [](Index n, const Number* x, Index m,
														Index numberElementsJacobian) {
		numberVector values = {
			x[1]*x[2]*x[3], // 0,0
			x[0]*x[2]*x[3], // 0,1
			x[0]*x[1]*x[3], // 0,2
			x[0]*x[1]*x[2], // 0,3
			2*x[0], // 1,0
			2*x[1], // 1,1
			2*x[2], // 1,2
			2*x[3], // 1,3
		};

		return values;
	};

	indexVector hessianStructureRows;
	indexVector hessianStructureCols;

	for (Index row = 0; row < 4; row++) {
		for (Index col = 0; col <= row; col++) {
			hessianStructureRows.push_back(row);
			hessianStructureCols.push_back(col);
		}
	}

	GetHessianValueFunction hessianValueFunction = [](Index n, const Number* x,
													Number objFactor, Index m, const Number* lambda,
													Index numberElementsHessian) {
		
		numberVector values(numberNonzeroHessian);

		values[0] = objFactor * (2*x[3]); // 0,0

		values[1] = objFactor * (x[3]);   // 1,0
		values[2] = 0;                     // 1,1

		values[3] = objFactor * (x[3]);   // 2,0
		values[4] = 0;                     // 2,1
		values[5] = 0;                     // 2,2

		values[6] = objFactor * (2*x[0] + x[1] + x[2]); // 3,0
		values[7] = objFactor * (x[0]);                 // 3,1
		values[8] = objFactor * (x[0]);                 // 3,2
		values[9] = 0;                                   // 3,3


		// add the portion for the first constraint
		values[1] += lambda[0] * (x[2] * x[3]); // 1,0

		values[3] += lambda[0] * (x[1] * x[3]); // 2,0
		values[4] += lambda[0] * (x[0] * x[3]); // 2,1

		values[6] += lambda[0] * (x[1] * x[2]); // 3,0
		values[7] += lambda[0] * (x[0] * x[2]); // 3,1
		values[8] += lambda[0] * (x[0] * x[1]); // 3,2

		// add the portion for the second constraint
		values[0] += lambda[1] * 2; // 0,0

		values[2] += lambda[1] * 2; // 1,1

		values[5] += lambda[1] * 2; // 2,2

		values[9] += lambda[1] * 2; // 3,3

		return values;
	};

	FinalizerFunction finalizerFunction = [](SolverReturn status, Index n, const Number* x,
												const Number* zLower, const Number* zUpper,
												Index m, const Number* g, const Number* lambda,
												Number objValue, const IpoptData* ipData,
												IpoptCalculatedQuantities* ipCalculatedQuantities) {
		printf("\n\nSolution of the primal variables, x\n");
		for (Index i=0; i<n; i++) {
			printf("x[%d] = %e\n", i, x[i]); 
		}

		printf("\n\nSolution of the bound multipliers, z_L and z_U\n");
		for (Index i=0; i<n; i++) {
			printf("z_L[%d] = %e\n", i, zLower[i]); 
		}
		for (Index i=0; i<n; i++) {
			printf("z_U[%d] = %e\n", i, zUpper[i]); 
		}

		printf("\n\nObjective value\n");
		printf("f(x*) = %e\n", objValue); 
	};

	SmartPtr<TNLP> trajectoryOptimizer = new TrajectoryOptimizer(numberVariablesX,
												numberConstraintsG,
												numberNonzeroJacobian,
												numberNonzeroHessian,
												xLowerBounds,
												xUpperBounds,
												gLowerBounds,
												gUpperBounds,
												xStartingPoint,
												objectiveFunction,
												gradientFunction,
												constraintFunction,
												jacStructureRows,
												jacStructureCols,
												jacobianValueFunction,
												hessianStructureRows,
												hessianStructureCols,
												hessianValueFunction,
												finalizerFunction);

	SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

	app->Options()->SetNumericValue("tol", 1e-9);
	app->Options()->SetStringValue("mu_strategy", "adaptive");

	ApplicationReturnStatus status;
	status = app->Initialize();
	if (status != Solve_Succeeded) {
		std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
		FAIL();
	}

	status = app->OptimizeTNLP(trajectoryOptimizer);

	Number final_obj;
	if (status == Solve_Succeeded) {
		Index iter_count = app->Statistics()->IterationCount();
		std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

		final_obj = app->Statistics()->FinalObjective();
		std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
	}

	EXPECT_THAT(status, Solve_Succeeded);
	EXPECT_THAT(final_obj, Ge(17.013));
	EXPECT_THAT(final_obj, Lt(17.015));
}

TEST(optimizerTest, SolutionForZeroes) {
	const int numberVariablesX = 2;
	const int numberConstraintsG = 0;
	const int numberNonzeroJacobian = 0;
	const int numberNonzeroHessian = 0;

	const numberVector xLowerBounds = {0, 0};
	const numberVector xUpperBounds = {5, 5};
	const numberVector gBounds;

	const numberVector xStartingPoint = {0, 0};

	EvaluateObjectiveFunction objectiveFunction = [](Index n, const Number* x) {
		return 0;
	};

	EvaluateGradientFunction gradientFunction = [](Index n, const Number* x) {
		numberVector v = {0, 0};
		return v;
	};

	EvaluateConstraintFunction constraintFunction = [](Index n, const Number* x, Index m) {
		numberVector v;
		return v;
	};

	indexVector jacHessStructure;

	GetJacobianValueFunction jacobianValueFunction = [](Index n, const Number* x, Index m,
														Index numberElementsJacobian) {
		numberVector v;
		return v;
	};

	GetHessianValueFunction hessianValueFunction = [](Index n, const Number* x,
													Number objFactor, Index m, const Number* lambda,
													Index numberElementsHessian) {
		numberVector v;
		return v;
	};

	FinalizerFunction finalizerFunction = [](SolverReturn status, Index n, const Number* x,
												const Number* zLower, const Number* zUpper,
												Index m, const Number* g, const Number* lambda,
												Number objValue, const IpoptData* ipData,
												IpoptCalculatedQuantities* ipCalculatedQuantities) {
		printf("\n\nSolution of the primal variables, x\n");
		for (Index i=0; i<n; i++) {
			printf("x[%d] = %e\n", i, x[i]); 
		}

		printf("\n\nSolution of the bound multipliers, z_L and z_U\n");
		for (Index i=0; i<n; i++) {
			printf("z_L[%d] = %e\n", i, zLower[i]); 
		}
		for (Index i=0; i<n; i++) {
			printf("z_U[%d] = %e\n", i, zUpper[i]); 
		}

		printf("\n\nObjective value\n");
		printf("f(x*) = %e\n", objValue); 
	};

	SmartPtr<TNLP> trajectoryOptimizer = new TrajectoryOptimizer(numberVariablesX,
												numberConstraintsG,
												numberNonzeroJacobian,
												numberNonzeroHessian,
												xLowerBounds,
												xUpperBounds,
												gBounds,
												gBounds,
												xStartingPoint,
												objectiveFunction,
												gradientFunction,
												constraintFunction,
												jacHessStructure,
												jacHessStructure,
												jacobianValueFunction,
												jacHessStructure,
												jacHessStructure,
												hessianValueFunction,
												finalizerFunction);

	SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

	app->Options()->SetNumericValue("tol", 1e-9);
	app->Options()->SetStringValue("mu_strategy", "adaptive");
	app->Options()->SetStringValue("hessian_approximation", "limited-memory");

	ApplicationReturnStatus status;
	status = app->Initialize();
	if (status != Solve_Succeeded) {
		std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
		FAIL();
	}

	status = app->OptimizeTNLP(trajectoryOptimizer);

	Number final_obj;
	if (status == Solve_Succeeded) {
		Index iter_count = app->Statistics()->IterationCount();
		std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

		final_obj = app->Statistics()->FinalObjective();
		std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
	}

	EXPECT_THAT(status, Solve_Succeeded);
	EXPECT_THAT(final_obj, 0);
}