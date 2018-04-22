#include <gtest/gtest.h> 
#include <gmock/gmock.h>
#include <functional>
#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSolveStatistics.hpp"
#include "optimizer.hpp"

using namespace trajectoryOptimization::optimizer;
using namespace testing;
using namespace Ipopt;

TEST(optimizerTest, TestSampleCode) {

	OptimizerParameters optimizerParameters = {.numberVariablesX = 4,
												.numberConstraintsG = 2,
												.numberNonzeroJacobian = 8,
												.numberNonzeroHessian = 10};

	BoundsFunction boundsFunction = [](Index n, Index m, BoundsData* boundsData) {
		for (Index i=0; i<4; i++)
			boundsData->xLower[i] = 1.0;

		for (Index i=0; i<4; i++)
			boundsData->xUpper[i] = 5.0;

		boundsData->gLower[0] = 25;
		boundsData->gUpper[0] = 2e19; // == inf

		boundsData->gLower[1] = boundsData->gUpper[1] = 40.0;

		return true;
	};

	StartingPointFunction startingPointFunction = [](Index n, Index m, StartingPointData* startingPointData) {
		assert(startingPointData->initX == true);
		assert(startingPointData->initZ == false);
		assert(startingPointData->initLambda == false);

		startingPointData->x[0] = 1.0;
		startingPointData->x[1] = 5.0;
		startingPointData->x[2] = 5.0;
		startingPointData->x[3] = 1.0;

		return true;
	};

	ObjectiveFunction objectiveFunction = [](Index n, const Number* x, Number& objValue) {
		objValue = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
		return true;
	};

	GradientFunction gradientFunction = [](Index n, const Number* x, Number* gradF) {
		gradF[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
		gradF[1] = x[0] * x[3];
		gradF[2] = x[0] * x[3] + 1;
		gradF[3] = x[0] * (x[0] + x[1] + x[2]);
		return true;
	};

	ConstraintFunction constraintFunction = [](Index n, const Number* x, Index m, Number* g) {
		g[0] = x[0] * x[1] * x[2] * x[3];
		g[1] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
		return true;
	};

	JacobianStructureFunction jacobianStructureFunction = [](Index n, Index m, Index numberElementsJacobian,
																Index* iRow, Index *jCol) {
		iRow[0] = 0; jCol[0] = 0;
		iRow[1] = 0; jCol[1] = 1;
		iRow[2] = 0; jCol[2] = 2;
		iRow[3] = 0; jCol[3] = 3;
		iRow[4] = 1; jCol[4] = 0;
		iRow[5] = 1; jCol[5] = 1;
		iRow[6] = 1; jCol[6] = 2;
		iRow[7] = 1; jCol[7] = 3;

		return true;
	};

	JacobianValueFunction jacobianValueFunction = [](Index n, const Number* x, Index m,
														Index numberElementsJacobian, Number* values) {
		values[0] = x[1]*x[2]*x[3]; // 0,0
		values[1] = x[0]*x[2]*x[3]; // 0,1
		values[2] = x[0]*x[1]*x[3]; // 0,2
		values[3] = x[0]*x[1]*x[2]; // 0,3

		values[4] = 2*x[0]; // 1,0
		values[5] = 2*x[1]; // 1,1
		values[6] = 2*x[2]; // 1,2
		values[7] = 2*x[3]; // 1,3

		return true;
	};

	HessianStructureFunction hessianStructureFunction = [](Index n, Index m,
															Index numberElementsHessian,
															Index* iRow, Index* jCol) {
		Index idx=0;
		for (Index row = 0; row < 4; row++) {
			for (Index col = 0; col <= row; col++) {
				iRow[idx] = row; 
				jCol[idx] = col;
				idx++;
			}
		}

		return idx == numberElementsHessian;
	};

	HessianValueFunction hessianValueFunction = [](Index n, const Number* x,
													Number objFactor, Index m, const Number* lambda,
													Index numberElementsHessian, Number* values) {
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

		return true;
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

	SmartPtr<TNLP> trajectoryOptimizer = new TrajectoryOptimizer(&optimizerParameters,
												boundsFunction,
												startingPointFunction,
												objectiveFunction,
												gradientFunction,
												constraintFunction,
												jacobianStructureFunction,
												jacobianValueFunction,
												hessianStructureFunction,
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
	OptimizerParameters optimizerParameters = {.numberVariablesX = 2,
												.numberConstraintsG = 0,
												.numberNonzeroJacobian = 0,
												.numberNonzeroHessian = 0};

	BoundsFunction boundsFunction = [](Index n, Index m, BoundsData* boundsData) {
		for (Index i=0; i<2; i++)
			boundsData->xLower[i] = 0.0;

		for (Index i=0; i<2; i++)
			boundsData->xUpper[i] = 5.0;

		return true;
	};

	StartingPointFunction startingPointFunction = [](Index n, Index m, StartingPointData* startingPointData) {
		assert(startingPointData->initX == true);
		assert(startingPointData->initZ == false);
		assert(startingPointData->initLambda == false);

		startingPointData->x[0] = 0;
		startingPointData->x[1] = 0;

		return true;
	};

	ObjectiveFunction objectiveFunction = [](Index n, const Number* x, Number& objValue) {
		objValue = 0;
		return true;
	};

	GradientFunction gradientFunction = [](Index n, const Number* x, Number* gradF) {
		return true;
	};

	ConstraintFunction constraintFunction = [](Index n, const Number* x, Index m, Number* g) {
		return true;
	};

	JacobianStructureFunction jacobianStructureFunction = [](Index n, Index m, Index numberElementsJacobian,
																Index* iRow, Index *jCol) {
		return true;
	};

	JacobianValueFunction jacobianValueFunction = [](Index n, const Number* x, Index m,
														Index numberElementsJacobian, Number* values) {
		return true;
	};

	HessianStructureFunction hessianStructureFunction = [](Index n, Index m,
															Index numberElementsHessian,
															Index* iRow, Index* jCol) {
		return true;
	};

	HessianValueFunction hessianValueFunction = [](Index n, const Number* x,
													Number objFactor, Index m, const Number* lambda,
													Index numberElementsHessian, Number* values) {
		return true;
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

	SmartPtr<TNLP> trajectoryOptimizer = new TrajectoryOptimizer(&optimizerParameters,
												boundsFunction,
												startingPointFunction,
												objectiveFunction,
												gradientFunction,
												constraintFunction,
												jacobianStructureFunction,
												jacobianValueFunction,
												hessianStructureFunction,
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
	EXPECT_THAT(final_obj, 0);
}