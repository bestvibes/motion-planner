// Copyright (C) 2005, 2009 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: block_main.cpp 2398 2013-10-19 18:08:59Z stefan $

#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSolveStatistics.hpp"
#include <iostream>
#include <algorithm>
#include <functional>
#include <range/v3/view.hpp>

#include "constraint.hpp"
#include "cost.hpp"
#include "dynamic.hpp"
#include "optimizer.hpp"
#include "utilities.hpp"

using namespace Ipopt;
using namespace trajectoryOptimization::optimizer;
using namespace ranges;
using namespace trajectoryOptimization;

int main(int argv, char* argc[])
{
  const int worldDimension = 1;
  // pos, vel, acc (control)
  const int kinematicDimension = worldDimension * 2;
  const int controlDimensions = worldDimension;
  const int timePointDimension = kinematicDimension + controlDimensions;
  const int numTimePoints = 4;
  const int timeStepSize = 1;

  const dynamic::DynamicFunction blockDynamics = dynamic::BlockDynamics;

  const int numberVariablesX = timePointDimension * numTimePoints;

  const int startTimeIndex = 0;
  const numberVector startPoint = {0, 0, 0}; //TODO: control ignored?
  const int goalTimeIndex = numTimePoints - 1;
  const numberVector goalPoint = {5, 0, 0};

  numberVector timePointLowerBounds = {-10, -10, -10};
  numberVector timePointUpperBounds = {10, 10, 10};

  const numberVector xLowerBounds = utilities::createTrajectoryWithIdenticalPoints(numTimePoints, timePointLowerBounds);
  const numberVector xUpperBounds = utilities::createTrajectoryWithIdenticalPoints(numTimePoints, timePointUpperBounds);

  numberVector xStartingPoint(numberVariablesX);

  const auto costFunction = cost::GetControlSquareSum(numTimePoints, timePointDimension, controlDimensions);
  EvaluateObjectiveFunction objectiveFunction = [costFunction](Index n, const Number* x) {
    return costFunction(x);
  };

  const auto costGradientFunction = cost::GetControlSquareSumGradient(numTimePoints, timePointDimension, controlDimensions);
  EvaluateGradientFunction gradientFunction = [costGradientFunction](Index n, const Number* x) {
    return costGradientFunction(x);
  };

  std::vector<constraint::ConstraintFunction> constraints;
  std::vector<constraint::ConstraintGradientFunction> constraintGradients;
  std::vector<constraint::ConstraintGradientIndicesFunction> constraintGradientIndices;

  std::tie(constraints, constraintGradients, constraintGradientIndices) =
                constraint::applyGetToKinematicGoalSquareConstraint(constraints,
                                                                    constraintGradients,
                                                                    constraintGradientIndices,
                                                                    numTimePoints,
                                                                    timePointDimension,
                                                                    kinematicDimension,
                                                                    startTimeIndex,
                                                                    startPoint);

  const unsigned kinematicViolationConstraintStartIndex = 0;
  const unsigned kinematicViolationConstraintEndIndex = kinematicViolationConstraintStartIndex + numTimePoints - 1;
  std::tie(constraints, constraintGradients, constraintGradientIndices) =
                constraint::applyKinematicViolationConstraints(constraints,
                                                                constraintGradients,
                                                                constraintGradientIndices,
                                                                blockDynamics,
                                                                timePointDimension,
                                                                worldDimension,
                                                                kinematicViolationConstraintStartIndex,
                                                                kinematicViolationConstraintEndIndex,
                                                                timeStepSize);

  std::tie(constraints, constraintGradients, constraintGradientIndices) =
                constraint::applyGetToKinematicGoalSquareConstraint(constraints,
                                                                    constraintGradients,
                                                                    constraintGradientIndices,
                                                                    numTimePoints,
                                                                    timePointDimension,
                                                                    kinematicDimension,
                                                                    goalTimeIndex,
                                                                    goalPoint);

  const constraint::ConstraintFunction stackedConstraints = constraint::StackConstriants(constraints);
  EvaluateConstraintFunction constraintFunction = [stackedConstraints](Index n, const Number* x, Index m) {
    return stackedConstraints(x);
  };

  const unsigned startConstraintIndex = 0;
  const constraint::ConstraintGradientIndicesFunction stackedConstraintGradientIndices = constraint::StackConstriantGradientIndices(constraintGradientIndices);
  auto const [numberConstraintsG, jacStructureRows, jacStructureCols] = stackedConstraintGradientIndices(startConstraintIndex);
  const numberVector gLowerBounds(numberConstraintsG);
  const numberVector gUpperBounds(numberConstraintsG);
  const int numberNonzeroJacobian = jacStructureRows.size();

  const constraint::ConstraintGradientFunction stackedConstraintGradients = constraint::StackConstriantGradients(constraintGradients);
  GetJacobianValueFunction jacobianValueFunction = [stackedConstraintGradients](Index n, const Number* x, Index m,
                            Index numberElementsJacobian) {
    return stackedConstraintGradients(x);
  };


  const int numberNonzeroHessian = 4;
  indexVector hessianStructureRows = {0, 1, 2, 3};
  indexVector hessianStructureCols = {2, 5, 8, 11};

  GetHessianValueFunction hessianValueFunction = [](Index n, const Number* x,
                          const Number objFactor, Index m, const Number* lambda,
                          Index numberElementsHessian) {

    //TODO: account for constraints and move this logic to appropriate places
    numberVector values(numberNonzeroHessian, objFactor * 2);

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
  } else {
    status = app->OptimizeTNLP(trajectoryOptimizer);

    Number final_obj;
    if (status == Solve_Succeeded) {
      Index iter_count = app->Statistics()->IterationCount();
      std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

      final_obj = app->Statistics()->FinalObjective();
      std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
    }
  }
}
