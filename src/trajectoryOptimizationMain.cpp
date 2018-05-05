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
  const int kinematicDimensions = worldDimension * 2;
  const int controlDimensions = worldDimension;
  const int timePointDimension = kinematicDimensions + controlDimensions;
  const int numTimePoints = 4;
  const int timeStepSize = 1;

  const dynamic::DynamicFunction blockDynamics = dynamic::BlockDynamics;

  const int numberVariablesX = timePointDimension * numTimePoints;
  const int numberNonzeroHessian = 0;

  const int startTimeIndex = 0;
  const numberVector startPoint = {0, 0, 0}; //TODO: control ignored?
  const int goalTimeIndex = numTimePoints - 1;
  const numberVector goalPoint = {5, 0, 0};

  numberVector timePointLowerBounds = {-5, -5, -5};
  numberVector timePointUpperBounds = {5, 5, 5};

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

  unsigned numberConstraints;
  std::vector<int> constraintGradientRows, constraintGradientCols;
  std::vector<constraint::ConstraintFunction> constraints;
  std::vector<constraint::ConstraintGradientFunction> constraintGradients;
  std::vector<int> jacStructureRows, jacStructureCols;
  unsigned currentConstraintIndex = 0;
  constraints.push_back(constraint::GetToKinematicGoalSquare(numTimePoints,
                                                              timePointDimension,
                                                              kinematicDimensions,
                                                              startTimeIndex,
                                                              startPoint));
  constraintGradients.push_back(constraint::GetToKinematicGoalSquareGradient(numTimePoints,
                                                              timePointDimension,
                                                              kinematicDimensions,
                                                              startTimeIndex,
                                                              startPoint));
  std::tie(numberConstraints, constraintGradientRows, constraintGradientCols) =
                    constraint::getToKinematicGoalSquareGradientIndices(currentConstraintIndex,
                                                                        timePointDimension,
                                                                        kinematicDimensions,
                                                                        startTimeIndex);
  jacStructureRows = view::concat(jacStructureRows, constraintGradientRows);
  jacStructureCols = view::concat(jacStructureCols, constraintGradientCols);
  currentConstraintIndex += numberConstraints;

  for (int timeIndex = 0; timeIndex < numTimePoints - 1; timeIndex++) {
    constraints.push_back(constraint::GetKinematicViolation(blockDynamics,
                                                            timePointDimension,
                                                            worldDimension,
                                                            timeIndex,
                                                            timeStepSize));
    constraintGradients.push_back(constraint::GetKinematicViolationGradient(blockDynamics,
                                                                            timePointDimension,
                                                                            worldDimension,
                                                                            timeIndex,
                                                                            timeStepSize));
    std::tie(numberConstraints, constraintGradientRows, constraintGradientCols) =
                    constraint::getKinematicViolationGradientIndices(currentConstraintIndex,
                                                                        timePointDimension,
                                                                        worldDimension,
                                                                        timeIndex);
    jacStructureRows = view::concat(jacStructureRows, constraintGradientRows);
    jacStructureCols = view::concat(jacStructureCols, constraintGradientCols);
    currentConstraintIndex += numberConstraints;
  }
  constraints.push_back(constraint::GetToKinematicGoalSquare(numTimePoints,
                                                              timePointDimension,
                                                              kinematicDimensions,
                                                              goalTimeIndex,
                                                              goalPoint));
  constraintGradients.push_back(constraint::GetToKinematicGoalSquareGradient(numTimePoints,
                                                              timePointDimension,
                                                              kinematicDimensions,
                                                              goalTimeIndex,
                                                              goalPoint));
  std::tie(numberConstraints, constraintGradientRows, constraintGradientCols) =
                    constraint::getToKinematicGoalSquareGradientIndices(currentConstraintIndex,
                                                                        timePointDimension,
                                                                        kinematicDimensions,
                                                                        goalTimeIndex);
  jacStructureRows = view::concat(jacStructureRows, constraintGradientRows);
  jacStructureCols = view::concat(jacStructureCols, constraintGradientCols);
  currentConstraintIndex += numberConstraints;

  const int numberConstraintsG = *std::max_element(jacStructureRows.begin(), jacStructureRows.end()) + 1;

  const numberVector gLowerBounds(numberConstraintsG);
  const numberVector gUpperBounds(numberConstraintsG);
  const int numberNonzeroJacobian = jacStructureRows.size();

  const constraint::ConstraintFunction stackedConstraints = constraint::StackConstriants(constraints);
  EvaluateConstraintFunction constraintFunction = [stackedConstraints](Index n, const Number* x, Index m) {
    return stackedConstraints(x);
  };

  const constraint::ConstraintGradientFunction stackedConstraintGradients = constraint::StackConstriantGradients(constraintGradients);
  GetJacobianValueFunction jacobianValueFunction = [stackedConstraintGradients](Index n, const Number* x, Index m,
                            Index numberElementsJacobian) {
    return stackedConstraintGradients(x);
  };

  indexVector hessianStructureRows(numberNonzeroHessian);
  indexVector hessianStructureCols(numberNonzeroHessian);

  // for (Index row = 0; row < 4; row++) {
  //   for (Index col = 0; col <= row; col++) {
  //     hessianStructureRows.push_back(row);
  //     hessianStructureCols.push_back(col);
  //   }
  // }

  GetHessianValueFunction hessianValueFunction = [](Index n, const Number* x,
                          Number objFactor, Index m, const Number* lambda,
                          Index numberElementsHessian) {
    
    numberVector values(numberNonzeroHessian);

    // values[0] = objFactor * (2*x[3]); // 0,0

    // values[1] = objFactor * (x[3]);   // 1,0
    // values[2] = 0;                     // 1,1

    // values[3] = objFactor * (x[3]);   // 2,0
    // values[4] = 0;                     // 2,1
    // values[5] = 0;                     // 2,2

    // values[6] = objFactor * (2*x[0] + x[1] + x[2]); // 3,0
    // values[7] = objFactor * (x[0]);                 // 3,1
    // values[8] = objFactor * (x[0]);                 // 3,2
    // values[9] = 0;                                   // 3,3


    // // add the portion for the first constraint
    // values[1] += lambda[0] * (x[2] * x[3]); // 1,0

    // values[3] += lambda[0] * (x[1] * x[3]); // 2,0
    // values[4] += lambda[0] * (x[0] * x[3]); // 2,1

    // values[6] += lambda[0] * (x[1] * x[2]); // 3,0
    // values[7] += lambda[0] * (x[0] * x[2]); // 3,1
    // values[8] += lambda[0] * (x[0] * x[1]); // 3,2

    // // add the portion for the second constraint
    // values[0] += lambda[1] * 2; // 0,0

    // values[2] += lambda[1] * 2; // 1,1

    // values[5] += lambda[1] * 2; // 2,2

    // values[9] += lambda[1] * 2; // 3,3

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
