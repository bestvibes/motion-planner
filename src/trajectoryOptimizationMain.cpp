#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSolveStatistics.hpp"
#include <iostream>
#include <algorithm>
#include <functional>
#include <range/v3/view.hpp>

#include "trajectoryOptimization/constraint.hpp"
#include "trajectoryOptimization/cost.hpp"
#include "trajectoryOptimization/derivative.hpp"
#include "trajectoryOptimization/dynamic.hpp"
#include "trajectoryOptimization/optimizer.hpp"
#include "trajectoryOptimization/utilities.hpp"

using namespace Ipopt;
using namespace trajectoryOptimization::optimizer;
using namespace ranges;
using namespace trajectoryOptimization;

int main(int argv, char* argc[])
{
  const char* positionFilename = "position.txt";
  const char* velocityFilename = "velocity.txt";
  const char* controlFilename = "control.txt";

  const int worldDimension = 3;
  // pos, vel, acc (control)
  const int kinematicDimension = worldDimension * 2;
  const int controlDimension = worldDimension;
  const int timePointDimension = kinematicDimension + controlDimension;
  const int numTimePoints = 50;
  const int timeStepSize = 1;

  const dynamic::DynamicFunction blockDynamics = dynamic::BlockDynamics;

  const int numberVariablesX = timePointDimension * numTimePoints;

  const int startTimeIndex = 0;
  const numberVector startPoint = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  const int goalTimeIndex = numTimePoints - 1;
  const numberVector goalPoint = {50, 40, 30, 0, 0, 0, 0, 0, 0};

  const numberVector xLowerBounds(numberVariablesX, -100);
  const numberVector xUpperBounds(numberVariablesX, 100);

  const numberVector xStartingPoint(numberVariablesX, 0);

  const auto costFunction = cost::GetControlSquareSum(numTimePoints, timePointDimension, controlDimension);
  EvaluateObjectiveFunction objectiveFunction = [costFunction](Index n, const Number* x) {
    return costFunction(x);
  };

  const auto costGradientFunction = derivative::GetGradientOfVectorToDoubleFunction(costFunction, numberVariablesX);
  EvaluateGradientFunction gradientFunction = [costGradientFunction](Index n, const Number* x) {
    return costGradientFunction(x);
  };

  std::vector<constraint::ConstraintFunction> constraints;

  constraints.push_back(constraint::GetToKinematicGoalSquare(numTimePoints,
                                                              timePointDimension,
                                                              kinematicDimension,
                                                              startTimeIndex,
                                                              startPoint));

  const unsigned randomTargetTimeIndex = 25;
  const std::vector<double> randomTarget = {-10, 20, 30, 0, 0, 0, -10, 20, 30};
  constraints.push_back(constraint::GetToKinematicGoalSquare(numTimePoints,
                                                              timePointDimension,
                                                              kinematicDimension,
                                                              randomTargetTimeIndex,
                                                              randomTarget));

  const unsigned kinematicViolationConstraintStartIndex = 0;
  const unsigned kinematicViolationConstraintEndIndex = kinematicViolationConstraintStartIndex + numTimePoints - 1;
  constraints = constraint::applyKinematicViolationConstraints(constraints,
                                                                blockDynamics,
                                                                timePointDimension,
                                                                worldDimension,
                                                                kinematicViolationConstraintStartIndex,
                                                                kinematicViolationConstraintEndIndex,
                                                                timeStepSize);
                
  constraints.push_back(constraint::GetToKinematicGoalSquare(numTimePoints,
                                                                timePointDimension,
                                                                kinematicDimension,
                                                                goalTimeIndex,
                                                                goalPoint));

  const constraint::ConstraintFunction stackedConstraintFunction = constraint::StackConstriants(numberVariablesX, constraints);
  const unsigned numberConstraintsG = stackedConstraintFunction(xStartingPoint.data()).size();
  const numberVector gLowerBounds(numberConstraintsG);
  const numberVector gUpperBounds(numberConstraintsG);
  EvaluateConstraintFunction constraintFunction = [stackedConstraintFunction](Index n, const Number* x, Index m) {
    return stackedConstraintFunction(x);
  };

  indexVector jacStructureRows, jacStructureCols;
  constraint::ConstraintGradientFunction evaluateJacobianValueFunction;
  std::tie(jacStructureRows, jacStructureCols, evaluateJacobianValueFunction) =
      derivative::getSparsityPatternAndJacobianFunctionOfVectorToVectorFunction(stackedConstraintFunction, numberVariablesX);

  const int numberNonzeroJacobian = jacStructureRows.size();
  GetJacobianValueFunction jacobianValueFunction = [evaluateJacobianValueFunction](Index n, const Number* x, Index m,
                            Index numberElementsJacobian) {
    return evaluateJacobianValueFunction(x);
  };


  const int numberNonzeroHessian = 0;
  indexVector hessianStructureRows;
  indexVector hessianStructureCols;

  GetHessianValueFunction hessianValueFunction = [](Index n, const Number* x,
                          const Number objFactor, Index m, const Number* lambda,
                          Index numberElementsHessian) {
    numberVector values;
    return values;
  };

  FinalizerFunction finalizerFunction = [&](SolverReturn status, Index n, const Number* x,
                        const Number* zLower, const Number* zUpper,
                        Index m, const Number* g, const Number* lambda,
                        Number objValue, const IpoptData* ipData,
                        IpoptCalculatedQuantities* ipCalculatedQuantities) {
    printf("\n\nSolution of the primal variables, x\n");
    for (Index i=0; i<n; i++) {
      printf("x[%d] = %e\n", i, x[i]); 
    }

    utilities::outputPositionVelocityControlToFiles(x,
                                                    numTimePoints,
                                                    timePointDimension,
                                                    worldDimension,
                                                    positionFilename,
                                                    velocityFilename,
                                                    controlFilename);

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
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");

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

  utilities::plotTrajectory(worldDimension, positionFilename, velocityFilename, controlFilename);
}
