#include <gtest/gtest.h> 
#include <gmock/gmock.h>
#include <range/v3/view.hpp>
#include <functional>
#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSolveStatistics.hpp"
#include "optimizer.hpp"

using namespace trajectoryOptimization::optimizer;
using namespace ranges;
using namespace Ipopt;

TEST(optimizerTest, TestSampleCode) {

	// Create an instance of our nlp...
	SmartPtr<TNLP> trajNLP = new TrajectoryNLP();

	// Create an instance of the IpoptApplication
	SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

	// Set options to optimize solving this example
	app->Options()->SetNumericValue("tol", 1e-9);
	app->Options()->SetStringValue("mu_strategy", "adaptive");

	// Initialize the IpoptApplication and process the options
	ApplicationReturnStatus status;
	status = app->Initialize();
	if (status != Solve_Succeeded) {
		std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
		FAIL();
	}

	// Solve problem
	status = app->OptimizeTNLP(trajNLP);

	if (status == Solve_Succeeded) {
		// Retrieve some statistics about the solve
		Index iter_count = app->Statistics()->IterationCount();
		std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

		Number final_obj = app->Statistics()->FinalObjective();
		std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
	}

	EXPECT_THAT(status, Solve_Succeeded);

}