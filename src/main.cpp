// Copyright (C) 2005, 2009 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: block_main.cpp 2398 2013-10-19 18:08:59Z stefan $

#include "IpIpoptApplication.hpp"
#include "traj_nlp.hpp"
#include <iostream>
#include <algorithm>
#include <functional>
#include "hs071_func.hpp"

using namespace Ipopt;

int main(int argv, char* argc[])
{
  // Create a new instance of your nlp
  //  (use a SmartPtr, not raw)
  // SmartPtr<TNLP> mynlp = new Traj_NLP("test");
	//N is the dimension of x
	const unsigned N = 4;
	const unsigned  M = 2;
	const int nnzj = hs071::nnzj;
	std::vector<int> iRow = hs071::iRow;
	std::vector<int> jCol = hs071::jCol;

	assert(nnzj == iRow.size());
	std::array<double, N> x_l;
	x_l.fill(1);
	// std::fill_n(x_l, N, 1);

	std::array<double, N> x_u;
	x_u.fill(5);
	// std::fill_n(x_u, N, 5);

	std::array<double, N> x_init = {{1.0, 5.0, 5.0, 1.0}};

	std::array<double, M> g_l, g_u;
	g_l[0] = 25; g_l[1] = 40; 
	g_u[0] = 2e19; g_u[1] = 40;

	auto eval_f = hs071::eval_f;
	auto eval_grad_f = hs071::eval_grad_f;
	auto eval_g = hs071::eval_g;
	auto eval_jac_g = hs071::eval_jac_g;

	SmartPtr<TNLP> mynlp =
		new traj_opt::Traj_NLP<N, M>(nnzj, iRow, jCol, x_l, x_u, x_init,
				g_l, g_u, eval_f, eval_grad_f, eval_g, eval_jac_g);

  // Create a new instance of IpoptApplication
  //  (use a SmartPtr, not raw)
  // We are using the factory, since this allows us to compile this
  // example with an Ipopt Windows DLL
  SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
  app->RethrowNonIpoptException(true);

  // Change some options
  // Note: The following choices are only examples, they might not be
  //       suitable for your optimization problem.
  app->Options()->SetNumericValue("tol", 1e-7);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");
	// turn this option on when no hessian approximation is available  
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  // The following overwrites the default name (ipopt.opt) of the
  // options file
  // app->Options()->SetStringValue("option_file_name", "hs071.opt");

  // Initialize the IpoptApplication and process the options
  ApplicationReturnStatus status;
  status = app->Initialize();
  if (status != Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    return (int) status;
  }

  // Ask Ipopt to solve the problem
  status = app->OptimizeTNLP(mynlp);

  if (status == Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
  }
  else {
    std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
  }
  // As the SmartPtrs go out of scope, the reference count
  // will be decremented and the objects will automatically
  // be deleted.

  return (int) status;
}
