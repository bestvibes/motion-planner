//Tao Gao at UCLA
#pragma once

#include <functional>
#include "IpTNLP.hpp"
#include <array>
#include <vector>
#include <algorithm>  
#include <cassert>
#include <iostream>


namespace traj_opt{
using namespace Ipopt;

// template <unsigned int N>
using Eval_F_Func = std::function<double(const double*)>;

using Eval_Grad_F_Func = std::function<const std::vector<double> (const double*)>;;

using Eval_G_Func = std::function<const std::vector<double> (const double*)>; 

//cannot determine the size of the non-zero at compile time;  
using Eval_Jac_G_Func = std::function <const std::vector<double> (const double*)>; 

//N is the size of the traj, M is the number of the constriants
template <unsigned int N, unsigned int M >
class Traj_NLP : public TNLP
{
public:
  /** default constructor */
  Traj_NLP(const Index _nnzj,
					 const std::vector<Index>& _jRow,
					 const std::vector<Index>& _jCol,
					 const std::array<Number, N>& _xl,
					 const std::array<Number, N>& _xu,
					 const std::array<Number, N>& _x_init,
					 const std::array<Number, M>& _gl,
					 const std::array<Number, M>& _gu,
					 Eval_F_Func _eval_f_func,
					 Eval_Grad_F_Func _eval_grad_f_func,
					 Eval_G_Func _eval_g_func,
					 Eval_Jac_G_Func _eval_jac_g_func ):
				nnzj(_nnzj),
				jacRow(_jRow),
				jacCol(_jCol),
				xl(_xl),
				xu(_xu),
				x_init(_x_init),
				gl(_gl),
				gu(_gu),
				eval_f_func(_eval_f_func),
				eval_grad_f_func(_eval_grad_f_func),
				eval_g_func(_eval_g_func),
				eval_jac_g_func(_eval_jac_g_func)
	{
		assert(nnzj == jacRow.size());
		assert(nnzj == jacCol.size());
		std::cout << x_init.size() << std::endl;
		std::cout << eval_f_func(x_init.data()) << std::endl;
		std::cout << eval_grad_f_func(x_init.data()).size() << std::endl;
		std::cout << eval_g_func(x_init.data()).size() << std::endl;
		std::cout << eval_jac_g_func(x_init.data()).size() << std::endl;
	};

  /** default destructor */
  ~Traj_NLP(){};

  /**@name Overloaded from TNLP */
  //@{
  /** Method to return some info about the nlp */
   bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style) {
	
		// The problem described in Traj_NLP.hpp has 4 variables, x[0] through x[3]
		n = N;

		// one equality constraint and one inequality constraint
		m = M;

		// in this example the jacobian is dense and contains 8 nonzeros
		nnz_jac_g = nnzj;

		// the hessian is also dense and has 16 total nonzeros, but we
		// only need the lower left corner (since it is symmetric)
		// FIXME: this is a majic number here
		nnz_h_lag = 10;

		// use the C style indexing (0-based)
		index_style = TNLP::C_STYLE;


		return true;
	
	};

  /** Method to return the bounds for my problem */
  bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u){
		assert (n == N);
		assert (m == M);
		assert (n == xl.size());

		std::copy(xl.begin(), xl.end(), x_l);
		std::copy(xu.begin(), xu.end(), x_u);
		std::copy(gl.begin(), gl.end(), g_l);
		std::copy(gu.begin(), gu.end(), g_u);

		std::cout << "I reached the bound info" << std::endl;
		return true;
	}

  /** Method to return the starting point for the algorithm */
  bool get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda){
	
  // Here, we assume we only have starting values for x, if you code
  // your own NLP, you can provide starting values for the dual variables
  // if you wish
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

	std::copy(x_init.begin(), x_init.end(), x);
  return true;
	}

  /** Method to return the objective value */
  bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value){
		obj_value = eval_f_func(x);
		return true;
	}

  /** Method to return the gradient of the objective */
  bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f){
		auto result = eval_grad_f_func(x);
		std::copy(result.begin(), result.end(), grad_f);
		return true;
	}

  /** Method to return the constraint residuals */
  bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g){
		auto result = eval_g_func(x);
		std::copy(result.begin(), result.end(), g);
		return true;
	}

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
   bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values){

		if (values == NULL) {
			std::copy(jacRow.begin(), jacRow.end(), iRow);
			std::copy(jacCol.begin(), jacCol.end(), jCol);
		}
		else{
			auto result = eval_jac_g_func(x);
			std::copy(result.begin(), result.end(), values);
		}
		return true;
	 }

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  // virtual bool eval_h(Index n, const Number* x, bool new_x,
  //                     Number obj_factor, Index m, const Number* lambda,
  //                     bool new_lambda, Index nele_hess, Index* iRow,
  //                     Index* jCol, Number* values);

  //@}

  /** @name Solution Methods */
  //@{
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
  void finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
				 const IpoptData* ip_data,
				 IpoptCalculatedQuantities* ip_cq)
	{
	
  // here is where we would store the solution to variables, or write to a file, etc
  // so we could use the solution.

  // For this example, we write the solution to the console
  std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
  for (Index i=0; i<n; i++) {
     std::cout << "x[" << i << "] = " << x[i] << std::endl;
  }

  std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
  for (Index i=0; i<n; i++) {
    std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
  }
  for (Index i=0; i<n; i++) {
    std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
  }

  std::cout << std::endl << std::endl << "Objective value" << std::endl;
  std::cout << "f(x*) = " << obj_value << std::endl;

  std::cout << std::endl << "Final value of the constraints:" << std::endl;
  for (Index i=0; i<m ;i++) {
    std::cout << "g(" << i << ") = " << g[i] << std::endl;
  }
	
	};
  //@}

private:
  /**@name Methods to Block default compiler methods.
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
  //  Traj_NLP();
	// const std::string name;  
	// const int xdim; 
	// const int gdim;
  //@}
	const int nnzj;
	// const int nnzh;
	const std::vector<Index>& jacRow; 
	const std::vector<Index>& jacCol; 
	const std::array<double, N>& xl; 
	const std::array<double, N>& xu; 
	const std::array<double, N>& x_init;
	const std::array<double, M>& gl;
	const std::array<double, M>& gu;
	Eval_F_Func eval_f_func;
	Eval_Grad_F_Func eval_grad_f_func;
	Eval_G_Func eval_g_func;
	Eval_Jac_G_Func eval_jac_g_func;

  Traj_NLP(const Traj_NLP&);
  Traj_NLP& operator=(const Traj_NLP&);
};
}
