#pragma once
#include "coin/IpTNLP.hpp"

namespace trajectoryOptimization::optimizer {

  using namespace Ipopt;
  using namespace testing;

  class TrajectoryNLP : public TNLP
  {
  public:
    TrajectoryNLP() {}
    virtual ~TrajectoryNLP() {}

    /** Method to return some info about the nlp */
    virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                              Index& nnz_h_lag, IndexStyleEnum& index_style) {

      // the number of variables in the problem (dimension of x)
      n = 4;

      // the number of constraints in the problem (dimension of g(x)).
      m = 2;

      // the number of nonzero entries in the Jacobian
      nnz_jac_g = 8;

      // the number of nonzero entries in the Hessian
      nnz_h_lag = 10;

      // We use the standard c-style index style for row/col entries (0-based)
      index_style = C_STYLE;

      return true;
    }

    /** Method to return the bounds for our problem */
    virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                                 Index m, Number* g_l, Number* g_u) {
      // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
      // If desired, we could assert to make sure they are what we think they are.
      // assert(n == 2);
      // assert(m == 1);

      // // lower and upper bounds for x
      // x_l[0] = -1.0;
      // x_u[0] = 1.0;

      // // x2 has no upper or lower bound, so we set them to
      // // a large negative and a large positive number.
      // // The value that is interpreted as -/+infinity can be
      // // set in the options, but it defaults to -/+1e19
      // x_l[1] = -1.0e19;
      // x_u[1] = +1.0e19;

      // // lower and upper bounds for g(x)
      // g_l[0] = g_u[0] = 0.0;

      // return true;

      // the variables have lower bounds of 1
      for (Index i=0; i<4; i++)
        x_l[i] = 1.0;

      // the variables have upper bounds of 5
      for (Index i=0; i<4; i++)
        x_u[i] = 5.0;

      // the first constraint g1 has a lower bound of 25
      g_l[0] = 25;
      // the first constraint g1 has NO upper bound, here we set it to 2e19.
      // Ipopt interprets any number greater than nlp_upper_bound_inf as 
      // infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
      // is 1e19 and can be changed through ipopt options.
      g_u[0] = 2e19;

      // the second constraint g2 is an equality constraint, so we set the 
      // upper and lower bound to the same value
      g_l[1] = g_u[1] = 40.0;

      return true;
    }

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                    bool init_z, Number* z_L, Number* z_U,
                                    Index m, bool init_lambda,
                                    Number* lambda) {
      // // Here, we assume we only have starting values for x,  we can provide
      // // starting values for the others if we wish.
      // assert(init_x == true);
      // assert(init_z == false);
      // assert(init_lambda == false);

      // // we initialize x in bounds, in the upper right quadrant
      // x[0] = 0.5;
      // x[1] = 1.5;

      // return true;

      // Here, we assume we only have starting values for x, if you code
      // your own NLP, you can provide starting values for the dual variables
      // if you wish to use a warmstart option
      assert(init_x == true);
      assert(init_z == false);
      assert(init_lambda == false);

      // initialize to the given starting point
      x[0] = 1.0;
      x[1] = 5.0;
      x[2] = 5.0;
      x[3] = 1.0;

      return true;
    }

    /** Method to return the objective value */
    virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
      // // return value of objective function
      // obj_value = 0;

      assert(n == 4);

      obj_value = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];

      return true;
    }

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {
      // // return the gradient of the objective function grad_{x} f(x)

      // // grad_{x1} f(x): x1 is not in the objective
      // grad_f[0] = 0.0;
      // grad_f[1] = 0.0;

      // return true;

      assert(n == 4);

      grad_f[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
      grad_f[1] = x[0] * x[3];
      grad_f[2] = x[0] * x[3] + 1;
      grad_f[3] = x[0] * (x[0] + x[1] + x[2]);

      return true;
    }

    /** Method to return the constraint residuals */
    virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) {
      // // return the value of the constraints: g(x)
      // Number x1 = x[0];
      // Number x2 = x[1];

      // g[0] = 0;

      // return true;

      assert(n == 4);
      assert(m == 2);

      g[0] = x[0] * x[1] * x[2] * x[3];
      g[1] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];

      return true;
    }

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                            Index m, Index nele_jac, Index* iRow, Index *jCol,
                            Number* values) {
      if (values == NULL) {
        // return the structure of the Jacobian

        // this particular Jacobian is dense
        iRow[0] = 0; jCol[0] = 0;
        iRow[1] = 0; jCol[1] = 1;
        iRow[2] = 0; jCol[2] = 2;
        iRow[3] = 0; jCol[3] = 3;
        iRow[4] = 1; jCol[4] = 0;
        iRow[5] = 1; jCol[5] = 1;
        iRow[6] = 1; jCol[6] = 2;
        iRow[7] = 1; jCol[7] = 3;
      }
      else {
        // return the values of the Jacobian of the constraints
        
        values[0] = x[1]*x[2]*x[3]; // 0,0
        values[1] = x[0]*x[2]*x[3]; // 0,1
        values[2] = x[0]*x[1]*x[3]; // 0,2
        values[3] = x[0]*x[1]*x[2]; // 0,3

        values[4] = 2*x[0]; // 1,0
        values[5] = 2*x[1]; // 1,1
        values[6] = 2*x[2]; // 1,2
        values[7] = 2*x[3]; // 1,3
      }

      return true;
    }

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    virtual bool eval_h(Index n, const Number* x, bool new_x,
                        Number obj_factor, Index m, const Number* lambda,
                        bool new_lambda, Index nele_hess, Index* iRow,
                        Index* jCol, Number* values) {
      if (values == NULL) {
        // return the structure. This is a symmetric matrix, fill the lower left
        // triangle only.

        // the Hessian for this problem is actually dense
        Index idx=0;
        for (Index row = 0; row < 4; row++) {
          for (Index col = 0; col <= row; col++) {
            iRow[idx] = row; 
            jCol[idx] = col;
            idx++;
          }
        }
        
        assert(idx == nele_hess);
      }
      else {
        // return the values. This is a symmetric matrix, fill the lower left
        // triangle only

        // fill the objective portion
        values[0] = obj_factor * (2*x[3]); // 0,0

        values[1] = obj_factor * (x[3]);   // 1,0
        values[2] = 0;                     // 1,1

        values[3] = obj_factor * (x[3]);   // 2,0
        values[4] = 0;                     // 2,1
        values[5] = 0;                     // 2,2

        values[6] = obj_factor * (2*x[0] + x[1] + x[2]); // 3,0
        values[7] = obj_factor * (x[0]);                 // 3,1
        values[8] = obj_factor * (x[0]);                 // 3,2
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
      }

      return true;
    }

    //@}

    /** @name Solution Methods */
    //@{
    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(SolverReturn status,
                                   Index n, const Number* x, const Number* z_L, const Number* z_U,
                                   Index m, const Number* g, const Number* lambda,
                                   Number obj_value,
  				 const IpoptData* ip_data,
  				 IpoptCalculatedQuantities* ip_cq) {
      // here is where we would store the solution to variables, or write to a file, etc

      // For this example, we write the solution to the console
      printf("\n\nSolution of the primal variables, x\n");
      for (Index i=0; i<n; i++) {
        printf("x[%d] = %e\n", i, x[i]); 
      }

      printf("\n\nSolution of the bound multipliers, z_L and z_U\n");
      for (Index i=0; i<n; i++) {
        printf("z_L[%d] = %e\n", i, z_L[i]); 
      }
      for (Index i=0; i<n; i++) {
        printf("z_U[%d] = %e\n", i, z_U[i]); 
      }

      printf("\n\nObjective value\n");
      printf("f(x*) = %e\n", obj_value); 
    }
    //@}

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
    //  TrajectoryNLP();
    TrajectoryNLP(const TrajectoryNLP&);
    TrajectoryNLP& operator=(const TrajectoryNLP&);
    //@}
  };
}