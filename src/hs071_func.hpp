/*
 * =====================================================================================
 *
 *       Filename:  hs071_func.hpp
 *
 *    Description:  nlp functions associated with hs071
 *
 *        Version:  1.0
 *        Created:  12/23/2017 07:59:07 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tao Gao 
 *   Organization:  ucla
 *
 * =====================================================================================
 */
#pragma once
#include <stdio.h> 
#include <vector>
#include <array>

namespace hs071{

const std::vector<int> iRow =  {{0, 0, 0, 0, 1, 1, 1, 1}};
const std::vector<int> jCol = {{0, 1, 2, 3, 0, 1, 2, 3}};
const int nnzj = 8;

double eval_f(const double* x){
	double f = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
	return f;
}

const std::vector<double> eval_grad_f(const double* x){
	std::vector<double> grad_f;
  grad_f.push_back(x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]));
	grad_f.push_back(x[0] * x[3]);
	grad_f.push_back(x[0] * x[3] + 1);
	grad_f.push_back(x[0] * (x[0] + x[1] + x[2]));
	return grad_f;
}

const std::vector<double> eval_g(const double* x){
	std::vector<double> g;
	g.push_back(x[0] * x[1] * x[2] * x[3]);
	g.push_back(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
	return g;
}

const std::vector<double> eval_jac_g(const double* x){
	std::vector<double> jac;
	jac.push_back(x[1]*x[2]*x[3]);
	jac.push_back(x[0]*x[2]*x[3]);
  jac.push_back(x[0]*x[1]*x[3]); // 0,2
  jac.push_back(x[0]*x[1]*x[2]); // 0,3
	jac.push_back(2*x[0]);
	jac.push_back(2*x[1]);
	jac.push_back(2*x[2]);
	jac.push_back(2*x[3]);
	return jac;
}

}//hs071

