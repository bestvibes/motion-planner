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

const int N =  4;
const int M = 2;

template<unsigned N>
using Array = std::array<double, N>;

const std::vector<int> iRow =  {{0, 0, 0, 0, 1, 1, 1, 1}};
const std::vector<int> jCol = {{0, 1, 2, 3, 0, 1, 2, 3}};
const int nnzj = 8;

double eval_f(const double* x){
	double f = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
	return f;
}

const Array<N> eval_grad_f(const double* x){
	Array<N> grad_f;
	grad_f[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
	grad_f[1] =  x[0] * x[3];
	grad_f[2] = x[0] * x[3] + 1;
	grad_f[3] = x[0] * (x[0] + x[1] + x[2]);
	return grad_f;
}

// template <unsigned N, unsigned M>
const Array<M> eval_g(const double* x){
	Array<M> g;
	g[0] = x[0] * x[1] * x[2] * x[3];
	g[1] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
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

