#pragma once
#include <cassert>
#include <functional>
#include <algorithm>
#include <unordered_map>
#include <range/v3/view.hpp>

namespace trajectoryOptimization::derivative {
	// http://www.it.uom.gr/teaching/linearalgebra/NumericalRecipiesInC/c5-7.pdf
	const double FALLBACK_H_IF_X_ZERO = 1e-8;
	const double EPSILON = std::numeric_limits<double>::epsilon();
	const double SQRT_EPSILON = std::sqrt(EPSILON);

	using VectorToDoubleFunction = std::function<double(const double* x)>;
	using VectorToVectorFunction = std::function<std::vector<double>(const double* x)>;

	double calculateH(const double* x, const unsigned partialIndex) {
		return (x[partialIndex] != 0 ? SQRT_EPSILON * x[partialIndex] : FALLBACK_H_IF_X_ZERO);
	}

	double calculateDerivative(const double h, const double f2, const double f1) {
		return (f2 - f1)/(2*h);
	}

	class GetPartialDerivativeOfVectorToDoubleFunction {
		const VectorToDoubleFunction f;
		const unsigned numberVariables;

	public:
		GetPartialDerivativeOfVectorToDoubleFunction(const VectorToDoubleFunction f, const unsigned numberVariables):
			f(f), numberVariables(numberVariables) {}

		double operator()(const double* x, const unsigned partialIndex) const {
			assert(partialIndex < numberVariables);

			double x1[numberVariables];
			std::copy(x, x + numberVariables, x1);

			const double h = calculateH(x, partialIndex);

			x1[partialIndex] -= h;
			const double f1 = f(x1);

			x1[partialIndex] = x[partialIndex];

			x1[partialIndex] += h;
			const double f2 = f(x1);

			return calculateDerivative(h, f2, f1);
		}
	};

	class GetGradientOfVectorToDoubleFunction {
		const VectorToDoubleFunction f;
		const int numberVariables;
		const GetPartialDerivativeOfVectorToDoubleFunction getPartialDerivative;
		const std::vector<int> variableIndexRange;

	public:
		GetGradientOfVectorToDoubleFunction(const VectorToDoubleFunction f, const int numberVariables):
			f(f),
			numberVariables(numberVariables),
			getPartialDerivative(f, numberVariables),
			variableIndexRange(ranges::view::ints(0, numberVariables)) {}

		std::vector<double> operator()(const double* x) const {
			const auto getPartialDerivativeOfIndex = [&](const auto partialIndex) {
				return getPartialDerivative(x, partialIndex);
			};

			std::vector<double> gradient(numberVariables);
			std::transform(variableIndexRange.begin(), variableIndexRange.end(),
							gradient.begin(),
							getPartialDerivativeOfIndex);

			return gradient;
		}
	};

	class GetPartialDerivativeOfVectorToVectorFunction {
		const VectorToVectorFunction f;
		const unsigned numberVariablesInput;

	public:
		GetPartialDerivativeOfVectorToVectorFunction(const VectorToVectorFunction f, const unsigned numberVariablesInput):
			f(f), numberVariablesInput(numberVariablesInput) {}

		std::vector<double> operator()(const double* x, const unsigned partialIndex) const {
			assert(partialIndex < numberVariablesInput);

			double x1[numberVariablesInput];
			std::copy(x, x + numberVariablesInput, x1);

			const double h = calculateH(x, partialIndex);

			x1[partialIndex] -= h;
			const std::vector<double> f1 = f(x1);

			x1[partialIndex] = x[partialIndex];

			x1[partialIndex] += h;
			const std::vector<double> f2 = f(x1);

			const auto calculateDerivativeOfValues = [h](const double f1y, const double f2y) {
				return calculateDerivative(h, f2y, f1y);
			};

			const auto numOutputVariables = f1.size();
			std::vector<double> partialDerivative(numOutputVariables);

			std::transform(f1.begin(), f1.end(), f2.begin(),
							partialDerivative.begin(),
							calculateDerivativeOfValues);

			return partialDerivative;
		}
	};

	class GetJacobianColumnsOfVectorToVectorFunction {
		const VectorToVectorFunction f;
		const unsigned numberVariablesInput;
		const GetPartialDerivativeOfVectorToVectorFunction getPartialDerivative;
		const std::vector<int> jacobianColRange;

	public:
		GetJacobianColumnsOfVectorToVectorFunction(const VectorToVectorFunction f, const unsigned numberVariablesInput):
			f(f),
			numberVariablesInput(numberVariablesInput),
			getPartialDerivative(f, numberVariablesInput),
			jacobianColRange(ranges::view::ints((unsigned) 0, numberVariablesInput)) {}

		std::vector<std::vector<double>> operator()(const double* x) const {
			const auto numJacCols = jacobianColRange.size();
			std::vector<std::vector<double>> jacobianColumnList(numJacCols);
			std::transform(jacobianColRange.begin(),
							jacobianColRange.end(),
							jacobianColumnList.begin(),
							[&] (const int columnIndex) { return getPartialDerivative(x, columnIndex); });

			return jacobianColumnList;
		}
	};

	class GetSparsityPatternOfVectorToVectorFunction {
		const VectorToVectorFunction f;
		const unsigned numberVariablesInput;
		const std::vector<double> x;
		const GetJacobianColumnsOfVectorToVectorFunction getJacobianColumns;

	public:
		GetSparsityPatternOfVectorToVectorFunction(const VectorToVectorFunction f, const unsigned numberVariablesInput):
			f(f),
			numberVariablesInput(numberVariablesInput),
			x(numberVariablesInput, 1),
			getJacobianColumns(f, numberVariablesInput) {}

		std::tuple<std::vector<int>,std::vector<int>> operator()() const {
			const std::vector<std::vector<double>> jacobianColumnList = getJacobianColumns(x.data());
			const int numJacobianRows = jacobianColumnList[0].size();
			const int numJacobianCols = jacobianColumnList.size();

			std::vector<int> jacobianRows;
			std::vector<int> jacobianCols;
			for (int c : ranges::view::ints(0, numJacobianCols)) {
				for (int r : ranges::view::ints(0, numJacobianRows)) {
					if (jacobianColumnList[c][r] != 0) {
						jacobianRows.push_back(r);
						jacobianCols.push_back(c);
					}
				}
			}

			return {jacobianRows, jacobianCols};
		}
	};

	class GetJacobianOfVectorToVectorFunctionUsingSparsityPattern {
		const VectorToVectorFunction f;
		const GetPartialDerivativeOfVectorToVectorFunction getPartialDerivative;
		const std::vector<int> jacobianRows;
		const std::vector<int> jacobianCols;
		const int numJacobianValues;
		const std::vector<int> jacobianPositionIndexRange;

	public:
		GetJacobianOfVectorToVectorFunctionUsingSparsityPattern(const VectorToVectorFunction f,
																const unsigned numberVariablesInput,
																const std::vector<int> jacobianRows,
																const std::vector<int> jacobianCols):
			f(f),
			getPartialDerivative(f, numberVariablesInput),
			jacobianRows(jacobianRows),
			jacobianCols(jacobianCols),
			numJacobianValues(jacobianRows.size()),
			jacobianPositionIndexRange(ranges::view::ints(0, numJacobianValues)) {
				assert(jacobianRows.size() == jacobianCols.size());
			}

		std::vector<double> operator()(const double* x) const {
			std::unordered_map<int, std::vector<double>> jacobianColumnMap;

			const auto getJacobianValue = [&] (const auto row, const auto col) {
				if (jacobianColumnMap.count(col) == 0) {
					jacobianColumnMap.insert({col, getPartialDerivative(x, col)});
				}

				return jacobianColumnMap.at(col)[row];
			};

			std::vector<double> jacobian(numJacobianValues);
			std::transform(jacobianRows.begin(), jacobianRows.end(), jacobianCols.begin(),
							jacobian.begin(),
							getJacobianValue);

			return jacobian;
		}
	};

	std::tuple<const std::vector<int>,
				const std::vector<int>,
				const VectorToVectorFunction>
					getSparsityPatternAndJacobianFunctionOfVectorToVectorFunction(const VectorToVectorFunction f,
																		const unsigned numberVariablesInput) {
		const auto [jacobianRows, jacobianCols] = GetSparsityPatternOfVectorToVectorFunction(f, numberVariablesInput)();
		auto getJacobian = GetJacobianOfVectorToVectorFunctionUsingSparsityPattern(f,
																					numberVariablesInput,
																					jacobianRows,
																					jacobianCols);

		return {jacobianRows, jacobianCols, getJacobian};
	}
}