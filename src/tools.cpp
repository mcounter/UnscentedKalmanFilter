#include <iostream>
#include <stdexcept>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	auto est_num = estimations.size();
	if (est_num != ground_truth.size())
	{
		throw std::runtime_error("Size of both vectors must be equal.");
		return rmse;
	}

	if (est_num <= 0)
	{
		return rmse;
	}

	for (int i = 0; i < est_num; ++i)
	{
		auto difference = (estimations[i] - ground_truth[i]).array();
		VectorXd residual = difference * difference;
		rmse += residual;
	}

	rmse /= est_num;

	rmse = rmse.array().sqrt();

	return rmse;
}
