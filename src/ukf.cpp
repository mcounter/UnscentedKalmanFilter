#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Definitions

#define NIS_FACTOR_UNDEFINED (-100)

// Process noise standard deviation longitudinal acceleration in m/s^2
const double std_a_ = 0.5;

// Process noise standard deviation yaw acceleration in rad/s^2
const double std_yawdd_ = 0.5;

// Help functions

void normalizeFi(double &fi)
{
	if ((fi > M_PI) || (fi < -M_PI))
	{
		while (fi > M_PI) { fi -= M_PI + M_PI; }
		while (fi < -M_PI) { fi += M_PI + M_PI; }
	}
}

// UKFProcess
UKFProcess::UKFProcess(const int &n_x, const int &n_noise, const double &lambda)
{
	is_initialized_ = false;

	n_x_ = n_x;
	n_noise_ = n_noise;
	lambda_ = lambda;

	initWeightsVector();
	m_weights_ = weights_.asDiagonal();
}

UKFProcess::~UKFProcess() {}

void UKFProcess::initWeightsVector()
{
	int weights_size = 2 * (n_x_ + n_noise_) + 1;
	double lambda_aug = lambda_ + n_x_ + n_noise_;

	weights_ = VectorXd(weights_size);
	weights_(0) = lambda_ / lambda_aug;
	weights_.tail(weights_size - 1).setConstant(0.5 / lambda_aug);
}

void UKFProcess::normalizeMatrix(MatrixXd &m) {}

MatrixXd UKFProcess::createSigmaPoints()
{
	int n_aug = n_x_ + n_noise_;

	VectorXd x_aug = VectorXd(n_aug);
	x_aug.head(n_x_) = x_;
	x_aug.tail(n_noise_).setZero();

	VectorXd noise_vector = noise_.array().pow(2);

	MatrixXd P_aug = MatrixXd(n_aug, n_aug);
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug.block(0, n_x_, n_x_, n_noise_).setZero();
	P_aug.block(n_x_, 0, n_noise_, n_x_).setZero();
	P_aug.bottomRightCorner(n_noise_, n_noise_) = noise_vector.asDiagonal();

	MatrixXd Psqrt = P_aug.llt().matrixL();
	Psqrt = Psqrt * sqrt(lambda_ + n_aug);

	MatrixXd x_sigma_points = MatrixXd(n_aug, 2 * n_aug + 1);
	x_sigma_points.col(0) = x_aug;
	x_sigma_points.block(0, 1, n_aug, n_aug) = Psqrt.colwise() + x_aug;
	x_sigma_points.block(0, 1 + n_aug, n_aug, n_aug) = (-Psqrt).colwise() + x_aug;

	return x_sigma_points;
}

MatrixXd UKFProcess::predictSigmaPoints(MatrixXd &sigma_points, const float &dt)
{
	MatrixXd x_sigma_predicted = MatrixXd(n_x_, 2 * (n_x_ + n_noise_) + 1);
	x_sigma_predicted.setZero();

	return x_sigma_predicted;
}

void UKFProcess::UpdateState(VectorXd &x, MatrixXd &P)
{
	x_ = x;
	P_ = P;

	is_initialized_ = true;
}

void UKFProcess::Predict(const float &dt)
{
	if (!is_initialized_)
	{
		return;
	}

	MatrixXd sigma_points = createSigmaPoints();
	MatrixXd new_x_sig_pred = predictSigmaPoints(sigma_points, dt);

	//predict state mean
	VectorXd newX = new_x_sig_pred * weights_;

	//predict state covariance matrix
	MatrixXd new_x_sig_res = new_x_sig_pred.colwise() - newX;
	normalizeMatrix(new_x_sig_res);

	//Predict state
	MatrixXd newP = new_x_sig_res * m_weights_ * new_x_sig_res.transpose();

	// If not any exceptions happened, update internal variables
	Xsig_pred_ = new_x_sig_pred;
	Xsig_res_ = new_x_sig_res;
	x_ = newX;
	P_ = newP;
}

// UKFMeasurement
UKFMeasurement::UKFMeasurement(UKFProcess &process, const int &n_z)
{
	process_ = &process;
	n_z_ = n_z;
	nis_ = 0;
	nis_factor_ = NIS_FACTOR_UNDEFINED;
}

UKFMeasurement::~UKFMeasurement() {}

void UKFMeasurement::initFromMeasurement(const VectorXd &z, VectorXd &x, MatrixXd &P) {}

void UKFMeasurement::normalizeMeasurement(VectorXd &z) {}

void UKFMeasurement::normalizeMatrix(MatrixXd &m) {}

MatrixXd UKFMeasurement::transformSigmaPoints(MatrixXd &sigma_points)
{
	MatrixXd z_sig = MatrixXd(n_z_, sigma_points.cols());
	z_sig.setZero();

	return z_sig;
}

void UKFMeasurement::analyzeNIS(double &nis)
{
	double xi095 = 0;
	double xi005 = 0;

	switch (n_z_)
	{
		case 1:
			xi095 = 0.004;
			xi005 = 3.841;
			break;

		case 2:
			xi095 = 0.103;
			xi005 = 5.991;
			break;

		case 3:
			xi095 = 0.352;
			xi005 = 7.815;
			break;

		case 4:
			xi095 = 0.711;
			xi005 = 9.488;
			break;

		default:
			xi095 = 1.145;
			xi005 = 11.070;
			break;
	}

	nis_ = nis;
	if (nis > xi005)
	{
		nis_factor_ = +1;
	}
	else
	if (nis < xi095)
	{
		nis_factor_ = -1;
	}
	else
	{
		nis_factor_ = 0;
	}
}

void UKFMeasurement::Predict(const float &dt)
{
	process_->Predict(dt);
}

void UKFMeasurement::Update(const VectorXd &z)
{
	VectorXd normZ = VectorXd(z);
	normalizeMeasurement(normZ);

	if (!process_->is_initialized_)
	{
		VectorXd newX;
		MatrixXd newP;

		initFromMeasurement(normZ, newX, newP);
		process_->UpdateState(newX, newP);
	}
	else
	{
		MatrixXd z_sig = transformSigmaPoints(process_->Xsig_pred_);
		VectorXd z_pred = z_sig * process_->weights_;
		
		MatrixXd z_sig_res = z_sig.colwise() - z_pred;
		normalizeMatrix(z_sig_res);
		MatrixXd z_sig_res_t = z_sig_res.transpose();

		MatrixXd S = z_sig_res * process_->m_weights_ * z_sig_res_t;

		VectorXd noise_vector = noise_.array().pow(2);
		S += noise_vector.asDiagonal();
		MatrixXd S1 = S.inverse();

		MatrixXd Tc = process_->Xsig_res_ * process_->m_weights_ * z_sig_res_t;
		MatrixXd K = Tc * S1;

		VectorXd y = normZ - z_pred;
		normalizeMeasurement(y);

		VectorXd newX = process_->x_ + K * y;
		MatrixXd newP = process_->P_ - K * S * K.transpose();
		
		double nis = y.transpose() * S1 * y;
		analyzeNIS(nis);

		// If not any exceptions happened, update internal variables
		process_->UpdateState(newX, newP);
	}
}

// UKFMeasurementRadar
UKFMeasurementRadar::UKFMeasurementRadar(UKFProcess &process) : UKFMeasurement(process, 3)
{
	// Radar measurement noise standard deviation radius in m
	const double std_radr = 0.3;

	// Radar measurement noise standard deviation angle in rad
	const double std_radphi = 0.03;

	// Radar measurement noise standard deviation radius change in m/s
	const double std_radrd = 0.3;

	noise_ = VectorXd(n_z_);
	noise_ << std_radr, std_radphi, std_radrd;
}

UKFMeasurementRadar::~UKFMeasurementRadar() {}

void UKFMeasurementRadar::normalizeMeasurement(VectorXd &z) 
{
	normalizeFi(z(1));
}

void UKFMeasurementRadar::normalizeMatrix(MatrixXd &m) 
{
	auto m_size = m.cols();
	for (int i = 0; i < m_size; ++i)
	{
		normalizeFi(m(1, i));
	}
}

MatrixXd UKFMeasurementRadar::transformSigmaPoints(MatrixXd &sigma_points)
{
	auto sig_size = sigma_points.cols();
	MatrixXd z_sig = MatrixXd(n_z_, sig_size);

	//transform sigma points into measurement space
	for (int i = 0; i < sig_size; ++i)
	{
		double& px = sigma_points(0, i);
		double& py = sigma_points(1, i);
		double& v = sigma_points(2, i);
		double& fi = sigma_points(3, i);
		double& dfi = sigma_points(4, i);

		double pxy11 = px * px + py * py;
		double pxy12 = sqrt(pxy11);

		if (pxy12 < min_distance_)
		{
			throw std::runtime_error("RADAR measurement too close to point of origin.");
		}

		z_sig(0, i) = pxy12;
		z_sig(1, i) = atan2(py, px);
		z_sig(2, i) = (px * v * cos(fi) + py * v * sin(fi)) / pxy12;
	}

	return z_sig;
}

void UKFMeasurementRadar::initFromMeasurement(const VectorXd &z, VectorXd &x, MatrixXd &P)
{
	auto& rho = z(0);
	auto& theta = z(1);
	auto& drho = z(2);

	x = VectorXd(5);
	x << rho * cos(theta), rho * sin(theta), drho, theta, 0;

	VectorXd p = VectorXd(5);
	p << noise_(0) * 3.0, noise_(0) * 3.0, 5, M_PI / 2.0, 0.1;

	p = p.array().pow(2);
	P = p.asDiagonal();
}

// UKFMeasurementLidar
UKFMeasurementLidar::UKFMeasurementLidar(UKFProcess &process) : UKFMeasurement(process, 2)
{
	// Laser measurement noise standard deviation position1 in m
	const double std_laspx = 0.15;
	
	// Laser measurement noise standard deviation position2 in m
	const double std_laspy = 0.15;

	noise_ = VectorXd(n_z_);
	noise_ << std_laspx, std_laspy;
}

UKFMeasurementLidar::~UKFMeasurementLidar() {}

MatrixXd UKFMeasurementLidar::transformSigmaPoints(MatrixXd &sigma_points)
{
	auto sig_size = sigma_points.cols();
	MatrixXd z_sig = MatrixXd(n_z_, sig_size);

	//transform sigma points into measurement space
	for (int i = 0; i < sig_size; ++i)
	{
		double& px = sigma_points(0, i);
		double& py = sigma_points(1, i);

		z_sig(0, i) = px;
		z_sig(1, i) = py;
	}

	return z_sig;
}

void UKFMeasurementLidar::initFromMeasurement(const VectorXd &z, VectorXd &x, MatrixXd &P)
{
	auto& px = z(0);
	auto& py = z(1);

	x = VectorXd(5);
	x << px, py, 0, 0, 0;

	VectorXd p = VectorXd(5);
	p << noise_(0) * 3.0, noise_(0) * 3.0, 5, M_PI / 2.0, 0.1;

	p = p.array().pow(2);
	P = p.asDiagonal();
}

// UKF
UKF::UKF() : UKFProcess(5, 2, 3 - 5 - 2)
{
	time_us_ = 0;
	nis_total_ = 0;
	nis_consistent_ = 0;
	nis_consistency_ = 0;

	noise_ = VectorXd(n_noise_);
	noise_ << std_a_, std_yawdd_;

	for (int i = 0; i < SensorTypeSize; i++)
	{
		switch (i)
		{
			case MeasurementPackage::LASER:
				measurements_[i] = new UKFMeasurementLidar(*this);
				break;

			case MeasurementPackage::RADAR:
				measurements_[i] = new UKFMeasurementRadar(*this);
				break;

			default:
				measurements_[i] = 0;
				break;
		}
	}
}

UKF::~UKF()
{
	for (int i = 0; i < SensorTypeSize; i++)
	{
		if (measurements_[i])
		{
			delete measurements_[i];
		}
	}
}

void UKF::normalizeMatrix(MatrixXd &m)
{
	auto m_size = m.cols();
	for (int i = 0; i < m_size; ++i)
	{
		normalizeFi(m(3, i));
	}
}

MatrixXd UKF::predictSigmaPoints(MatrixXd &sigma_points, const float &dt)
{
	int m_size = 2 * (n_x_ + n_noise_) + 1;
	MatrixXd x_sig_pred = MatrixXd(n_x_, m_size);

	double dt2 = dt * dt;
	for (int i = 0; i < m_size; ++i)
	{
		double& px = sigma_points(0, i);
		double& py = sigma_points(1, i);
		double& v = sigma_points(2, i);
		double& fi = sigma_points(3, i);
		double& dfi = sigma_points(4, i);
		double& va = sigma_points(5, i);
		double& vfi = sigma_points(6, i);

		double cosfi = cos(fi);
		double sinfi = sin(fi);

		if (fabs(dfi) < 0.0001)
		{
			x_sig_pred(0, i) = px + v * cosfi * dt + 0.5 * va * dt2 * cosfi;
			x_sig_pred(1, i) = py + v * sinfi * dt + 0.5 * va * dt2 * sinfi;
		}
		else
		{
			x_sig_pred(0, i) = px + v / dfi * (sin(fi + dfi * dt) - sinfi) + 0.5 * va * dt2 * cosfi;
			x_sig_pred(1, i) = py + v / dfi * (cosfi - cos(fi + dfi * dt)) + 0.5 * va * dt2 * sinfi;
		}

		x_sig_pred(2, i) = v + va * dt;
		x_sig_pred(3, i) = fi + dfi * dt + 0.5 * vfi * dt2;
		x_sig_pred(4, i) = dfi + vfi * dt;
	}

	return x_sig_pred;
}

void UKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
	try
	{
		if (!measurements_[measurement_pack.sensor_type_])
		{
			throw std::runtime_error("Unsupported sensor type.");
		}

		if (!is_initialized_ || use_sensor_[measurement_pack.sensor_type_])
		{
			if (time_us_ > 0)
			{
				float dt = (measurement_pack.timestamp_ - time_us_) / 1000000.0;
				measurements_[measurement_pack.sensor_type_]->Predict(dt);
			}

			time_us_ = measurement_pack.timestamp_;

			measurements_[measurement_pack.sensor_type_]->Update(measurement_pack.raw_measurements_);

			// print the output
			cout << "UKF: " << endl;
			cout << "x_ = " << x_ << endl;
			cout << "P_ = " << P_ << endl;

			if (measurements_[measurement_pack.sensor_type_]->nis_factor_ != NIS_FACTOR_UNDEFINED)
			{
				nis_total_++;
				if (measurements_[measurement_pack.sensor_type_]->nis_factor_ == 0)
				{
					nis_consistent_++;
				}

				nis_consistency_ = (double)nis_consistent_ / (double)nis_total_;
				cout << "NIS = " << (int)(nis_consistency_ * 1000) / 10.0 << endl;
			}
		}
	}
	catch (std::exception& ex)
	{
		cout << ex.what() << endl;
	}
}
