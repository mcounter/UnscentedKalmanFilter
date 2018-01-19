#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Base UKF process
class UKFProcess
{

protected:

	/**
	* Initialize weights of sigma points' weight vector
	*/
	virtual void initWeightsVector();

	/**
	* Normalize matrix
	* @param x Matrix
	*/
	virtual void normalizeMatrix(MatrixXd &m);

	/**
	* Create sigma points
	* @return Matrix with sigma points
	*/
	virtual MatrixXd createSigmaPoints();

	/**
	* Predict sigma points
	* @param sigma_points Sigma points
	* @param dt Time between k and k+1 step
	* @return Matrix with sigma points
	*/
	virtual MatrixXd predictSigmaPoints(MatrixXd &sigma_points, const float &dt);

public:
	
	///* Initially set to false, set to true in first call of ProcessMeasurement
	bool is_initialized_;

	///* State dimension
	int n_x_;

	///* Noise dimension
	int n_noise_;

	///* Sigma point spreading parameter
	double lambda_;

	///* State vector
	VectorXd x_;

	///* Noise vector
	VectorXd noise_;

	///* State covariance matrix
	MatrixXd P_;

	///* Predicted sigma points matrix (updated when Prediction method is called)
	MatrixXd Xsig_pred_;

	///* Sigma points mean residuals matrix (updated when Prediction method is called)
	MatrixXd Xsig_res_;

	///* Weights of sigma points (pre-calculated vector)
	VectorXd weights_;

	///* Weights of sigma points (pre-calculated matrix)
	MatrixXd m_weights_;

	/**
	* Constructor
	* @param n_x State dimension
	* @param n_noise Noise dimension
	* @param lambda Sigma point spreading parameter
	*/
	UKFProcess(const int &n_x, const int &n_noise, const double &lambda);

	/**
	* Destructor
	*/
	virtual ~UKFProcess();

	/**
	* Update (or initialize) Kalman filter process state and covariance matrix
	* @param x New state vector
	* @param P New state covariance vector
	*/
	virtual void UpdateState(VectorXd &x, MatrixXd &P);

	/**
	* Predict Predicts sigma points, the state, and the state covariance
	* matrix
	* @param dt Time between k and k+1 step
	*/
	virtual void Predict(const float &dt);
};

// Base UKF measurement
class UKFMeasurement
{

protected:

	// Pointer to UKF process
	UKFProcess* process_;

	///* Measurement dimension
	int n_z_;

	///* Noise vector
	VectorXd noise_;

	/**
	* Normalize measurement
	* @param z The measurement
	*/
	virtual void normalizeMeasurement(VectorXd &z);

	/**
	* Normalize matrix
	* @param x Matrix
	*/
	virtual void normalizeMatrix(MatrixXd &m);

	/**
	* Transform sigma points
	* @return Matrix with sigma points
	*/
	virtual MatrixXd transformSigmaPoints(MatrixXd &sigma_points);

	/**
	* Initialize state and covariance matrix from measurement
	* @param z The measurement
	* @param x State vector
	* @param P Covariance matrix
	*/
	virtual void initFromMeasurement(const VectorXd &z, VectorXd &x, MatrixXd &P);

	/**
	* Initialize state and covariance matrix from measurement
	* @param nis Normalized Innovation Squared
	*/
	virtual void analyzeNIS(double &nis);

public:

	///* Normalized Innovation Squared
	double nis_;

	///* Normalized Innovation Squared factor
	signed char nis_factor_;

	/**
	* Constructor
	* @param process UKF process
	* @param n_z Measurement dimension
	*/
	UKFMeasurement(UKFProcess &process, const int &n_z);

	/**
	* Destructor
	*/
	virtual ~UKFMeasurement();

	/**
	* Predict Predicts sigma points, the state, and the state covariance
	* matrix
	* @param dt Time between k and k+1 step
	*/
	virtual void Predict(const float &dt);

	/**
	* Updates the state and the state covariance matrix
	* @param meas_package The measurement at k+1
	*/
	virtual void Update(const VectorXd &z);
};

// UKF measurement for RADAR
class UKFMeasurementRadar : public UKFMeasurement
{

protected:

	// Minimal distance to point of origin
	// Measurement near point of origin has huge measurement error for radial measurement and must be prevented
	float min_distance_ = 0.01;

	/**
	* Normalize measurement
	* @param z The measurement
	*/
	void normalizeMeasurement(VectorXd &z);

	/**
	* Normalize matrix
	* @param x Matrix
	*/
	void normalizeMatrix(MatrixXd &m);

	/**
	* Transform sigma points
	* @return Matrix with sigma points
	*/
	MatrixXd transformSigmaPoints(MatrixXd &sigma_points);

	/**
	* Initialize state and covariance matrix from measurement
	* @param z The measurement
	* @param x State vector
	* @param P Covariance matrix
	*/
	void initFromMeasurement(const VectorXd &z, VectorXd &x, MatrixXd &P);

public:

	/**
	* Constructor
	* @param process UKF process
	* @param n_z Measurement dimension
	*/
	UKFMeasurementRadar(UKFProcess &process);

	/**
	* Destructor
	*/
	virtual ~UKFMeasurementRadar();
};


// UKF measurement for LIDAR
class UKFMeasurementLidar: public UKFMeasurement
{

protected:

	/**
	* Transform sigma points
	* @return Matrix with sigma points
	*/
	MatrixXd transformSigmaPoints(MatrixXd &sigma_points);

	/**
	* Initialize state and covariance matrix from measurement
	* @param z The measurement
	* @param x State vector
	* @param P Covariance matrix
	*/
	void initFromMeasurement(const VectorXd &z, VectorXd &x, MatrixXd &P);

public:

	/**
	* Constructor
	* @param process UKF process
	* @param n_z Measurement dimension
	*/
	UKFMeasurementLidar(UKFProcess &process);

	/**
	* Destructor
	*/
	virtual ~UKFMeasurementLidar();
};

// UKF for CTRV process model
class UKF : public UKFProcess
{

protected:
	///* Normalized Innovation Squared - total
	long nis_total_;

	///* Normalized Innovation Squared - consistent
	long nis_consistent_;

	///* Measurements classes for each sensor type
	UKFMeasurement* measurements_[SensorTypeSize];

	/**
	* Normalize matrix
	* @param x Matrix
	*/
	void normalizeMatrix(MatrixXd &m);

	/**
	* Predict sigma points
	* @param sigma_points Sigma points
	* @param dt Time between k and k+1 step
	* @return Matrix with sigma points
	*/
	MatrixXd predictSigmaPoints(MatrixXd &sigma_points, const float &dt);

public:

	///* If this is false, sensor measurements will be ignored (except for init)
	///* LASER = 0, RADAR = 1
	bool use_sensor_[SensorTypeSize] = { true, true};

	///* Time when the state is true, in us (initial state is 0)
	long long time_us_;

	///* Normalized Innovation Squared consistency
	double nis_consistency_;

	/**
	* Constructor
	*/
	UKF();

	/**
	* Destructor
	*/
	virtual ~UKF();

	/**
	* ProcessMeasurement
	* @param meas_package The latest measurement data of sensor
	*/
	virtual void ProcessMeasurement(const MeasurementPackage &measurement_pack);

};

#endif /* UKF_H */
