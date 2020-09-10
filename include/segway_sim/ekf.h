#ifndef EKF_H
#define EKF_H

#include "dynamics_ekf.h"
#include <Eigen/LU>

class EKF
{
public:
	// Methods
	EKF(const std::vector<double> &Q,
	    const std::vector<double> &R);

	void init(const std::vector<double> &x0,
			  const std::vector<double> &P0vec);

	void update(const std::vector<double> &U,
				const std::vector<double> &M,
				const double dt);

	// Attributes
	std::vector<double> x_t_;
	std::vector<double> P_t_;
	const std::vector<double> Q_;
	const std::vector<double> R_;
	uint32_t initialized = 0;
};

#endif