#ifndef SEGWAY_SIM_COMMON_H
#define SEGWAY_SIM_COMMON_H

#include "segway_sim/CyberTimer.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>
#include <map>
#include <functional>
#include <signal.h>

static const uint32_t STATE_LENGTH = 7;
static const uint32_t INPUT_LENGTH = 2;
static const uint32_t CMD_LENGTH = 2;

inline void quat2eulZYX(const Eigen::Quaterniond &q,
                              Eigen::Vector3d &eul)
{
	double eul_tmp = 2.0 * (q.y() * q.y());
	eul(0) = atan2(2.0 * q.w() * q.x() + 2.0 * q.y() * q.z(), (1.0 - 2.0 * (q.x() * q.x())) - eul_tmp);
	
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (abs(sinp) >= 1)
		eul(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		eul(1) = asin(sinp);

	eul(2) = atan2(2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(), (1.0 - eul_tmp) - 2.0 * (q.z() * q.z()));
}

inline void quat2rotm(const Eigen::Quaterniond &q,
                            Eigen::Matrix3d &rotm)
{
	rotm(0,0) = 1-2*q.y()*q.y()-2*q.z()*q.z();
	rotm(0,1) = 2*q.x()*q.y()-2*q.z()*q.w();
	rotm(0,2) = 2*q.x()*q.z()+2*q.y()*q.w();
	rotm(1,0) = 2*q.x()*q.y()+2*q.z()*q.w();
	rotm(1,1) = 1-2*q.x()*q.x()-2*q.z()*q.z();
	rotm(1,2) = 2*q.y()*q.z()-2*q.x()*q.w();
	rotm(2,0) = 2*q.x()*q.z()-2*q.y()*q.w();
	rotm(2,1) = 2*q.y()*q.z()+2*q.x()*q.w();
	rotm(2,2) = 1-2*q.x()*q.x()-2*q.y()*q.y();
}

inline void eul2quatZYX(const Eigen::Vector3d &eul,
                              Eigen::Quaterniond &q)
{
	double cy = cos(eul(2) * 0.5);
	double sy = sin(eul(2) * 0.5);
	double cp = cos(eul(1) * 0.5);
	double sp = sin(eul(1) * 0.5);
	double cr = cos(eul(0) * 0.5);
	double sr = sin(eul(0) * 0.5);

	q.w() = cy * cp * cr + sy * sp * sr;
	q.x() = cy * cp * sr - sy * sp * cr;
	q.y() = sy * cp * sr + cy * sp * cr;
	q.z() = sy * cp * cr - cy * sp * sr;
}

inline void removeYaw(Eigen::Quaterniond &q)
{
	Eigen::Vector3d eul;
	quat2eulZYX(q,eul);
	eul(2) = 0;
	eul2quatZYX(eul,q);
}

inline Eigen::Matrix3d rotx(double angle)
{
	Eigen::Matrix3d rotm;
	rotm(0,0) =  1; rotm(0,1) =           0; rotm(0,2) =           0;
	rotm(1,0) =  0; rotm(1,1) =  cos(angle); rotm(1,2) = -sin(angle);
	rotm(2,0) =  0; rotm(2,1) =  sin(angle); rotm(2,2) =  cos(angle);
	return rotm;
}

inline Eigen::Matrix3d roty(double angle)
{
	Eigen::Matrix3d rotm;
	rotm(0,0) =  cos(angle); rotm(0,1) =  0; rotm(0,2) = sin(angle);
	rotm(1,0) =           0; rotm(1,1) =  1; rotm(1,2) = 0;
	rotm(2,0) = -sin(angle); rotm(2,1) =  0; rotm(2,2) = cos(angle);
	return rotm;
}

inline Eigen::Matrix3d rotz(double angle)
{
	Eigen::Matrix3d rotm;
	rotm(0,0) =  cos(angle); rotm(0,1) = -sin(angle); rotm(0,2) = 0;
	rotm(1,0) =  sin(angle); rotm(1,1) =  cos(angle); rotm(1,2) = 0;
	rotm(2,0) =           0; rotm(2,1) =           0; rotm(2,2) = 1;
	return rotm;
}

inline double deg2rad(double angle)
{
	return angle*M_PI/180.0;
}

inline double rad2deg(double angle)
{
	return angle*180.0/M_PI;
}

inline double saturate(double in, double min, double max)
{
	double out;
	if(min>max)
	{
		double tmp = max;
		max = min;
		min = tmp;
	}

	if(in>max)
		out = max;
	else if(in<min)
		out = min;
	else
		out = in;

	return out;
}

inline void saturateInPlace(double &in, double min, double max)
{
	if(min>max)
	{
		double tmp = max;
		max = min;
		min = tmp;
	}

	if(in>max)
		in = max;
	else if(in<min)
		in = min;
}

inline void saturateInPlace(double in[], double min, double max, uint32_t len)
{
	if(min>max)
	{
		double tmp = max;
		max = min;
		min = tmp;
	}

	for(uint32_t i = 0; i<len; i++)
	{
		if(in[i]>max)
			in[i] = max;
		else if(in[i]<min)
			in[i] = min;
	}

}

inline double constrainAngle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += M_PI;
    return x - M_PI;
}

inline void omega2eulRatesZYX(const Eigen::Vector3d &omega,
                              const Eigen::Quaterniond &q,
                                    Eigen::Vector3d &eulDot)
{
	Eigen::Vector3d eul;
	quat2eulZYX(q,eul);
	const double x = eul(0);
	const double y = eul(1);

	Eigen::Matrix3d M;
	M(0,0) = 1; M(0,1) = sin(x)*tan(y); M(0,2) = cos(x)*tan(y);
	M(1,0) = 0; M(1,1) =        cos(x); M(1,2) =       -sin(x);
	M(2,0) = 0; M(2,1) = sin(x)/cos(y); M(2,2) = cos(x)/cos(y);

	eulDot = M*omega;
}

inline void eulRates2omegaZYX(const Eigen::Vector3d &eulDot,
                              const Eigen::Quaterniond &q,
                                    Eigen::Vector3d &omega)
{
	Eigen::Vector3d eul;
	quat2eulZYX(q,eul);
	const double x = eul(0);
	const double y = eul(1);

	Eigen::Matrix3d M;
	M(0,0) = 1; M(0,1) =       0; M(0,2) =       -sin(y);
	M(1,0) = 0; M(1,1) =  cos(x); M(1,2) = sin(x)*cos(y);
	M(2,0) = 0; M(2,1) = -sin(x); M(2,2) = cos(x)*cos(y);

	omega = M*eulDot;
}
#endif