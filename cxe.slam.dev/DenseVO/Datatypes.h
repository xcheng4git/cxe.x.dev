#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifndef __USER_DATATYPES__
#define __USER_DATATYPES__

	typedef float IntensityType;
	static const IntensityType Invalid = std::numeric_limits<IntensityType>::quiet_NaN();

	typedef float DepthType;
	static const DepthType InvalidDepth = std::numeric_limits<DepthType>::quiet_NaN();

	// float/double, determines numeric precision
	typedef float NumType;

	typedef Eigen::Matrix<NumType, 6, 6> Matrix6x6;
	typedef Eigen::Matrix<NumType, 1, 2> Matrix1x2;
	typedef Eigen::Matrix<NumType, 2, 6> Matrix2x6;

	typedef Eigen::Matrix<NumType, 6, 1> Vector6;
	typedef Eigen::Matrix<NumType, 4, 1> Vector4;

	typedef Eigen::Transform<NumType, 3, Eigen::Affine> AffineTransform;

	typedef Eigen::Affine3d AffineTransformd;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;

#endif