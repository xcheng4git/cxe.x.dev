#pragma once
#include "DenseVO.h"
#include <Eigen/core>

namespace vo
{
	class DENSEVO_API IntrinsicMatrix
	{
	public:
		struct Hash : std::unary_function<IntrinsicMatrix, std::size_t>
		{
			std::size_t operator()(IntrinsicMatrix const& value) const;
		};

		struct Equal : std::binary_function<IntrinsicMatrix, IntrinsicMatrix, bool>
		{
			bool operator()(IntrinsicMatrix const& left, IntrinsicMatrix const& right) const;
		};

		static IntrinsicMatrix create(float fx, float fy, float ox, float oy);

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		IntrinsicMatrix() {}

		IntrinsicMatrix(const IntrinsicMatrix& other);

		float fx() const;
		float fy() const;

		float ox() const;
		float oy() const;

		void invertOffset();
		void scale(float factor);

		Eigen::Matrix3f data;
	};
}

