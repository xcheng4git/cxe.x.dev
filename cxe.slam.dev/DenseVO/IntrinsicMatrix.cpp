#include "stdafx.h"
#include "IntrinsicMatrix.h"
#include <boost/unordered_map.hpp>

namespace vo
{

	std::size_t IntrinsicMatrix::Hash::operator ()(IntrinsicMatrix const& value) const
	{
		std::size_t seed = 0;

		boost::hash_combine(seed, value.fx());
		boost::hash_combine(seed, value.fy());
		boost::hash_combine(seed, value.ox());
		boost::hash_combine(seed, value.oy());

		return seed;
	}

	bool IntrinsicMatrix::Equal::operator()(IntrinsicMatrix const& left, IntrinsicMatrix const& right) const
	{
		return left.fx() == right.fx() && left.fy() == right.fy() && left.ox() == right.ox() && left.oy() == right.oy();
	}

	IntrinsicMatrix IntrinsicMatrix::create(float fx, float fy, float ox, float oy)
	{
		IntrinsicMatrix result;
		result.data.setZero();
		result.data(0, 0) = fx;
		result.data(1, 1) = fy;
		result.data(2, 2) = 1.0f;
		result.data(0, 2) = ox;
		result.data(1, 2) = oy;

		return result;
	}

	IntrinsicMatrix::IntrinsicMatrix(const IntrinsicMatrix & other) : data(other.data)
	{
	}

	float IntrinsicMatrix::fx() const
	{
		return data(0, 0);
	}

	float IntrinsicMatrix::fy() const
	{
		return data(1, 1);
	}

	float IntrinsicMatrix::ox() const
	{
		return data(0, 2);
	}

	float IntrinsicMatrix::oy() const
	{
		return data(1, 2);
	}

	void IntrinsicMatrix::invertOffset()
	{
		data(0, 2) *= -1;
		data(1, 2) *= -1;
	}

	void IntrinsicMatrix::scale(float factor)
	{
		data *= factor;
	}
}