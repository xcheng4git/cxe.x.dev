#pragma once
#include <g2o/core/optimizable_graph.h>
namespace slam
{
	class Timestamped : public g2o::OptimizableGraph::Data
	{
	public:
		Timestamped()
		{
		}

		Timestamped(const double new_timestamp) :
			timestamp(new_timestamp)
		{
			//assert(!timestamp.isZero());
		}

		//! read the data from a stream
		virtual bool read(std::istream& is)
		{
			is >> timestamp;

			return true;
		}

		//! write the data to a stream
		virtual bool write(std::ostream& os) const
		{
			os << timestamp;
			return true;
		}

		double timestamp;
	};
}