#include "stdafx.h"
#include "TrackingResultEvaluation.h"

namespace slam
{
	void TrackingResultEvaluation::add(const vo::DenseTracker::Result& r)
	{
		average_ += value(r);
		n_ += 1.0;
	}

	double TrackingResultEvaluation::ratioWithFirst(const vo::DenseTracker::Result& r) const
	{
		return value(r) / first_;
	}

	double TrackingResultEvaluation::ratioWithAverage(const vo::DenseTracker::Result& r) const
	{
		return value(r) / average_* n_;
	}

	TrackingResultEvaluation::TrackingResultEvaluation(double first)
	{
		first_ = first;
		average_ = first_;
		n_ = 1.0;
	}

	double EntropyRatioTrackingResultEvaluation::value(const vo::DenseTracker::Result& r) const
	{
		return std::log(r.Information.determinant());
	}

	double LogLikelihoodTrackingResultEvaluation::value(const vo::DenseTracker::Result& r) const
	{
		return -r.LogLikelihood;
	}

	double NormalizedLogLikelihoodTrackingResultEvaluation::value(const vo::DenseTracker::Result& r) const
	{
		return -r.LogLikelihood / r.Statistics.Levels.back().Iterations.back().ValidConstraints;
	}
}
