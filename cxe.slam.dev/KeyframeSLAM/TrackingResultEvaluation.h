#pragma once
#include <boost/shared_ptr.hpp>
#include <DenseVO\DenseTracker.h>

namespace slam
{
	class TrackingResultEvaluation
	{
	public:
		typedef boost::shared_ptr<TrackingResultEvaluation> Ptr;
		typedef boost::shared_ptr<const TrackingResultEvaluation> ConstPtr;

		virtual ~TrackingResultEvaluation() {};

		virtual void add(const vo::DenseTracker::Result& r);

		virtual double ratioWithFirst(const vo::DenseTracker::Result& r) const;

		virtual double ratioWithAverage(const vo::DenseTracker::Result& r) const;

	protected:
		TrackingResultEvaluation(double first);

		virtual double value(const vo::DenseTracker::Result& r) const = 0;
	private:
		double first_, average_, n_;
	};

	class LogLikelihoodTrackingResultEvaluation : public TrackingResultEvaluation
	{
	public:
		LogLikelihoodTrackingResultEvaluation(const vo::DenseTracker::Result& r) : TrackingResultEvaluation(value(r)) {};
		virtual ~LogLikelihoodTrackingResultEvaluation() {};
		virtual double value(const vo::DenseTracker::Result& r) const;
	};

	class NormalizedLogLikelihoodTrackingResultEvaluation : public TrackingResultEvaluation
	{
	public:
		NormalizedLogLikelihoodTrackingResultEvaluation(const vo::DenseTracker::Result& r) : TrackingResultEvaluation(value(r)) {};
		virtual ~NormalizedLogLikelihoodTrackingResultEvaluation() {};
		virtual double value(const vo::DenseTracker::Result& r) const;
	};


	class EntropyRatioTrackingResultEvaluation : public TrackingResultEvaluation
	{
	public:
		EntropyRatioTrackingResultEvaluation(const vo::DenseTracker::Result& r) : TrackingResultEvaluation(value(r)) {};
		virtual ~EntropyRatioTrackingResultEvaluation() {};
		virtual double value(const vo::DenseTracker::Result& r) const;
	};
}

