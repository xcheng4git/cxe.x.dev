#include "stdafx.h"
#include "DenseTracker.h"
#include <Eigen/Eigenvalues>

namespace vo
{
	DenseTracker::Config::Config() :
		FirstLevel(3),
		LastLevel(1),
		MaxIterationsPerLevel(100),
		Precision(5e-7),
		UseInitialEstimate(false),
		UseWeighting(true),
		Mu(0),
		InfluenceFuntionType(vo::InfluenceFunctions::TDistribution),
		InfluenceFunctionParam(vo::TDistributionInfluenceFunction::DEFAULT_DOF),
		ScaleEstimatorType(vo::ScaleEstimators::TDistribution),
		ScaleEstimatorParam(vo::TDistributionScaleEstimator::DEFAULT_DOF),
		IntensityDerivativeThreshold(0.0f),
		DepthDerivativeThreshold(0.0f)
	{
	}

	size_t DenseTracker::Config::getNumLevels() const
	{
		return FirstLevel + 1;
	}

	bool DenseTracker::Config::UseEstimateSmoothing() const
	{
		return Mu > 1e-6;
	}

	bool DenseTracker::Config::IsSane() const
	{
		return FirstLevel >= LastLevel;
	}

	DenseTracker::IterationContext::IterationContext(const Config& cfg) :
		cfg(cfg)
	{
	}

	bool DenseTracker::IterationContext::IsFirstIteration() const
	{
		return IsFirstLevel() && IsFirstIterationOnLevel();
	}

	bool DenseTracker::IterationContext::IsFirstIterationOnLevel() const
	{
		return Iteration == 0;
	}

	bool DenseTracker::IterationContext::IsFirstLevel() const
	{
		return cfg.FirstLevel == Level;
	}

	bool DenseTracker::IterationContext::IsLastLevel() const
	{
		return cfg.LastLevel == Level;
	}

	double DenseTracker::IterationContext::ErrorDiff() const
	{
		return LastError - Error;
	}

	bool DenseTracker::IterationContext::IterationsExceeded() const
	{
		int max_iterations = cfg.MaxIterationsPerLevel;

		return Iteration >= max_iterations;
	}

	bool DenseTracker::Result::isNaN() const
	{
		return !std::isfinite(Transformation.matrix().sum()) || !std::isfinite(Information.sum());
	}

	DenseTracker::Result::Result() :
		LogLikelihood(std::numeric_limits<double>::max())
	{
		double nan = std::numeric_limits<double>::quiet_NaN();
		Transformation.linear().setConstant(nan);
		Transformation.translation().setConstant(nan);
		Information.setIdentity();
	}

	void DenseTracker::Result::setIdentity()
	{
		Transformation.setIdentity();
		Information.setIdentity();
		LogLikelihood = 0.0;
	}

	void DenseTracker::Result::clearStatistics()
	{
		Statistics.Levels.clear();
	}

	void DenseTracker::IterationStats::InformationEigenValues(Vector6d& eigenvalues) const
	{
		Eigen::EigenSolver<Matrix6d> evd(EstimateInformation);
		eigenvalues = evd.eigenvalues().real();
		std::sort(eigenvalues.data(), eigenvalues.data() + eigenvalues.rows());
	}

	double DenseTracker::IterationStats::InformationConditionNumber() const
	{
		Vector6d ev;
		InformationEigenValues(ev);

		return std::abs(ev(5) / ev(0));
	}


	bool DenseTracker::LevelStats::HasIterationWithIncrement() const
	{
		int min = TerminationCriterion == DenseTracker::TerminationCriteria::LogLikelihoodDecreased || TerminationCriterion == DenseTracker::TerminationCriteria::TooFewConstraints ? 2 : 1;

		return Iterations.size() >= min;
	}

	DenseTracker::IterationStats& DenseTracker::LevelStats::LastIterationWithIncrement()
	{
		if (!HasIterationWithIncrement())
		{
			std::cerr << "awkward " << *this << std::endl;

			assert(false);
		}

		return TerminationCriterion == DenseTracker::TerminationCriteria::LogLikelihoodDecreased ? Iterations[Iterations.size() - 2] : Iterations[Iterations.size() - 1];
	}

	DenseTracker::IterationStats& DenseTracker::LevelStats::LastIteration()
	{
		return Iterations.back();
	}

	const DenseTracker::IterationStats& DenseTracker::LevelStats::LastIterationWithIncrement() const
	{
		if (!HasIterationWithIncrement())
		{
			std::cerr << "awkward " << *this << std::endl;

			assert(false);
		}
		return TerminationCriterion == DenseTracker::TerminationCriteria::LogLikelihoodDecreased ? Iterations[Iterations.size() - 2] : Iterations[Iterations.size() - 1];
	}

	const DenseTracker::IterationStats& DenseTracker::LevelStats::LastIteration() const
	{
		return Iterations.back();
	}
}