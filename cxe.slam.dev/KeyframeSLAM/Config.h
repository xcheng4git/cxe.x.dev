#pragma once
#include "KeyframeSLAM.h"
#include <ostream>
#include <DenseVO\DenseTracker.h>

namespace slam
{
	struct KEYFRAMESLAM_API KeyframeTrackerConfig
	{
		 bool UseMultiThreading;

		 double MaxTranslationalDistance;
		 double MaxRotationalDistance;
		 double MinEntropyRatio;
		 double MinEquationSystemConstraintRatio;

		KeyframeTrackerConfig();
	};

	struct KEYFRAMESLAM_API KeyframeGraphConfig
	{
		 bool UseRobustKernel;
		 bool UseMultiThreading;

		 double NewConstraintSearchRadius;
		 double NewConstraintMinEntropyRatioCoarse;
		 double NewConstraintMinEntropyRatioFine;
		 double MinEquationSystemConstraintRatio;

		 bool OptimizationUseDenseGraph;
		 bool OptimizationRemoveOutliers;
		 double OptimizationOutlierWeightThreshold;
		 size_t OptimizationIterations;

		 bool FinalOptimizationUseDenseGraph;
		 bool FinalOptimizationRemoveOutliers;
		 double FinalOptimizationOutlierWeightThreshold;
		 size_t FinalOptimizationIterations;

		 size_t MinConstraintDistance;

		KeyframeGraphConfig();
	};

	struct KEYFRAMESLAM_API CameraDenseTrackerConfig
	{
		CameraDenseTrackerConfig();

		bool run_dense_tracking;
		bool use_dense_tracking_estimate;
		int coarsest_level;
		int finest_level;
		int max_iterations;
		double precision;
		bool use_initial_estimate;
		bool use_weighting;
		int scale_estimator;
		double scale_estimator_param;
		int influence_function;
		double influence_function_param;
		double min_intensity_deriv;
		double min_depth_deriv;
		double lambda;
		double mu;
		bool reconstruction;

		int RunDenseTracking;
		int UseDenseTrackingEstimate;
		int ConfigParam;
		int MiscParam;

		const static int NormalDistributionScaleEstimator = 1;
		const static int TDistributionScaleEstimator = 2;
		const static int MADScaleEstimator = 3;
		const static int TukeyInfluenceFunction = 1;
		const static int TDistributionInfluenceFunction = 2;
		const static int HuberInfluenceFunction = 3;
	};


	void KEYFRAMESLAM_API UpdateTrackerConfig(slam::CameraDenseTrackerConfig& config, vo::DenseTracker::Config& cfgTracker);

} /* namespace slam */

template<typename _CharT, typename _Traits>
std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& out, const slam::KeyframeTrackerConfig& cfg)
{
	out
		<< "UseMultiThreading: " << cfg.UseMultiThreading << " "
		<< "MaxTranslationalDistance: " << cfg.MaxTranslationalDistance << " "
		<< "MaxRotationalDistance: " << cfg.MaxRotationalDistance << " "
		<< "MinEntropyRatio: " << cfg.MinEntropyRatio << " "
		<< "MinEquationSystemConstraintRatio: " << cfg.MinEquationSystemConstraintRatio;

	return out;
}

template<typename _CharT, typename _Traits>
std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& out, const slam::KeyframeGraphConfig& cfg)
{
	out
		<< "UseRobustKernel: " << cfg.UseRobustKernel << " "
		<< "UseMultiThreading: " << cfg.UseMultiThreading << " "
		<< "NewConstraintSearchRadius: " << cfg.NewConstraintSearchRadius << " "
		<< "NewConstraintMinEntropyRatioCoarse: " << cfg.NewConstraintMinEntropyRatioCoarse << " "
		<< "NewConstraintMinEntropyRatioFine: " << cfg.NewConstraintMinEntropyRatioFine << " "
		<< "MinEquationSystemConstraintRatio: " << cfg.MinEquationSystemConstraintRatio << " "
		<< "MinConstraintDistance: " << cfg.MinConstraintDistance << " "
		<< "OptimizationUseDenseGraph: " << cfg.OptimizationUseDenseGraph << " "
		<< "OptimizationIterations: " << cfg.OptimizationIterations << " "
		<< "OptimizationRemoveOutliers: " << cfg.OptimizationRemoveOutliers << " "
		<< "OptimizationOutlierWeightThreshold: " << cfg.OptimizationOutlierWeightThreshold << " "
		<< "FinalOptimizationUseDenseGraph: " << cfg.FinalOptimizationUseDenseGraph << " "
		<< "FinalOptimizationIterations: " << cfg.FinalOptimizationIterations << " "
		<< "FinalOptimizationRemoveOutliers: " << cfg.FinalOptimizationRemoveOutliers << " "
		<< "FinalOptimizationOutlierWeightThreshold: " << cfg.FinalOptimizationOutlierWeightThreshold;

	return out;
}