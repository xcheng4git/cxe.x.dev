#include "stdafx.h"
#include "Config.h"

namespace slam
{

	KeyframeTrackerConfig::KeyframeTrackerConfig() :
		UseMultiThreading(true),
		MaxTranslationalDistance(0.2),
		//MaxRotationalDistance(std::numeric_limits<double>::max()),
		//MinEntropyRatio(0.91),
		//MinEquationSystemConstraintRatio(0.33)
		MaxRotationalDistance(0),
		MinEntropyRatio(0.6),
		MinEquationSystemConstraintRatio(0.3)
	{
	}

	KeyframeGraphConfig::KeyframeGraphConfig() :
		UseRobustKernel(true),
		UseMultiThreading(true),
		NewConstraintSearchRadius(5.0),
		NewConstraintMinEntropyRatioCoarse(0.03),
		NewConstraintMinEntropyRatioFine(0.6),
		MinEquationSystemConstraintRatio(0.3),
		MinConstraintDistance(0),
		OptimizationUseDenseGraph(false),
		OptimizationIterations(20),
		OptimizationRemoveOutliers(true),
		OptimizationOutlierWeightThreshold(0.1),
		FinalOptimizationUseDenseGraph(true),
		FinalOptimizationIterations(1000),
		FinalOptimizationRemoveOutliers(true),
		FinalOptimizationOutlierWeightThreshold(0.1)
	{
	}

	CameraDenseTrackerConfig::CameraDenseTrackerConfig()
	{
		run_dense_tracking = false;
		use_dense_tracking_estimate = false;
		coarsest_level = 3;
		finest_level = 1;
		max_iterations = 50;
		precision = 0.0001;
		use_initial_estimate = true;
		use_weighting = true;
		scale_estimator = 2;
		scale_estimator_param = 5.0;
		influence_function = 2;
		influence_function_param = 5.0;
		min_intensity_deriv = 0.0;
		min_depth_deriv = 0.0;
		lambda = 0;
		mu = 0.05;
		reconstruction = true;

		RunDenseTracking = 1;
		UseDenseTrackingEstimate = 2;
		ConfigParam = 4;
		MiscParam = 8;
	}

	void UpdateTrackerConfig(slam::CameraDenseTrackerConfig& config, vo::DenseTracker::Config& cfgTracker)
	{
		vo::ScaleEstimators::enum_t scale_estimator;
		switch (config.scale_estimator) {
		case slam::CameraDenseTrackerConfig::NormalDistributionScaleEstimator:
			scale_estimator = vo::ScaleEstimators::NormalDistribution;
			break;
		case slam::CameraDenseTrackerConfig::TDistributionScaleEstimator:
			scale_estimator = vo::ScaleEstimators::TDistribution;
			break;
		case slam::CameraDenseTrackerConfig::MADScaleEstimator:
			scale_estimator = vo::ScaleEstimators::MAD;
			break;
		default:
			assert(false && "unknown scale estimator");
			break;
		}

		vo::InfluenceFunctions::enum_t influence_function;
		switch (config.influence_function) {
		case slam::CameraDenseTrackerConfig::TukeyInfluenceFunction:
			influence_function = vo::InfluenceFunctions::Tukey;
			break;
		case slam::CameraDenseTrackerConfig::TDistributionInfluenceFunction:
			influence_function = vo::InfluenceFunctions::TDistribution;
			break;
		case slam::CameraDenseTrackerConfig::HuberInfluenceFunction:
			influence_function = vo::InfluenceFunctions::Huber;
			break;
		default:
			assert(false && "unknown influence function");
			break;
		}
		cfgTracker.FirstLevel = config.coarsest_level;
		cfgTracker.LastLevel = config.finest_level;
		cfgTracker.MaxIterationsPerLevel = config.max_iterations;
		cfgTracker.Precision = config.precision;
		cfgTracker.UseInitialEstimate = config.use_initial_estimate;
		cfgTracker.UseWeighting = config.use_weighting;
		cfgTracker.ScaleEstimatorType = scale_estimator;
		cfgTracker.ScaleEstimatorParam = config.scale_estimator_param;
		cfgTracker.InfluenceFuntionType = influence_function;
		cfgTracker.InfluenceFunctionParam = config.influence_function_param;
		cfgTracker.Mu = config.mu;
		cfgTracker.IntensityDerivativeThreshold = config.min_intensity_deriv;
		cfgTracker.DepthDerivativeThreshold = config.min_depth_deriv;

	}
}