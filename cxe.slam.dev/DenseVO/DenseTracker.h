#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Datatypes.h"
#include "IntrinsicMatrix.h"
#include "RgbdImagePyramid.h"
#include "WeightCalculation.h"
#include "PointSelection.h"

namespace vo
{
	class DENSEVO_API DenseTracker
	{
	public:
		struct DENSEVO_API Config
		{
			int FirstLevel, LastLevel;
			int MaxIterationsPerLevel;
			double Precision;
			double Mu; // precision (1/sigma^2) of prior

			bool UseInitialEstimate;
			bool UseWeighting;

			bool UseParallel;

			vo::InfluenceFunctions::enum_t InfluenceFuntionType;
			float InfluenceFunctionParam;

			vo::ScaleEstimators::enum_t ScaleEstimatorType;
			float ScaleEstimatorParam;

			float IntensityDerivativeThreshold;
			float DepthDerivativeThreshold;

			Config();
			size_t getNumLevels() const;

			bool UseEstimateSmoothing() const;

			bool IsSane() const;
		};

		struct TerminationCriteria
		{
			enum Enum
			{
				IterationsExceeded,
				IncrementTooSmall,
				LogLikelihoodDecreased,
				TooFewConstraints,
				NumCriteria
			};
		};

		struct DENSEVO_API IterationStats
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			size_t Id, ValidConstraints;

			double TDistributionLogLikelihood;
			Eigen::Vector2d TDistributionMean;
			Eigen::Matrix2d TDistributionPrecision;

			double PriorLogLikelihood;

			Vector6d EstimateIncrement;
			Matrix6d EstimateInformation;

			void InformationEigenValues(Vector6d& eigenvalues) const;

			double InformationConditionNumber() const;
		};
		typedef std::vector<IterationStats, Eigen::aligned_allocator<IterationStats> > IterationStatsVector;

		struct DENSEVO_API LevelStats
		{
			size_t Id, MaxValidPixels, ValidPixels;
			TerminationCriteria::Enum TerminationCriterion;
			IterationStatsVector Iterations;

			bool HasIterationWithIncrement() const;

			IterationStats& LastIterationWithIncrement();
			IterationStats& LastIteration();

			const IterationStats& LastIterationWithIncrement() const;
			const IterationStats& LastIteration() const;
		};
		typedef std::vector<LevelStats> LevelStatsVector;

		struct Stats
		{
			LevelStatsVector Levels;
		};

		struct DENSEVO_API Result
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			AffineTransformd Transformation;
			Matrix6d Information;
			double LogLikelihood;

			Stats Statistics;

			Result();

			bool isNaN() const;
			void setIdentity();
			void clearStatistics();
		};

		static const Config& getDefaultConfig();

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		DenseTracker(const Config& cfg = getDefaultConfig());
		DenseTracker(const vo::DenseTracker& other);

		const Config& configuration() const
		{
			return cfg;
		}

		void configure(const Config& cfg);

		bool match(vo::RgbdImagePyramid& reference, vo::RgbdImagePyramid& current, AffineTransformd& transformation);
		bool match(vo::PointSelection& reference, vo::RgbdImagePyramid& current, AffineTransformd& transformation);

		bool match(vo::RgbdImagePyramid& reference, vo::RgbdImagePyramid& current, vo::DenseTracker::Result& result);
		bool match(vo::PointSelection& reference, vo::RgbdImagePyramid& current, vo::DenseTracker::Result& result);

		cv::Mat computeIntensityErrorImage(vo::RgbdImagePyramid& reference, vo::RgbdImagePyramid& current, const AffineTransformd& transformation, size_t level = 0);


		static inline void computeJacobianOfProjectionAndTransformation(const Vector4& p, Matrix2x6& jacobian);

		static inline void compute3rdRowOfJacobianOfTransformation(const Vector4& p, Vector6& j);

		typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > ResidualVectorType;
		typedef std::vector<float> WeightVectorType;
	private:
		struct IterationContext
		{
			const Config& cfg;

			int Level;
			int Iteration;

			double Error, LastError;

			IterationContext(const Config& cfg);

			// returns true if this is the first iteration
			bool IsFirstIteration() const;

			// returns true if this is the first iteration on the current level
			bool IsFirstIterationOnLevel() const;

			// returns true if this is the first level
			bool IsFirstLevel() const;

			// returns true if this is the last level
			bool IsLastLevel() const;

			bool IterationsExceeded() const;

			// returns LastError - Error
			double ErrorDiff() const;
		};

		Config cfg;

		IterationContext itctx_;

		vo::WeightCalculation weight_calculation_;
		vo::PointSelection reference_selection_;
		vo::ValidPointAndGradientThresholdPredicate selection_predicate_;

		vo::PointWithIntensityAndDepth::VectorType points, points_error;

		ResidualVectorType residuals;
		WeightVectorType weights;
	};

} /* namespace dvo */

template<typename CharT, typename Traits>
std::ostream& operator<< (std::basic_ostream<CharT, Traits> &out, const vo::DenseTracker::Config &config)
{
	out
		<< "First Level = " << config.FirstLevel
		<< ", Last Level = " << config.LastLevel
		<< ", Max Iterations per Level = " << config.MaxIterationsPerLevel
		<< ", Precision = " << config.Precision
		<< ", Mu = " << config.Mu
		<< ", Use Initial Estimate = " << (config.UseInitialEstimate ? "true" : "false")
		<< ", Use Weighting = " << (config.UseWeighting ? "true" : "false")
		<< ", Scale Estimator = " << vo::ScaleEstimators::str(config.ScaleEstimatorType)
		<< ", Scale Estimator Param = " << config.ScaleEstimatorParam
		<< ", Influence Function = " << vo::InfluenceFunctions::str(config.InfluenceFuntionType)
		<< ", Influence Function Param = " << config.InfluenceFunctionParam
		<< ", Intensity Derivative Threshold = " << config.IntensityDerivativeThreshold
		<< ", Depth Derivative Threshold = " << config.DepthDerivativeThreshold
		;

	return out;
}

template<typename CharT, typename Traits>
std::ostream& operator<< (std::basic_ostream<CharT, Traits> &o, const vo::DenseTracker::IterationStats &s)
{
	o << "Iteration: " << s.Id << " ValidConstraints: " << s.ValidConstraints << " DataLogLikelihood: " << s.TDistributionLogLikelihood << " PriorLogLikelihood: " << s.PriorLogLikelihood << std::endl;

	return o;
}

template<typename CharT, typename Traits>
std::ostream& operator<< (std::basic_ostream<CharT, Traits> &o, const vo::DenseTracker::LevelStats &s)
{
	std::string termination;

	switch (s.TerminationCriterion)
	{
	case vo::DenseTracker::TerminationCriteria::IterationsExceeded:
		termination = "IterationsExceeded";
		break;
	case vo::DenseTracker::TerminationCriteria::IncrementTooSmall:
		termination = "IncrementTooSmall";
		break;
	case vo::DenseTracker::TerminationCriteria::LogLikelihoodDecreased:
		termination = "LogLikelihoodDecreased";
		break;
	case vo::DenseTracker::TerminationCriteria::TooFewConstraints:
		termination = "TooFewConstraints";
		break;
	default:
		break;
	}

	o << "Level: " << s.Id << " Pixel: " << s.ValidPixels << "/" << s.MaxValidPixels << " Termination: " << termination << " Iterations: " << s.Iterations.size() << std::endl;

	for (vo::DenseTracker::IterationStatsVector::const_iterator it = s.Iterations.begin(); it != s.Iterations.end(); ++it)
	{
		o << *it;
	}

	return o;
}

template<typename CharT, typename Traits>
std::ostream& operator<< (std::basic_ostream<CharT, Traits> &o, const vo::DenseTracker::Stats &s)
{
	o << s.Levels.size() << " levels" << std::endl;

	for (vo::DenseTracker::LevelStatsVector::const_iterator it = s.Levels.begin(); it != s.Levels.end(); ++it)
	{
		o << *it;
	}

	return o;
}


