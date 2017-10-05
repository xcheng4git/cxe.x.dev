#include "stdafx.h"
#include "KeyframeTracker.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include "KeyframeTracker.h"
#include "KeyframeGraph.h"
#include "LocalTracker.h"
#include "TrackingResultEvaluation.h"
#include "MapSerializer.h"

namespace slam
{
	class KeyframeTracker::Impl
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		friend class ::slam::KeyframeTracker;

		Impl(): graph_() 
			, lt_()
		{
			lt_.addMapInitializedCallback(boost::bind(&KeyframeTracker::Impl::onMapInitialized, this, _1, _2, _3));
			lt_.addMapCompleteCallback(boost::bind(&KeyframeTracker::Impl::onMapComplete, this, _1, _2));

			/*
			在lt_的accept_激活时 （LocalTracker.cpp : L165），会依次调用如下的5个函数，其返回值是五个函数结果的逻辑并，即真或假
			*/
			lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionTrackingResultEvaluation, this, _1, _2, _3));
			lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionEstimateDivergence, this, _1, _2, _3));
			lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionDistance, this, _1, _2, _3));
			lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionConstraintRatio, this, _1, _2, _3));
			lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionConditionNumber, this, _1, _2, _3));
		}


		slam::TrackingResultEvaluation::Ptr evaluation;
		AffineTransformd last_transform_to_keyframe_;

		void onMapInitialized(const LocalTracker& lt, const LocalMap::Ptr& m, const LocalTracker::TrackingResult& r_odometry)
		{
			std::cerr << __FUNCTION__ << std::endl << r_odometry.Information << std::endl;
			cv::imshow("keyframe", m->getKeyframe()->level(0).intensity / 255.0f);
			cv::waitKey(5);

			last_transform_to_keyframe_ = r_odometry.Transformation;

			evaluation.reset(new slam::LogLikelihoodTrackingResultEvaluation(r_odometry));
		}

		void onMapComplete(const slam::LocalTracker& lt, const slam::LocalMap::Ptr& m)
		{
			slam::TrackingResultEvaluation::ConstPtr const_evaluation(evaluation);
			m->setEvaluation(const_evaluation);
			graph_.add(m);
		}

		bool onAcceptCriterionTrackingResultEvaluation(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
		{
			bool accept = evaluation->ratioWithFirst(r_keyframe) > cfg_.MinEntropyRatio;

			if (accept)
				evaluation->add(r_keyframe);
			return accept;
		}

		bool onAcceptCriterionEstimateDivergence(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
		{
			bool reject1 = r_odometry.Transformation.translation().norm() > 0.1 || r_keyframe.Transformation.translation().norm() > 1.5 * cfg_.MaxTranslationalDistance;


			// TODO: awful hack
			if (reject1)
			{
				std::cerr << "od before modify: " << r_odometry.Transformation.translation().norm() << std::endl;
				Eigen::Affine3d& p = const_cast<Eigen::Affine3d&>(r_odometry.Transformation);
				p.setIdentity();
				std::cerr << "od after modify: " << r_odometry.Transformation.translation().norm() << std::endl;

				Matrix6d& information = const_cast<Matrix6d&>(r_odometry.Information);
				information.setIdentity();
				information *= 0.008 * 0.008;

				std::cerr << "kf before modify: " << r_keyframe.Transformation.translation().norm() << std::endl;
				Eigen::Affine3d& p2 = const_cast<Eigen::Affine3d&>(r_keyframe.Transformation);
				p2 = last_transform_to_keyframe_;
				std::cerr << "kf after modify: " << r_keyframe.Transformation.translation().norm() << std::endl;
			}

			last_transform_to_keyframe_ = r_keyframe.Transformation;

			return !reject1;
		}

		bool onAcceptCriterionDistance(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
		{
			//bool rtc = r_keyframe.Transformation.translation().norm() < cfg_.MaxTranslationalDistance;
			//if (!rtc) {
			//	std::cerr << __FUNCTION__ << std::endl;
			//	std::cerr << "r_keyframe.Transformation.translation().norm() : " << r_keyframe.Transformation.translation().norm() << " < " << cfg_.MaxTranslationalDistance << std::endl;
			//	std::cerr << r_keyframe.Transformation.translation();
			//}
			//return rtc;

			return r_keyframe.Transformation.translation().norm() < cfg_.MaxTranslationalDistance;
		}

		bool onAcceptCriterionConstraintRatio(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
		{
			return (double(r_keyframe.Statistics.Levels.back().Iterations.back().ValidConstraints) / double(r_keyframe.Statistics.Levels.back().ValidPixels)) > cfg_.MinEquationSystemConstraintRatio;
		}

		bool onAcceptCriterionConditionNumber(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
		{
			Eigen::SelfAdjointEigenSolver<Matrix6d> eigensolver1(r_odometry.Information);
			Vector6d eigenvalues = eigensolver1.eigenvalues().real();
			std::sort(eigenvalues.data(), eigenvalues.data() + eigenvalues.rows());

			Eigen::SelfAdjointEigenSolver<Matrix6d> eigensolver2(r_keyframe.Information);
			eigenvalues = eigensolver2.eigenvalues().real();
			std::sort(eigenvalues.data(), eigenvalues.data() + eigenvalues.rows());


			return true;
		}

		void onGlobalMapChangedUpdateVisualization(KeyframeGraph& map)
		{
		}


		void onGlobalMapChangedSaveTrajectory(KeyframeGraph& map)
		{
			static int idx = 1;

			std::stringstream ss;
			ss << "assoc_opt_traj" << idx << ".txt";

			slam::serialization::FileSerializer<slam::serialization::TrajectorySerializer> serializer(ss.str());
			serializer.serialize(map);

			++idx;
		}

		void forceKeyframe()
		{
			lt_.forceCompleteCurrentLocalMap();
		}

		void init(const Eigen::Affine3d& initial_transformation)
		{
			initial_transformation_ = initial_transformation;
			relative_transformation_.setIdentity();
		}

		void update(const vo::RgbdImagePyramid::Ptr& current, const double current_time, Eigen::Affine3d& absolute_transformation)
		{
			current->level(0).timestamp = current_time;

			if (!previous_)
			{
				previous_ = current;
				absolute_transformation = initial_transformation_;
				return;
			}

			if (!lt_.getLocalMap())
			{
				previous_->compute(4);  //only for debug. by Shawn at 20170521
				lt_.initNewLocalMap(previous_, current, initial_transformation_);
				lt_.getCurrentPose(absolute_transformation);
				return;
			}

			lt_.update(current, absolute_transformation);
		}

		void finish()
		{
			graph_.finalOptimization();
		}

	private:
		KeyframeGraph graph_;
		LocalTracker lt_;
		Eigen::Affine3d initial_transformation_, relative_transformation_, last_absolute_transformation_;
		vo::RgbdImagePyramid::Ptr previous_;

		KeyframeTrackerConfig cfg_;
	};

	KeyframeTracker::KeyframeTracker() :
		impl_(new KeyframeTracker::Impl())
	{
	}

	void KeyframeTracker::configureTracking(const vo::DenseTracker::Config& cfg)
	{
		impl_->graph_.configureValidationTracking(cfg);
		impl_->lt_.configure(cfg);
	}

	void KeyframeTracker::configureKeyframeSelection(const slam::KeyframeTrackerConfig& cfg)
	{
		impl_->cfg_ = cfg;
	}

	void KeyframeTracker::configureMapping(const slam::KeyframeGraphConfig& cfg)
	{
		impl_->graph_.configure(cfg);
	}

	const vo::DenseTracker::Config& KeyframeTracker::trackingConfiguration() const
	{
		return impl_->lt_.configuration();
	}

	const slam::KeyframeTrackerConfig& KeyframeTracker::keyframeSelectionConfiguration() const
	{
		return impl_->cfg_;
	}

	const slam::KeyframeGraphConfig& KeyframeTracker::mappingConfiguration() const
	{
		return impl_->graph_.configuration();
	}

	void KeyframeTracker::init()
	{
		init(Eigen::Affine3d::Identity());
	}

	void KeyframeTracker::init(const Eigen::Affine3d& initial_transformation)
	{
		impl_->init(initial_transformation);
	}

	void KeyframeTracker::update(const vo::RgbdImagePyramid::Ptr& current, const double current_time, Eigen::Affine3d& absolute_transformation)
	{
		impl_->update(current, current_time, absolute_transformation);
	}

	void KeyframeTracker::forceKeyframe()
	{
		impl_->forceKeyframe();
	}

	void KeyframeTracker::finish()
	{
		impl_->finish();
	}

	void KeyframeTracker::addMapChangedCallback(const slam::KeyframeGraph::MapChangedCallback& callback)
	{
		impl_->graph_.addMapChangedCallback(callback);
	}

	void KeyframeTracker::serializeMap(slam::serialization::MapSerializerInterface& serializer)
	{
		serializer.serialize(impl_->graph_);
	}
}
