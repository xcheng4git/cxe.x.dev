#include "stdafx.h"
#include "LocalTracker.h"
#include <DenseVO\PointSelection.h>

#include <tbb/parallel_invoke.h>
#include <tbb/tbb_thread.h>

#include "Timer.h"

namespace slam
{
	namespace internal
	{
		typedef boost::shared_ptr<vo::PointSelection> PointSelectionPtr;
		typedef boost::shared_ptr<vo::DenseTracker> DenseTrackerPtr;

		struct LocalTrackerImpl
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			friend class LocalTracker;

			DenseTrackerPtr keyframe_tracker_, odometry_tracker_;

			vo::ValidPointAndGradientThresholdPredicate predicate;

			AffineTransformd last_keyframe_pose_;

			PointSelectionPtr keyframe_points_, active_frame_points_;

			bool force_;

			LocalTracker::AcceptSignal accept_;
			LocalTracker::MapInitializedSignal map_initialized_;
			LocalTracker::MapCompleteSignal map_complete_;

			static void match(const DenseTrackerPtr& tracker, const PointSelectionPtr& ref, const vo::RgbdImagePyramid::Ptr& cur, LocalTracker::TrackingResult* r)
			{
				tracker->match(*ref, *cur, *r);
			}
		};
	} /* namespace internal */

	LocalTracker::LocalTracker() :
		impl_(new internal::LocalTrackerImpl())
	{
		impl_->keyframe_tracker_.reset(new vo::DenseTracker());
		impl_->odometry_tracker_.reset(new vo::DenseTracker());
		impl_->last_keyframe_pose_.setIdentity();
		impl_->force_ = false;
		impl_->keyframe_points_.reset(new vo::PointSelection(impl_->predicate));
		impl_->active_frame_points_.reset(new vo::PointSelection(impl_->predicate));
	}

	LocalTracker::~LocalTracker()
	{
	}

	slam::LocalMap::Ptr LocalTracker::getLocalMap() const
	{
		return local_map_;
	}

	void LocalTracker::getCurrentPose(AffineTransformd& pose)
	{
		local_map_->getCurrentFramePose(pose);
	}

	boost::signals2::connection LocalTracker::addAcceptCallback(const AcceptCallback& callback)
	{
		return impl_->accept_.connect(callback);
	}

	boost::signals2::connection LocalTracker::addMapCompleteCallback(const MapCompleteCallback& callback)
	{
		return impl_->map_complete_.connect(callback);
	}

	boost::signals2::connection LocalTracker::addMapInitializedCallback(const MapInitializedCallback& callback)
	{
		return impl_->map_initialized_.connect(callback);
	}

	const vo::DenseTracker::Config& LocalTracker::configuration() const
	{
		return impl_->odometry_tracker_->configuration();
	}

	void LocalTracker::configure(const vo::DenseTracker::Config& config)
	{
		impl_->keyframe_tracker_->configure(config);
		impl_->odometry_tracker_->configure(config);

		if (impl_->predicate.intensity_threshold != config.IntensityDerivativeThreshold || impl_->predicate.depth_threshold != config.DepthDerivativeThreshold)
		{
			impl_->predicate.intensity_threshold = config.IntensityDerivativeThreshold;
			impl_->predicate.depth_threshold = config.DepthDerivativeThreshold;

			if (local_map_)
			{
				impl_->keyframe_points_->setRgbdImagePyramid(*local_map_->getKeyframe());
			}
		}
	}
	void LocalTracker::initNewLocalMap(const vo::RgbdImagePyramid::Ptr& keyframe, const vo::RgbdImagePyramid::Ptr& frame, const AffineTransformd& keyframe_pose)
	{
		impl_->keyframe_points_->setRgbdImagePyramid(*keyframe);
		impl_->active_frame_points_->setRgbdImagePyramid(*frame);

		TrackingResult r_odometry;
		r_odometry.Transformation.setIdentity();

		impl_->odometry_tracker_->match(*(impl_->keyframe_points_), *frame, r_odometry);
		impl_->last_keyframe_pose_ = r_odometry.Transformation;

		initNewLocalMap(keyframe, frame, r_odometry, keyframe_pose);
	}

	void LocalTracker::initNewLocalMap(const vo::RgbdImagePyramid::Ptr& keyframe, const vo::RgbdImagePyramid::Ptr& frame, TrackingResult& r_odometry, const AffineTransformd& keyframe_pose)
	{
		// TODO: should be side effect free, i.e., not changing r_odometry
		if (r_odometry.isNaN())
		{
			std::cerr << "NaN in Map Initialization!" << std::endl;
			OutputDebugString(_T("NaN in Map Initialization!"));
			r_odometry.setIdentity();
		}

		local_map_ = LocalMap::create(keyframe, keyframe_pose);
		local_map_->addFrame(frame);
		local_map_->addKeyframeMeasurement(r_odometry.Transformation, r_odometry.Information);

		impl_->map_initialized_(*this, local_map_, r_odometry);
	}

	void LocalTracker::update(const vo::RgbdImagePyramid::Ptr& image, AffineTransformd& pose)
	{
		// prepare image
		const vo::DenseTracker::Config& config = impl_->keyframe_tracker_->configuration();
		image->build(config.getNumLevels());

		for (int idx = config.LastLevel; idx <= config.FirstLevel; ++idx)
		{
			image->level(idx).buildPointCloud();
			image->level(idx).buildAccelerationStructure();
		}

		TrackingResult r_odometry, r_keyframe;
		r_odometry.Transformation.setIdentity();
		r_keyframe.Transformation = impl_->last_keyframe_pose_.inverse(Eigen::Isometry);

		// recycle, so we can reuse the allocated memory
		impl_->active_frame_points_->setRgbdImagePyramid(*local_map_->getCurrentFrame());

		// TODO: fix me!
		boost::function<void()> h1 = boost::bind(&internal::LocalTrackerImpl::match, impl_->keyframe_tracker_, impl_->keyframe_points_, image, &r_keyframe);
		boost::function<void()> h2 = boost::bind(&internal::LocalTrackerImpl::match, impl_->odometry_tracker_, impl_->active_frame_points_, image, &r_odometry);

		tbb::parallel_invoke(h1, h2);

		if (r_odometry.isNaN()) {
			std::cerr << __FUNCTION__ << std::endl << "NaN in Odometry" << std::endl;
			OutputDebugString(_T("NAN in Odometry"));
		}
		if (r_keyframe.isNaN()) {
			std::cerr << __FUNCTION__ << std::endl << "NaN in Keyframe" << std::endl;
			OutputDebugString(_T("NAN in Keyframe"));
		}

		impl_->force_ = impl_->force_ || r_odometry.isNaN() || r_keyframe.isNaN();

		if (impl_->accept_(*this, r_odometry, r_keyframe) && !impl_->force_)
		{
			local_map_->addFrame(image);
			local_map_->addOdometryMeasurement(r_odometry.Transformation, r_odometry.Information);
			local_map_->addKeyframeMeasurement(r_keyframe.Transformation, r_keyframe.Information);

			impl_->last_keyframe_pose_ = r_keyframe.Transformation;
		}
		else
		{
			impl_->force_ = false;
			impl_->keyframe_points_.swap(impl_->active_frame_points_);

			slam::LocalMap::Ptr old_map = local_map_;
			AffineTransformd old_pose = old_map->getCurrentFramePose();
			impl_->map_complete_(*this, old_map);

			// TODO: if we have a tracking failure in odometry and in keyframe this initialization makes no sense
			initNewLocalMap(old_map->getCurrentFrame(), image, r_odometry, old_pose);

			impl_->last_keyframe_pose_ = r_odometry.Transformation;
		}

		local_map_->getCurrentFramePose(pose);
	}

	void LocalTracker::forceCompleteCurrentLocalMap()
	{
		impl_->force_ = true;
	}
}