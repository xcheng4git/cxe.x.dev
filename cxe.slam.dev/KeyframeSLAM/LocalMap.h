#pragma once
#include <g2o/core/sparse_optimizer.h>

#include <DenseVO\RgbdImagePyramid.h>

#include "TrackingResultEvaluation.h"
#include "Datatypes.h"

namespace slam
{
	namespace internal
	{
		struct LocalMapImpl;
	} /* namespace internal */

	class LocalMap
	{
	public:
		typedef boost::shared_ptr<LocalMap> Ptr;
		typedef boost::shared_ptr<const LocalMap> ConstPtr;

		virtual ~LocalMap();

		/**
		* Creates a new local map with the given keyframe.
		*/
		static LocalMap::Ptr create(const vo::RgbdImagePyramid::Ptr& keyframe, const AffineTransformd& keyframe_pose);

		vo::RgbdImagePyramid::Ptr getKeyframe();

		void setKeyframePose(const AffineTransformd& keyframe_pose);

		vo::RgbdImagePyramid::Ptr getCurrentFrame();

		void getCurrentFramePose(AffineTransformd& current_pose);
		AffineTransformd getCurrentFramePose();

		g2o::SparseOptimizer& getGraph();

		void setEvaluation(slam::TrackingResultEvaluation::ConstPtr& evaluation);

		slam::TrackingResultEvaluation::ConstPtr getEvaluation();

		/**
		* Adds a new RGB-D frame to the map, which becomes the active one.
		*/
		void addFrame(const vo::RgbdImagePyramid::Ptr& frame);

		/**
		* Adds a measurement connecting the last frame and the active frame to the map.
		*/
		void addOdometryMeasurement(const AffineTransformd& pose, const Matrix6d& information);

		/**
		* Adds a measurement connecting the keyframe and the active frame to the map.
		*/
		void addKeyframeMeasurement(const AffineTransformd& pose, const Matrix6d& information);

		void optimize();
	private:
		LocalMap(const vo::RgbdImagePyramid::Ptr& keyframe, const AffineTransformd& keyframe_pose);

		boost::scoped_ptr<internal::LocalMapImpl> impl_;
	};
}

