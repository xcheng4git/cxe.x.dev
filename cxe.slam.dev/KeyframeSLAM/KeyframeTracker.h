#pragma once

#include <DenseVO\RgbdImagePyramid.h>
#include <DenseVO\DenseTracker.h>

#include "KeyframeSLAM.h"
#include "KeyframeGraph.h"
#include "Config.h"
#include "MapSerializerInterface.h"
#include "Timer.h"

namespace slam
{
	class KEYFRAMESLAM_API KeyframeTracker
	{
	public:
		KeyframeTracker();

		const vo::DenseTracker::Config& trackingConfiguration() const;
		const slam::KeyframeTrackerConfig& keyframeSelectionConfiguration() const;
		const slam::KeyframeGraphConfig& mappingConfiguration() const;

		void configureTracking(const vo::DenseTracker::Config& cfg);

		void configureKeyframeSelection(const slam::KeyframeTrackerConfig& cfg);

		void configureMapping(const slam::KeyframeGraphConfig& cfg);

		void init();
		void init(const Eigen::Affine3d& initial_transformation);

		void update(const vo::RgbdImagePyramid::Ptr& current, const double current_time, Eigen::Affine3d& absolute_transformation);

		void forceKeyframe();

		void finish();

		void addMapChangedCallback(const slam::KeyframeGraph::MapChangedCallback& callback);

		void serializeMap(slam::serialization::MapSerializerInterface& serializer);
	private:
		class Impl;
		boost::shared_ptr<Impl> impl_;
	};
}

