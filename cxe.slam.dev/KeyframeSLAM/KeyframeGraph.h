#pragma once
#include "KeyframeSLAM.h"
#include <boost/function.hpp>

#include "Config.h"
#include "LocalMap.h"
#include "Keyframe.h"

#include <DenseVO\DenseTracker.h>

namespace slam
{
	namespace internal
	{
		class KeyframeGraphImpl;
		typedef boost::scoped_ptr<KeyframeGraphImpl> KeyframeGraphImplPtr;
	}

	class KEYFRAMESLAM_API  KeyframeGraph
	{
	public:
		typedef void MapChangedCallbackSignature(KeyframeGraph&);
		typedef boost::function<MapChangedCallbackSignature> MapChangedCallback;

		KeyframeGraph();
		virtual ~KeyframeGraph();

		const slam::KeyframeGraphConfig& configuration() const;

		void configure(const slam::KeyframeGraphConfig& config);

		void configureValidationTracking(const vo::DenseTracker::Config& cfg);

		void add(const slam::LocalMap::Ptr& keyframe);

		void finalOptimization();

		void addMapChangedCallback(const KeyframeGraph::MapChangedCallback& callback);

		const KeyframeVector& keyframes() const;

		const g2o::SparseOptimizer& graph() const;

		cv::Mat computeIntensityErrorImage(int edge_id, bool use_measurement = true) const;

		void debugLoopClosureConstraint(int keyframe1, int keyframe2) const;

	private:
		internal::KeyframeGraphImplPtr impl_;
	};
}


