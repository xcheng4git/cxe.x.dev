#pragma once
#include "Keyframe.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace slam
{
	class KeyframeConstraintSearchInterface
	{
	public:
		virtual ~KeyframeConstraintSearchInterface() {}
		virtual void findPossibleConstraints(const KeyframeVector& all, const KeyframePtr& keyframe, KeyframeVector& candidates) = 0;
	};
	typedef boost::shared_ptr<KeyframeConstraintSearchInterface> KeyframeConstraintSearchInterfacePtr;

	class NearestNeighborConstraintSearch : public KeyframeConstraintSearchInterface
	{
	public:
		NearestNeighborConstraintSearch(float max_distance) :
			max_distance_(max_distance)
		{
		};
		virtual ~NearestNeighborConstraintSearch() {
		};

		float maxDistance() const;

		void maxDistance(const float& d);

		virtual void findPossibleConstraints(const KeyframeVector& all, const KeyframePtr& keyframe, KeyframeVector& candidates);
	private:
		float max_distance_;
	};
}

