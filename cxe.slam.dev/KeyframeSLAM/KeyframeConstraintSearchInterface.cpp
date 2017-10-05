#include "stdafx.h"
#include "KeyframeConstraintSearchInterface.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>


namespace slam
{
	float NearestNeighborConstraintSearch::maxDistance() const
	{
		return max_distance_;
	}

	void NearestNeighborConstraintSearch::maxDistance(const float& d)
	{
		max_distance_ = d;
	}

	void NearestNeighborConstraintSearch::findPossibleConstraints(const KeyframeVector& all, const KeyframePtr& keyframe, KeyframeVector& candidates)
	{
		pcl::PointXYZ search_point;
		search_point.x = keyframe->pose().translation()(0);
		search_point.y = keyframe->pose().translation()(1);
		search_point.z = keyframe->pose().translation()(2);

		pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_points(new pcl::PointCloud<pcl::PointXYZ>());

		for (KeyframeVector::const_iterator it = all.begin(); it != all.end(); ++it)
		{
			pcl::PointXYZ p;
			p.x = (*it)->pose().translation()(0);
			p.y = (*it)->pose().translation()(1);
			p.z = (*it)->pose().translation()(2);

			keyframe_points->points.push_back(p);
		}

		std::vector<int> indices;
		//std::map<int, KeyframePtr> ids;
		std::vector<float> distances;

		pcl::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>(false));
		kdtree->setInputCloud(keyframe_points);
		kdtree->radiusSearch(search_point, max_distance_, indices, distances);
		
		//pcl::KdTree<pcl::PointXYZ> *kdtree_ = new pcl::KdTreeFLANN<pcl::PointXYZ>(false);
		//kdtree_->setInputCloud(keyframe_points);
		//kdtree_->radiusSearch(search_point, max_distance_, indices, distances);

		for (std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it)
		{
			candidates.push_back(all[*it]);
		}

		//delete kdtree_;
	}
}