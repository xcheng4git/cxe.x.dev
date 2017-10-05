#include "stdafx.h"
#include "MapSerializer.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

#include "Timestamped.h"

namespace slam
{
	namespace serialization
	{

		TrajectorySerializer::TrajectorySerializer(std::ostream& stream) :
			stream_(stream)
		{
		}

		TrajectorySerializer::~TrajectorySerializer()
		{
		}

		void TrajectorySerializer::serialize(const slam::KeyframeGraph& map)
		{
			std::map<double, Eigen::Isometry3d> poses;

			for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = map.graph().vertices().begin(); it != map.graph().vertices().end(); ++it)
			{
				g2o::VertexSE3 *v = (g2o::VertexSE3 *) it->second;

				Timestamped *t = dynamic_cast<Timestamped *>(v->userData());

				assert(t != 0);

				poses[t->timestamp] = v->estimate();
			}

			for (std::map<double, Eigen::Isometry3d>::iterator it = poses.begin(); it != poses.end(); ++it)
			{
				Eigen::Quaterniond q(it->second.rotation());

				stream_ << it->first << " " << it->second.translation()(0) << " " << it->second.translation()(1) << " " << it->second.translation()(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << std::endl;
			}
		}

		EdgeErrorSerializer::EdgeErrorSerializer(std::ostream& stream) :
			stream_(stream)
		{
		}

		EdgeErrorSerializer::~EdgeErrorSerializer()
		{
		}

		void EdgeErrorSerializer::serialize(const slam::KeyframeGraph& map)
		{
			for (g2o::HyperGraph::EdgeSet::const_iterator it = map.graph().edges().begin(); it != map.graph().edges().end(); ++it)
			{
				g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);

				int type = edge->vertices().size() == 2 && edge->level() == 0 && edge->vertex(0)->id() > 0 && edge->vertex(1)->id() > 0 ? 1 : 0;

				Eigen::Vector3d rho;
				if (edge->robustKernel() != 0)
					edge->robustKernel()->robustify(edge->chi2(), rho);
				else
					rho.setZero();

				//if(edge->vertices().size() == 2 && edge->level() == 0 && edge->vertex(0)->id() > 0 && edge->vertex(1)->id() > 0)
				stream_ << type << " " << edge->error().transpose() << " " << edge->chi2() << " " << rho.transpose() << std::endl;
			}
		}
	} /* namespace serialization */
}