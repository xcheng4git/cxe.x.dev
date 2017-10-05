#pragma once
#include <Eigen/Geometry>
#include <vector>

#include <DenseVO\RgbdImagePyramid.h>
#include "FluentInterface.h"
#include "Timer.h"
#include "TrackingResultEvaluation.h"

namespace slam
{
	class Keyframe
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Keyframe() : id_(-1) {};
		virtual ~Keyframe() {};

		FI_ATTRIBUTE(Keyframe, short, id)
		FI_ATTRIBUTE(Keyframe, vo::RgbdImagePyramid::Ptr, image)
		FI_ATTRIBUTE(Keyframe, Eigen::Affine3d, pose)
		FI_ATTRIBUTE(Keyframe, slam::TrackingResultEvaluation::ConstPtr, evaluation)

		double timestamp() const
		{
			Timing::Timer timer;
			return timer.AbsoluteTime();
		}
	};

	typedef boost::shared_ptr<Keyframe> KeyframePtr;
	typedef std::vector<KeyframePtr> KeyframeVector;

}

