#pragma once
#include "KeyframeSLAM.h"

#include "KeyframeGraph.h"

namespace slam
{
	namespace serialization
	{

		class KEYFRAMESLAM_API MapSerializerInterface
		{
		public:
			MapSerializerInterface();
			virtual ~MapSerializerInterface();

			virtual void serialize(const slam::KeyframeGraph& map) = 0;
		};

	} /* namespace serialization */
}

