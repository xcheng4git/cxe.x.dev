#pragma once
#include "KeyframeSLAM.h"
#include <iostream>
#include "MapSerializerInterface.h"

namespace slam
{

	namespace serialization
	{

		template<typename DelegateType>
		class FileSerializer : public MapSerializerInterface
		{
		public:
			FileSerializer(const std::string& filename) :
				fstream_(filename.c_str()),
				delegate_(fstream_)
			{
				std::cerr << filename << std::endl;
			}
			virtual ~FileSerializer()
			{
				fstream_.flush();
				fstream_.close();
			}

			virtual void serialize(const slam::KeyframeGraph& map)
			{
				delegate_.serialize(map);
			}
		private:
			std::ofstream fstream_;
			DelegateType delegate_;
		};

		class KEYFRAMESLAM_API TrajectorySerializer : public MapSerializerInterface
		{
		public:
			TrajectorySerializer(std::ostream& stream);
			virtual ~TrajectorySerializer();

			virtual void serialize(const slam::KeyframeGraph& map);
		private:
			std::ostream& stream_;
		};

		class KEYFRAMESLAM_API EdgeErrorSerializer : public MapSerializerInterface
		{
		public:
			EdgeErrorSerializer(std::ostream& stream);
			virtual ~EdgeErrorSerializer();

			virtual void serialize(const slam::KeyframeGraph& map);
		private:
			std::ostream& stream_;
		};
	} /* namespace serialization */
}

