#pragma once
#include <mmintrin.h>
#include <emmintrin.h>

#include <opencv2/core/core.hpp>

#include "DenseVO.h"

namespace vo
{
	class DENSEVO_API SurfacePyramid
	{
	public:

		/**
		* Converts the given raw depth image (type CV_16U) to a CV_32F image rescaling every pixel with the given scale
		* and replacing 0 with NaNs.
		*/
		static void convertRawDepthImage(const cv::Mat& input, cv::Mat& output, float scale);

		static void convertRawDepthImageSse(const cv::Mat& input, cv::Mat& output, float scale);

		SurfacePyramid();
		virtual ~SurfacePyramid();
	};
}
