#include "stdafx.h"
#include "SurfacePyramid.h"
#include <iostream>
#include <limits>
#include <immintrin.h>

namespace vo
{

	SurfacePyramid::SurfacePyramid()
	{
		// TODO Auto-generated constructor stub

	}

	SurfacePyramid::~SurfacePyramid()
	{
		// TODO Auto-generated destructor stub
	}

	/**
	* Converts the given raw depth image (type CV_16UC1) to a CV_32FC1 image rescaling every pixel with the given scale
	* and replacing 0 with NaNs.
	*/
	void SurfacePyramid::convertRawDepthImage(const cv::Mat& input, cv::Mat& output, float scale)
	{
		output.create(input.rows, input.cols, CV_32FC1);

		const unsigned short* input_ptr = input.ptr<unsigned short>();
		float* output_ptr = output.ptr<float>();

		for (int idx = 0; idx < input.size().area(); idx++, input_ptr++, output_ptr++)
		{
			if (*input_ptr == 0)
			{
				*output_ptr = std::numeric_limits<float>::quiet_NaN();
			}
			else
			{
				*output_ptr = ((float)*input_ptr) * scale;
			}
		}
	}

	void SurfacePyramid::convertRawDepthImageSse(const cv::Mat& input, cv::Mat& output, float scale)
	{
		output.create(input.rows, input.cols, CV_32FC1);

		const unsigned short* input_ptr = input.ptr<unsigned short>();
		float* output_ptr = output.ptr<float>();

#if 1
		__m128 _scale = _mm_set1_ps(scale);
		__m128 _zero = _mm_setzero_ps();
		__m128 _nan = _mm_set1_ps(std::numeric_limits<float>::quiet_NaN());

		for (int idx = 0; idx < input.size().area(); idx += 8, input_ptr += 8, output_ptr += 8)
		{
			__m128 _input, mask;
			__m128i _inputi = _mm_load_si128((__m128i*) input_ptr);

			// load low shorts and convert to float
			_input = _mm_cvtepi32_ps(_mm_unpacklo_epi16(_inputi, _mm_setzero_si128()));

			mask = _mm_cmpeq_ps(_input, _zero);

			// zero to nan
			_input = _mm_or_ps(_input, _mm_and_ps(mask, _nan));
			// scale
			_input = _mm_mul_ps(_input, _scale);
			// save
			_mm_store_ps(output_ptr + 0, _input);

			// load high shorts and convert to float
			_input = _mm_cvtepi32_ps(_mm_unpackhi_epi16(_inputi, _mm_setzero_si128()));

			mask = _mm_cmpeq_ps(_input, _zero);

			// zero to nan
			_input = _mm_or_ps(_input, _mm_and_ps(mask, _nan));
			// scale
			_input = _mm_mul_ps(_input, _scale);
			// save
			_mm_store_ps(output_ptr + 4, _input);
		}
#else
		__m256 _scale = _mm256_set1_ps(scale);
		__m256i _zeroi = _mm256_setzero_si256();
		__m256 _nan = _mm256_set1_ps(std::numeric_limits<float>::quiet_NaN());
		for (int idx = 0; idx < input.size().area(); idx += 16, input_ptr += 16, output_ptr += 16) {
			__m256 _input;
			__m256i _inputi16 = _mm256_load_si256((__m256i*) input_ptr), maski;
			__m256i _inputi32 = _mm256_unpacklo_epi16(_inputi16, _mm256_setzero_si256());
			maski = _mm256_cmpeq_epi32(_inputi32, _zeroi);

			_input = _mm256_or_ps(_mm256_cvtepi32_ps(_inputi32), _mm256_and_ps(_mm256_cvtepi32_ps(maski), _nan));
			_input = _mm256_mul_ps(_input, _scale);
			_mm256_store_ps(output_ptr + 0, _input);

			_inputi32 = _mm256_unpackhi_epi16(_inputi16, _mm256_setzero_si256());
			maski = _mm256_cmpeq_epi32(_inputi32, _zeroi);

			_input = _mm256_or_ps(_mm256_cvtepi32_ps(_inputi32), _mm256_and_ps(_mm256_cvtepi32_ps(maski), _nan));
			_input = _mm256_mul_ps(_input, _scale);
			_mm256_store_ps(output_ptr + 8, _input);
		}
#endif
	}
}