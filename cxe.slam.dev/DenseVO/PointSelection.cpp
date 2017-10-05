#include "stdafx.h"
#include "PointSelection.h"


namespace vo
{
	PointSelection::PointSelection(const PointSelectionPredicate& predicate) :
		pyramid_(0),
		predicate_(predicate),
		debug_(false)
	{
		debug_ = true;
	}

	PointSelection::PointSelection(vo::RgbdImagePyramid& pyramid, const PointSelectionPredicate& predicate) :
		pyramid_(&pyramid),
		predicate_(predicate),
		debug_(false)
	{
		debug_ = true;
	}

	PointSelection::~PointSelection()
	{
	}

	void PointSelection::recycle(vo::RgbdImagePyramid& pyramid)
	{
		setRgbdImagePyramid(pyramid);
	}

	void PointSelection::setRgbdImagePyramid(vo::RgbdImagePyramid& pyramid)
	{
		pyramid_ = &pyramid;

		for(size_t idx = 0; idx < storage_.size(); ++idx)
		{
			storage_[idx].is_cached = false;
		}
	}

	vo::RgbdImagePyramid& PointSelection::getRgbdImagePyramid()
	{
		assert(pyramid_ != 0);

		return *pyramid_;
	}

	size_t PointSelection::getMaximumNumberOfPoints(const size_t& level)
	{
		return size_t(pyramid_->level(0).intensity.total() * std::pow(0.25, double(level)));
	}


	bool PointSelection::getDebugIndex(const size_t& level, cv::Mat& dbg_idx)
	{
		if(debug_ && storage_.size() > level)
		{
			dbg_idx = storage_[level].debug_idx;

			return dbg_idx.total() > 0;
		}
		else
		{
			return false;
		}
	}


	void PointSelection::select(const size_t& level, PointSelection::PointIterator& first_point, PointSelection::PointIterator& last_point)
	{
		assert(pyramid_ != 0);

		pyramid_->compute(level + 1);

		if(storage_.size() < level + 1)
			storage_.resize(level + 1);

		Storage& storage = storage_[level];

		if(!storage.is_cached || debug_)
		{
			vo::RgbdImage& img = pyramid_->level(level);
			img.buildPointCloud();
			img.buildAccelerationStructure();

			if(debug_)
				storage.debug_idx = cv::Mat::zeros(img.intensity.size(), CV_8UC1);

			storage.allocate(img.intensity.total());
			storage.points_end = selectPointsFromImage(img, storage.points.begin(), storage.points.end(), storage.debug_idx);

			storage.is_cached = true;
		}
		if (debug_) {
			std::stringstream ss;
			ss << "okindicator_lvl" << level << ".bmp";
			cv::imwrite(ss.str(), storage.debug_idx * 255);
			ss.clear(); ss.str("");
		}

		first_point = storage.points.begin();
		last_point = storage.points_end;
	}

	PointSelection::PointIterator PointSelection::selectPointsFromImage(const vo::RgbdImage& img, const PointSelection::PointIterator& first_point, const PointSelection::PointIterator& last_point, cv::Mat& debug_idx)
	{
		const PointWithIntensityAndDepth::Point *points = (const PointWithIntensityAndDepth::Point *) img.pointcloud.data();
		const PointWithIntensityAndDepth::IntensityAndDepth *intensity_and_depth = img.acceleration.ptr<PointWithIntensityAndDepth::IntensityAndDepth>();

		PointWithIntensityAndDepth::VectorType::iterator selected_points_it = first_point;

		//float dt = 1.0f / 30.0f / img.height;

		for(int y = 0; y < img.height; ++y)
		{
			//float time_interpolation = 1 + (y - 0.5f * img.height) * dt;

			for(int x = 0; x < img.width; ++x, ++points, ++intensity_and_depth)
			{
				if(predicate_.isPointOk(x, y, points->z, intensity_and_depth->idx, intensity_and_depth->idy, intensity_and_depth->zdx, intensity_and_depth->zdy))
				{
					selected_points_it->point = *points;
					selected_points_it->intensity_and_depth = *intensity_and_depth;
					//selected_points_it->intensity_and_depth.time_interpolation = time_interpolation;

					++selected_points_it;

					if(debug_)
						debug_idx.at<uint8_t>(y, x) = 1;

					if(selected_points_it == last_point)
						return selected_points_it;
				}
			}
		}

		return selected_points_it;
	}

	PointSelection::Storage::Storage() :
		points(),
		points_end(points.end()),
		is_cached(false)
	{
	}

	void PointSelection::Storage::allocate(size_t max_points)
	{
		if(points.size() < max_points)
		{
			points.resize(max_points);
		}
	}
}