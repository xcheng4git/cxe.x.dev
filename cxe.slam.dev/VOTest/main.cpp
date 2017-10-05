// VOTest.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include <iostream>
#include <string>
using namespace std;
#include "Timer.h"

#include <DenseVO\IntrinsicMatrix.h>
#include <DenseVO\RgbdImagePyramid.h>
#include <DenseVO\DenseTracker.h>
#include <DenseVO\SurfacePyramid.h>

fstream *trajectory_out_;
vo::RgbdImagePyramidPtr load(vo::RgbdCameraPyramid& camera, std::string rgb_file, std::string depth_file);
void ConfigurateTracker(vo::DenseTracker::Config& cfgTracker);

int main()
{
	std::string folder = "D:\\#project\\Kinect 1\\cxe.kinect1.dev\\data\\d4\\";
	std::string trajectoryfile = folder + "output\\trajectory.txt";
	trajectory_out_ = new std::fstream(trajectoryfile.c_str());
	if (trajectory_out_->fail()) {
		delete trajectory_out_;
		std::cerr << "Failed to open '" << trajectoryfile << "'!" << std::endl;
		return 0;
	}
	trajectory_out_->setf(ios::fixed);

	vo::IntrinsicMatrix intrinsics = vo::IntrinsicMatrix::create(518.0, 519.0, 325.5, 253.5);
	vo::RgbdCameraPyramid camera(640, 480, intrinsics);
	vo::DenseTracker::Config cfg = vo::DenseTracker::getDefaultConfig();
	ConfigurateTracker(cfg);
	camera.build(cfg.getNumLevels());

	vo::ValidPointAndGradientThresholdPredicate predicate;
	boost::shared_ptr<vo::PointSelection> keyframe_points;
	boost::shared_ptr<vo::DenseTracker> odometry_tracker;
	odometry_tracker.reset(new vo::DenseTracker());
	odometry_tracker->configure(cfg);
	keyframe_points.reset(new vo::PointSelection(predicate));

	Eigen::Affine3d trajectory, relative;
	trajectory.setIdentity();

	Eigen::Affine3d initial_transformation_; initial_transformation_.setIdentity();
	vo::RgbdImagePyramid::Ptr previous_;
	{
		cout << "tracker config: " << odometry_tracker->configuration() << endl;
	}
	Timing::Timer timer;
	std::stringstream ss; std::string rgbName, depthName;
	int iFrame; vo::RgbdImagePyramid::Ptr current;
	for (iFrame = 1; iFrame < 100; iFrame++) {
		ss << folder << "rgb\\" << iFrame << ".png";
		rgbName = ss.str(); ss.str("");
		ss << folder << "depth\\" << iFrame << ".png";
		depthName = ss.str(); ss.str("");

		current = load(camera, rgbName, depthName);
		if (!current) continue;

		////////////////////////////////////////////////
		// visual odometry
		double currentTimestamp = timer.AbsoluteTime();

		current->level(0).timestamp = currentTimestamp;
		if (!previous_) {
			previous_ = current;
			trajectory = initial_transformation_;
		}
		else {
			keyframe_points->setRgbdImagePyramid(*previous_);
			vo::DenseTracker::Result r_odometry;
			r_odometry.Transformation.setIdentity();

			odometry_tracker->match(*keyframe_points, *current, r_odometry);

			trajectory = r_odometry.Transformation;
		}



		if (trajectory_out_ != nullptr) {
			Eigen::Quaterniond q(trajectory.rotation());

			(*trajectory_out_)
				<< setprecision(2) << currentTimestamp << " "
				<< setprecision(6) << trajectory.translation()(0) << " "
				<< setprecision(6) << trajectory.translation()(1) << " "
				<< setprecision(6) << trajectory.translation()(2) << " "
				<< setprecision(6) << q.x() << " "
				<< setprecision(6) << q.y() << " "
				<< setprecision(6) << q.z() << " "
				<< setprecision(6) << q.w() << " "
				<< std::endl;
		}
		std::cerr.setf(ios::fixed);
		std::cerr << "Frame " << iFrame << " at moment " << setprecision(2) << currentTimestamp << " is processed, cost " << setprecision(2) << timer.AbsoluteTime() - currentTimestamp << " seconds." << endl;
	}
	trajectory_out_->flush(); trajectory_out_->close();

	return 0;
}



vo::RgbdImagePyramidPtr load(vo::RgbdCameraPyramid& camera, std::string rgb_file, std::string depth_file)
{
	cv::Mat rgb, grey, grey_s16, depth, depth_inpainted, depth_mask, depth_mono, depth_float;

	bool rgb_available = false;
	rgb = cv::imread(rgb_file, 1);
	depth = cv::imread(depth_file, -1);

	if (rgb.total() == 0 || depth.total() == 0) return vo::RgbdImagePyramidPtr();

	if (rgb.type() != CV_32FC1)
	{
		if (rgb.type() == CV_8UC3)
		{
			cv::cvtColor(rgb, grey, CV_BGR2GRAY);
			rgb_available = true;
		}
		else
		{
			grey = rgb;
		}

		grey.convertTo(grey_s16, CV_32F);
	}
	else
	{
		grey_s16 = rgb;
	}

	if (depth.type() != CV_32FC1)
	{
		vo::SurfacePyramid::convertRawDepthImageSse(depth, depth_float, 1.0f / 1000.0f);
	}
	else
	{
		depth_float = depth;
	}


	//depth_float.setTo(dvo::core::InvalidDepth, depth_float > 1.2f);

	vo::RgbdImagePyramidPtr result = camera.create(grey_s16, depth_float);

	if (rgb_available)
		rgb.convertTo(result->level(0).rgb, CV_32FC3);

	return result;
}

void ConfigurateTracker(vo::DenseTracker::Config& cfgTracker)
{
	cfgTracker.FirstLevel = 3;
	cfgTracker.LastLevel = 0;
	cfgTracker.MaxIterationsPerLevel = 50;
	cfgTracker.Precision = 0.0001;
	cfgTracker.UseInitialEstimate = true;
	cfgTracker.UseWeighting = true;
	cfgTracker.ScaleEstimatorType = vo::ScaleEstimators::TDistribution;
	cfgTracker.ScaleEstimatorParam = 5.0;
	cfgTracker.InfluenceFuntionType = vo::InfluenceFunctions::TDistribution;
	cfgTracker.InfluenceFunctionParam = 5.0;
	cfgTracker.Mu = 0.05;
	cfgTracker.IntensityDerivativeThreshold = 0.0;
	cfgTracker.DepthDerivativeThreshold = 0.0;
}