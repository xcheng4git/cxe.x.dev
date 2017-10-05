#include "stdafx.h"
#include <iostream>
using namespace std;
#include "Timer.h"

#include <DenseVO\IntrinsicMatrix.h>
#include <DenseVO\RgbdImagePyramid.h>
#include <DenseVO\DenseTracker.h>
#include <DenseVO\SurfacePyramid.h>
#include <KeyframeSLAM\Config.h>
#include <KeyframeSLAM\KeyframeTracker.h>
#include <KeyframeSLAM\MapSerializer.h>

#include <boost/bind.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/bilateral.h>//required  
#include <pcl/filters/fast_bilateral.h>  
#include <pcl/filters/fast_bilateral_omp.h>  
#include <pcl/common/transforms.h> 

class Config
{
public:

	bool EstimateTrajectory;
	std::string TrajectoryFile;
	bool RenderVideo;
	std::string VideoFolder;

	std::string CameraFile;

	std::string RgbdPairFile;
	std::string GroundtruthFile;

	bool ShowGroundtruth;
	bool ShowEstimate;

	bool KeepAlive;

	bool EstimateRequired()
	{
		return EstimateTrajectory || ShowEstimate;
	}
	bool VisualizationRequired()
	{
		return ShowGroundtruth || ShowEstimate || RenderVideo;
	}
};

ostream *trajectory_out_;
bool configure(Config& config);

vo::RgbdImagePyramidPtr load(vo::RgbdCameraPyramid& camera, std::string rgb_file, std::string depth_file);

void onMapChanged(slam::KeyframeGraph& map);
void CreateScenePointCloud(const std::string trajectoryFile, const std::string dataFolder);
int main()
{
	Config cfg_;

	if (!configure(cfg_)) {
		std::cerr << "Error on configuration." << std::endl;
		getchar();
		return -1;
	}
	std::string optimized_trajectory_file = cfg_.RgbdPairFile.substr(0, cfg_.RgbdPairFile.find_last_of(".")) + "_opt_traj_final.txt";
	std::string edge_error_file = cfg_.RgbdPairFile.substr(0, cfg_.RgbdPairFile.find_last_of(".")) + "_error.txt";
	std::string folder = cfg_.RgbdPairFile.substr(0, cfg_.RgbdPairFile.find_last_of("\\") + 1);

	//vo::IntrinsicMatrix intrinsics = vo::IntrinsicMatrix::create(544.47329, 544.47329, 320, 240);
	vo::IntrinsicMatrix intrinsics = vo::IntrinsicMatrix::create(518.0, 519.0, 325.5, 253.5);
	vo::RgbdCameraPyramid camera(640, 480, intrinsics);

	slam::CameraDenseTrackerConfig cfgCamera;
	vo::DenseTracker::Config cfg = vo::DenseTracker::getDefaultConfig();
	slam::UpdateTrackerConfig(cfgCamera, cfg);

	camera.build(cfg.getNumLevels());

	slam::KeyframeTrackerConfig cfgFrontend;
	slam::KeyframeGraphConfig cfgBackend;

	slam::KeyframeTracker trackerKeyframe;
	trackerKeyframe.configureTracking(cfg);
	trackerKeyframe.configureKeyframeSelection(cfgFrontend);
	trackerKeyframe.configureMapping(cfgBackend);

	{
		cout << "tracker config: " << trackerKeyframe.trackingConfiguration() << endl;
		cout << "frontend config: " << trackerKeyframe.keyframeSelectionConfiguration() << endl;
		cout << "backend config: " << trackerKeyframe.mappingConfiguration() << endl;
	}

	
	trackerKeyframe.addMapChangedCallback(boost::bind(onMapChanged, _1));

	Eigen::Affine3d trajectory, relative;
	trajectory.setIdentity();
	trackerKeyframe.init(trajectory);


	Timing::Timer timer;
	vo::RgbdImagePyramid::Ptr current;
	std::stringstream ss; std::string rgbName, depthName;
	int iFrame; 
	for (iFrame = 1; iFrame < 20; iFrame++) {
		ss << folder << "rgb\\" << iFrame << ".png";
		rgbName = ss.str(); ss.str("");
		ss << folder << "depth\\" << iFrame << ".png";
		depthName = ss.str(); ss.str("");
		current = load(camera, rgbName, depthName);
		if (!current) continue;

		////////////////////////////////////////////////
		// visual odometry
		double currentTimestamp = timer.AbsoluteTime();

		if (iFrame == 99) {
			std::cerr << "Forcing keyframe." << endl;
			trackerKeyframe.forceKeyframe();
		}

		trackerKeyframe.update(current, currentTimestamp, trajectory);

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


	slam::serialization::FileSerializer<slam::serialization::TrajectorySerializer> serializer(optimized_trajectory_file);
	trackerKeyframe.serializeMap(serializer);

	slam::serialization::FileSerializer<slam::serialization::EdgeErrorSerializer> error_serializer(edge_error_file);
	trackerKeyframe.serializeMap(error_serializer);
	
	CreateScenePointCloud(optimized_trajectory_file, folder);
	return 0;
}


bool configure(Config& config)
{
	//std::string folder = "e:\\datasets\\slam\\rgbd_dataset_freiburg1_xyz\\";
	//
	//config.RgbdPairFile = folder + "assoc.txt";
	//readerRgbdPair_ = new FileReader<RgbdPair>(config.RgbdPairFile);
	//readerRgbdPair_->skipComments();
	//if (!readerRgbdPair_->next()) {
	//	std::cerr << "Failed to open '" << config.RgbdPairFile << "'!" << std::endl;
	//	return false;
	//}
	std::string folder = "E:\\datasets\\slam\\rgbd_dataset_caffe\\";
	config.RgbdPairFile = folder + "assoc.txt";

	config.EstimateTrajectory = true;
	if (config.EstimateTrajectory) {
		config.TrajectoryFile = folder + "output\\trajectory.txt";
		trajectory_out_ = new std::ofstream(config.TrajectoryFile.c_str());
		if (trajectory_out_->fail()) {
			delete trajectory_out_;
			std::cerr << "Failed to open '" << config.TrajectoryFile << "'!" << std::endl;
			return false;
		}
	}
	else {
		trajectory_out_ = &std::cout;
	}
	trajectory_out_->setf(ios::fixed);

	config.RenderVideo = false;

	config.CameraFile = folder + "output\\camera.txt";

	config.ShowGroundtruth = false;

	config.ShowEstimate = true;

	config.KeepAlive = false;

	return true;
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

void onMapChanged(slam::KeyframeGraph& map)
{
	std::cerr << "Map changed..." << std::endl;

	map.graph().save("map.g2o");
}


void CreateScenePointCloud(const std::string trajectoryFile, const std::string dataFoler)
{
	// 相机内参
	const double camera_factor = 1000;
	const double camera_cx = 325.5;
	const double camera_cy = 253.5;
	const double camera_fx = 518.0;
	const double camera_fy = 519.0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	ifstream inTraj; inTraj.open(trajectoryFile);
	std::vector<Eigen::Isometry3d> trajectory;
	double tick, tx, ty, tz, qx, qy, qz, qw;
	
	std::stringstream ss; 
	int iFrame = 2; cv::Mat rgb, depth;
	while (!inTraj.eof()) {
		cout << "frame " << setfill('0') << setw(3) << iFrame << endl;
		//cout << "frame " << iFrame << endl;
		inTraj >> tick >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
		Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); ;
		Eigen::Quaterniond q = Eigen::Quaterniond(qw, qx, qy, qz);
		pose.rotate(q.matrix());
		pose.pretranslate(Eigen::Vector3d(tx, ty, tz));
		//cout << tx << ", " << ty << ", " << tz << ", " << qx << ", " << qy << ", " << qz << ", " << qw << endl;
		//cout << "q.coeffs=" << q.coeffs() << endl;
		//cout << "pose transformation is: " << endl << pose.matrix() << endl; getchar();

		ss << dataFoler << "rgb\\" << iFrame << ".png";
		rgb = cv::imread(ss.str()); ss.str("");
		ss << dataFoler << "depth\\" << iFrame << ".png";
		depth = cv::imread(ss.str(), -1); ss.str("");

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		int height = rgb.rows, width = rgb.cols;
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				ushort d = depth.ptr<ushort>(y)[x];
				if (d == 0) continue;

				pcl::PointXYZRGB p;

				// 计算这个点的空间坐标
				p.z = double(d) / camera_factor;
				p.x = (x - camera_cx) * p.z / camera_fx;
				p.y = (y - camera_cy) * p.z / camera_fy;

				//cv::Vec3b bgr = rgb.at<cv::Vec3b>(y, x);
				p.r = rgb.at<cv::Vec3b>(y, x)[2];
				p.g = rgb.at<cv::Vec3b>(y, x)[1];
				p.b = rgb.at<cv::Vec3b>(y, x)[0];

				frameCloud->push_back(p);
			}
		}
		rgb.release(); depth.release();
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(frameCloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0, 8.0);
		pass.filter(*frameCloud);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*frameCloud, *transformed_cloud, pose.matrix());

		*sceneCloud += *transformed_cloud;
		pcl::VoxelGrid<pcl::PointXYZRGB> grid;
		grid.setInputCloud(sceneCloud);
		grid.setLeafSize(0.01, 0.01, 0.01);
		grid.filter(*sceneCloud);

		if (0 == (iFrame % 100)) {
			ss << dataFoler << "scene_" << setfill('0') << setw(3) << iFrame << ".pcd";
			pcl::io::savePCDFile(ss.str(), *sceneCloud, true); ss.str("");
		}
		iFrame++;
	}
	inTraj.close();

}