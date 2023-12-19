#include <k4a/k4a.hpp>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "3D_AzureKinect.h"

using namespace std;
using namespace boost;
using namespace pcl;
typedef pcl::PointXYZRGBA PointType;


int main(int argc, char** argv) {

	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0) {
		cout << "no azure kinect devices detected!" << endl;
	}

	// PCL Visualizer
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	//viewer->setCameraClipDistances(0.0,1.0);
	//viewer->setCameraFieldOfView(3.14);
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);

	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;
	cv::Mat rgbframe;

	// Retrieved Point Cloud Callback Function
	std::mutex mutex;
	std::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function = [&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		std::lock_guard<std::mutex> lock(mutex);
		cloud = ptr->makeShared();
	};

	// KinectAzureDKGrabber
	std::shared_ptr<pcl::Grabber> grabber = std::make_shared<pcl::KinectAzureDKGrabber>(0, K4A_DEPTH_MODE_WFOV_2X2BINNED, K4A_IMAGE_FORMAT_COLOR_BGRA32, K4A_COLOR_RESOLUTION_720P);
	std::shared_ptr<pcl::KinectAzureDKGrabber> grabber_ = std::dynamic_pointer_cast<pcl::KinectAzureDKGrabber>(grabber);

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	// Start Grabber
	grabber->start();
	k4a::calibration calibration = grabber_->getCalibration();
	k4a_calibration_intrinsic_parameters_t* intrinsics = &calibration.color_camera_calibration.intrinsics.parameters;
	Eigen::Matrix3f intrinsics_eigen;

	intrinsics_eigen <<
		intrinsics->param.fx, 0.0f, intrinsics->param.cx,
		0.0f, intrinsics->param.fy, intrinsics->param.cy,
		0.0f, 0.0f, 1.0f;

	Eigen::Matrix4f extrinsics_eigen = Eigen::Matrix4f::Identity();
	viewer->setCameraParameters(intrinsics_eigen, extrinsics_eigen);

	k4a::capture capture;
	k4a::image colorImage;
	cv::Mat colorFrame;

	while (!viewer->wasStopped()) {

		// Update Viewer
		viewer->spinOnce();
		std::unique_lock<std::mutex> lk(mutex);
		if (lk.owns_lock() && cloud) {
			if (!viewer->updatePointCloud(cloud, "cloud")) { viewer->addPointCloud(cloud, "cloud"); }
		}

		cv::waitKey(1);
	}

	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	return 0;
}