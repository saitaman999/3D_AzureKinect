#pragma once

#ifndef KINECTDLL_H
#define KINECTDLL_H

#ifdef KINECTDLL_EXPORTS
#define KINECTDLL_API __declspec(dllexport)
#else
#define KINECTDLL_API __declspec(dllimport)
#endif

#include <k4a/k4a.hpp>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/signals2/signal.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>

namespace pcl {

	struct pcl::PointXYZ;
	struct pcl::PointXYZI;
	struct pcl::PointXYZRGB;
	struct pcl::PointXYZRGBA;
	template <typename T> class pcl::PointCloud;


	extern "C" {

		class KINECTDLL_API Ckinectdll {
		public:
			Ckinectdll(void);
		};

		class  KINECTDLL_API KinectAzureDKGrabber : public pcl::Grabber {

		public:
			KinectAzureDKGrabber(const int& device_id_, const int& depth_mode_, const int& color_format_, const int& color_resolution_);
			virtual ~KinectAzureDKGrabber() throw ();
			virtual void start();
			virtual void stop();
			virtual bool isRunning() const;
			virtual std::string getName() const;
			virtual float getFramesPerSecond() const;
			typedef void (signal_KinectAzureDK_PointXYZ)(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>&);
			typedef void (signal_KinectAzureDK_PointXYZI)(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>&);
			typedef void (signal_KinectAzureDK_PointXYZRGB)(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>&);
			typedef void (signal_KinectAzureDK_PointXYZRGBA)(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>&);


		private:
			static int Kinect_mainTheardCallBack(void* Kinectparm);

		protected:
			void setupDevice(const int& device_id_, const int& depth_mode_, const int& color_format_, const int& color_resolution_);
			boost::signals2::signal<signal_KinectAzureDK_PointXYZ>* signal_PointXYZ;
			boost::signals2::signal<signal_KinectAzureDK_PointXYZI>* signal_PointXYZI;
			boost::signals2::signal<signal_KinectAzureDK_PointXYZRGB>* signal_PointXYZRGB;
			boost::signals2::signal<signal_KinectAzureDK_PointXYZRGBA>* signal_PointXYZRGBA;
			pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ();
			pcl::PointCloud<pcl::PointXYZI>::Ptr convertInfraredDepthToPointXYZI();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB();
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA();
			std::thread thread;
			mutable std::mutex mutex;
			void threadFunction();

			bool quit;
			bool running;
			k4a_device_configuration_t config;
			k4a::device dev;
			int device_id;
			k4a::calibration calibration;
			k4a::transformation transformation;
			int colorWidth;
			int colorHeight;
			k4a::image colorImage;
			int depthWidth;
			int depthHeight;
			k4a::image depthImage;
			int infraredWidth;
			int infraredHeight;
			k4a::image infraredImage;
			uint8_t* colorTextureBuffer;
			cv::Mat colorFrame;
		public:
			k4a::calibration getCalibration();
			void share_IMG(cv::Mat& share_img);
		};

		extern KINECTDLL_API int nkinectdll;
		KINECTDLL_API int fnkinectdll(void);

	};
}

#endif