#ifdef SendMessage
#undef SendMessage
#endif

#include <RobotRaconteur.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include "edu__rpi__cats__sensors__kinect2.h"
#include "edu__rpi__cats__sensors__kinect2_stubskel.h"

#include <Windows.h>
#include <Kinect.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <boost/enable_shared_from_this.hpp>
#include <map>

#pragma once

class Kinect2_impl : public edu::rpi::cats::sensors::kinect2::Kinect, public boost::enable_shared_from_this < Kinect2_impl >
{
public:

	Kinect2_impl();
	~Kinect2_impl();

	HRESULT StartupKinect();
	HRESULT ShutdownKinect();

	virtual void StartStreaming();
	virtual void StopStreaming();

	virtual uint8_t EnableSensors(RR_SHARED_PTR<edu::rpi::cats::sensors::kinect2::KinectMultiSource > s);
	virtual uint8_t DisableSensors();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::kinect2::KinectMultiSource > SensorsEnabled();

	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > getImageHeader();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > getDepthImageHeader();

	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > getCurrentImage();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image16 > getCurrentDepthImage();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image16 > getCurrentInfraredImage();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > getCurrentBodyIndexImage();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image16 > getCurrentLongExposureInfraredImage();

	virtual RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > get_ImageStream();
	virtual void set_ImageStream(RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > value);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint64_t > > getTrackedBodyIDs();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::kinect2::KinectBody > getDetectedBody(int32_t index);


private:
	uint64_t tracked_body_ids[6];
	IBody *kinect_bodies[BODY_COUNT];
	RGBQUAD *color_image_data;
	uint8_t *bodyindex_image_data;
	uint16_t *depth_image_data;
	uint16_t *infrared_image_data;
	uint16_t *longexposure_infrared_image_data;
	DWORD enabledSources;

	int color_image_width, color_image_height;
	int depth_image_width, depth_image_height;

	RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > image_pipe;

	IKinectSensor *kinect;
	IMultiSourceFrameReader *multi_reader;
	WAITABLE_HANDLE h_event;
	boost::mutex mtx_;
	boost::thread t1;


	void MultiSourceFrameArrived(IMultiSourceFrameArrivedEventArgs* pArgs);

	void backgroundPollingThread();

	template<class Interface> inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};