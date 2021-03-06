/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Synchronouse capture of camera images using Point Grey's FlyCapture2 library.
 *
 * @author Ulrich Eck <ueck@net-labs.de>
 *
 */

#include "FreenectFrameGrabber.h"
#include <utMeasurement/Timestamp.h>
#include <utUtil/TracingProvider.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <utDataflow/ComponentFactory.h>
#include <utUtil/OS.h>
#include <boost/array.hpp>

#include <log4cpp/Category.hh>

namespace Ubitrack { namespace Drivers {
// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.FreenectFrameGrabber" ) );


using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Drivers;
using namespace freenect_camera;

FreenectModule::FreenectModule( const FreenectModuleKey& moduleKey, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
        : Module< FreenectModuleKey, FreenectComponentKey, FreenectModule, FreenectComponent >( moduleKey, pFactory )
		, m_device_id(m_moduleKey.get())
        , m_bStop(false)
		, m_autoGPUUpload(false)
{
	freenect_init(&m_driver, NULL);
	//freenect_set_log_level(m_driver, FREENECT_LOG_FATAL); // Prevent's printing stuff to the screen
	freenect_set_log_level(m_driver, FREENECT_LOG_DEBUG); // Prevent's printing stuff to the screen
	freenect_select_subdevices(m_driver, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
	if (oclManager.isEnabled()) {
		if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
			m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
			LOG4CPP_INFO(logger, "Upload to GPU enabled? " << m_autoGPUUpload);
		}
		if (m_autoGPUUpload){
			oclManager.activate();
			LOG4CPP_INFO(logger, "Require OpenCLManager");
		}
	}


}

void FreenectModule::startModule() {

	m_device_serials.clear();
	freenect_device_attributes* attr_list;
	freenect_device_attributes* item;
	freenect_list_device_attributes(m_driver, &attr_list);
	for (item = attr_list; item != NULL; item = item->next) {
		LOG4CPP_INFO( logger, "Found freenect device with serial: " << std::string(item->camera_serial) );
		m_device_serials.push_back(std::string(item->camera_serial));
	}
	freenect_free_device_attributes(attr_list);

	if (m_device_serials.size() < 1) {
		LOG4CPP_ERROR( logger, "No devices found.");
		return;
	}
	if (m_device_id.empty())
		m_device_id = m_device_serials.at(0);

	m_device.reset(new FreenectDevice(m_driver, m_device_id));

	// check that steam only contains either IR or RGB nodes ..

	ComponentList allComponents( getAllComponents() );
	for ( ComponentList::iterator it = allComponents.begin(); it != allComponents.end(); it++ ) {
		(*it)->configureStream(m_device);
		switch ((*it)->getKey().getSensorType()) {
			case SENSOR_IR:
				m_device->registerIRCallback(&FreenectModule::irCb, *this );
				LOG4CPP_INFO( logger, "registered IR callback");
				if (!m_device->isIRStreamRunning())
					m_device->startIRStream();
				break;
			case SENSOR_RGB:
				m_device->registerImageCallback(&FreenectModule::rgbCb, *this );
				LOG4CPP_INFO( logger, "registered RGB callback");
				if (!m_device->isImageStreamRunning())
					m_device->startImageStream();
				break;
			case SENSOR_DEPTH:
				m_device->registerDepthCallback(&FreenectModule::depthCb, *this );
				LOG4CPP_INFO( logger, "registered DEPTH callback");
				if (!m_device->isDepthStreamRunning())
					m_device->startDepthStream();
				break;
			default:
				LOG4CPP_WARN( logger, "Device has no sensor with type: " << (*it)->getKey().getSensorType());
				break;
		}
	}


	// start thread immediately - it will not send images if the module is not running ..
	m_Thread.reset( new boost::thread( boost::bind( &FreenectModule::ThreadProc, this ) ) );

}

void FreenectModule::stopModule() {

	if (m_device) {
		if (m_device->isDepthStreamRunning()) {
			m_device->stopDepthStream();
		}
		if (m_device->isImageStreamRunning()) {
			m_device->stopImageStream();
		}
		if (m_device->isIRStreamRunning()) {
			m_device->stopIRStream();
		}
	}

	// may need a lock here ...
	if ( m_Thread )
	{
		m_bStop = true;
		m_Thread->join();
	}

	if (m_device)
		m_device->shutdown();
	m_device.reset();

}


FreenectModule::~FreenectModule()
{
	if (m_running) {
		stopModule();
	}
	freenect_shutdown(m_driver);
}


void FreenectModule::ThreadProc()
{
	LOG4CPP_DEBUG( logger, "Freenect Thread started" );

	while ( !m_bStop )
	{

		timeval t;
		t.tv_sec = 0;
		t.tv_usec = 10000;
		if (freenect_process_events_timeout(m_driver, &t) < 0)
			UBITRACK_THROW("freenect_process_events error");
		if (m_device)
			m_device->executeChanges();
	}

	LOG4CPP_DEBUG( logger, "Freenect Thread stopped" );
}

void FreenectModule::rgbCb(const ImageBuffer& image, void* cookie) {
	const ComponentKey key(SENSOR_RGB);
	if (hasComponent( key )) {
		getComponent( key )->imageCb(image);
	}
}

void FreenectModule::irCb(const ImageBuffer& image, void* cookie) {
	const ComponentKey key(SENSOR_IR);
	if (hasComponent( key )) {
		getComponent( key )->imageCb(image);
	}
}

void FreenectModule::depthCb(const ImageBuffer& image, void* cookie) {
	const ComponentKey key(SENSOR_DEPTH);
	if (hasComponent( key )) {
		getComponent( key )->imageCb(image);
	}
}

boost::shared_ptr< FreenectModule::ComponentClass > FreenectModule::createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph, const ComponentKey& key, ModuleClass* pModule ) {
	return boost::shared_ptr< ComponentClass >( new FreenectComponent( name, subgraph, key, pModule ) );
}


FreenectComponent::FreenectComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const FreenectComponentKey& componentKey, FreenectModule* pModule )
	: FreenectModule::Component( name, componentKey, pModule )
	, m_outPort( "Output", *this )
{
	switch(componentKey.getSensorType()) {
		case SENSOR_IR:
			subgraph->m_DataflowAttributes.getAttributeData( "videoModeIR", m_stream_mode );
			break;
		case SENSOR_RGB:
			subgraph->m_DataflowAttributes.getAttributeData( "videoModeRGB", m_stream_mode );
			break;
		case SENSOR_DEPTH:
			subgraph->m_DataflowAttributes.getAttributeData( "videoModeDEPTH", m_stream_mode );
			break;
		default:
			// never gets here ..
			break;
	}
}

void FreenectComponent::configureStream(const boost::shared_ptr<freenect_camera::FreenectDevice> &device) {
	switch(getKey().getSensorType()) {
		case SENSOR_IR:
			// currently no settings, defaults to IR_8BIT
			break;
		case SENSOR_RGB:
			// currently no settings, defaults to RGB
			break;
		case SENSOR_DEPTH:
			// currently no settings, defaults to 11BIT
			break;
		default:
			// never gets here ..
			break;
	}
}

void FreenectComponent::imageCb( const freenect_camera::ImageBuffer& image) {

	Ubitrack::Measurement::Timestamp ts = Ubitrack::Measurement::now();
	boost::shared_ptr< Vision::Image > pImage;

#ifdef ENABLE_EVENT_TRACING
	TRACEPOINT_MEASUREMENT_CREATE(getEventDomain(), ts, getName().c_str(), "VideoCapture")
#endif

	int width = image.metadata.width;
	int height = image.metadata.height;

	LOG4CPP_DEBUG( logger, "Image Callback Sensor: " << getKey().getSensorType() << " size: " << width << "x" << height);

	bool new_image_data = false;
	switch (getKey().getSensorType())
	{
		case SENSOR_IR:
			if (image.metadata.video_format == FREENECT_VIDEO_IR_8BIT) {
				pImage.reset(new Vision::Image(width, height, 1, IPL_DEPTH_8U));
				pImage->set_origin(0);
				pImage->set_pixelFormat(Vision::Image::LUMINANCE);
				pImage->set_bitsPerPixel(8);
//				freenect_camera::fill(image, pImage->imageData);
				memcpy(pImage->Mat().data, (unsigned char*)image.image_buffer.get(), image.metadata.bytes);
				new_image_data = true;

			} else  if (image.metadata.video_format == FREENECT_VIDEO_IR_10BIT) {
				pImage.reset(new Vision::Image(width, height, 1, IPL_DEPTH_16U));
				pImage->set_origin(0);
				pImage->set_pixelFormat(Vision::Image::LUMINANCE);
				pImage->set_bitsPerPixel(16);
//				freenect_camera::fill(image, pImage->imageData);
				memcpy(pImage->Mat().data, (unsigned char *) image.image_buffer.get(), image.metadata.bytes);
				new_image_data = true;

			} else {
				LOG4CPP_WARN( logger, "Unsupported IR Videomode: " << image.metadata.video_format );
			}
			break;
		case SENSOR_RGB:
			if (image.metadata.video_format == FREENECT_VIDEO_RGB) {
				pImage.reset(new Vision::Image(width, height, 3, IPL_DEPTH_8U));
				pImage->set_origin(0);
				pImage->set_pixelFormat(Vision::Image::RGB);
				pImage->set_bitsPerPixel(24);
//				freenect_camera::fill(image, pImage->imageData);
				memcpy(pImage->Mat().data, (unsigned char*)image.image_buffer.get(), image.metadata.bytes);
				new_image_data = true;

			} else {
				LOG4CPP_WARN( logger, "Unsupported RGB Videomode: " << image.metadata.video_format );
			}
			break;

		case SENSOR_DEPTH:
			if ((image.metadata.depth_format == FREENECT_DEPTH_11BIT) || (image.metadata.depth_format == FREENECT_DEPTH_MM)) {
				pImage.reset(new Vision::Image(width, height, 1, IPL_DEPTH_16U));
				pImage->set_origin(0);
				pImage->set_pixelFormat(Vision::Image::DEPTH);
//				freenect_camera::fill(image, pImage->imageData);
				memcpy(pImage->Mat().data, (unsigned char*)image.image_buffer.get(), image.metadata.bytes);
				new_image_data = true;

			} else {
				LOG4CPP_WARN( logger, "Unsupported DEPTH Videomode: " << image.metadata.video_format );
			}
			break;

		default:
			// should never get here ..
			break;
	}

	if (new_image_data) {

		if (getModule().autoGPUEnabled()){
			Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
			if (oclManager.isInitialized()) {
				//force upload to the GPU
				pImage->uMat();
			}
		}

		// undistortion should be added here ..

		m_outPort.send( Measurement::ImageMeasurement( ts, pImage ) );

	}

}

std::ostream& operator<<( std::ostream& s, const FreenectComponentKey& k )
{
	s << "FreenectComponent[ " << k.getSensorType()  << " ]";
	return s;
}

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerModule< FreenectModule > ( "FreenectFrameGrabber" );
}

} } // namespace Ubitrack::Drivers
