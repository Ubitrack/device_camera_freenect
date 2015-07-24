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
using namespace openni;

// static int to initialize/deinitialize openni only once.
unsigned int FreenectModule::m_openni_initialized_count = 0;

FreenectModule::FreenectModule( const FreenectModuleKey& moduleKey, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
        : Module< FreenectModuleKey, FreenectComponentKey, FreenectModule, FreenectComponent >( moduleKey, pFactory )
		, m_device_url(m_moduleKey.get())
		, m_timeout( 2000 ) // 2000ms
        , m_bStop(false)
{
	if (m_openni_initialized_count == 0) {
		Status rc = OpenNI::initialize();
		if (rc != STATUS_OK)
		{
			LOG4CPP_ERROR( logger, "Freenect Initialize failed: " << OpenNI::getExtendedError() );
			UBITRACK_THROW( "Freenect Initialize failed" );
		}
	}
	m_openni_initialized_count++;
}

void FreenectModule::startModule() {

	// start thread immediately - it will not send images if the module is not running ..
	m_Thread.reset( new boost::thread( boost::bind( &FreenectModule::ThreadProc, this ) ) );

}

void FreenectModule::stopModule() {
	// may need a lock here ...
	if ( m_Thread )
	{
		m_bStop = true;
		m_Thread->join();
	}
}


FreenectModule::~FreenectModule()
{
	if (m_running) {
		stopModule();
	}

	m_openni_initialized_count--;

	if (m_openni_initialized_count == 0) {
		OpenNI::shutdown();
	}

}


void FreenectModule::ThreadProc()
{
	LOG4CPP_DEBUG( logger, "Freenect Thread started" );

	const char* devurl = ANY_DEVICE;
	if (!m_device_url.empty()) {
		devurl = m_device_url.c_str();
	}

	Status rc = m_device.open(devurl);
	if (rc != STATUS_OK)
	{
		LOG4CPP_ERROR( logger, "Couldn't open device: " << OpenNI::getExtendedError() );
		return;
	}


	int index = 0;
	ComponentList allComponents( getAllComponents() );
	std::vector< VideoStream* > connected_streams;
	std::vector< ComponentKey > connected_components;

	for ( ComponentList::iterator it = allComponents.begin(); it != allComponents.end(); it++ ) {
		if (m_device.getSensorInfo((*it)->getKey().getSensorType()) != NULL) {
			VideoStream* stream = new VideoStream();

			rc = stream->create(m_device, (*it)->getKey().getSensorType());
			if (rc == STATUS_OK)
			{
				rc = stream->start();
				if (rc != STATUS_OK)
				{
					LOG4CPP_ERROR( logger, "Couldn't start the stream" << std::endl << OpenNI::getExtendedError());
					delete stream;
					stream = NULL;
				} else {
					connected_streams.push_back(stream);
					connected_components.push_back((*it)->getKey());
				}
			}
			else
			{
				LOG4CPP_ERROR( logger, "Couldn't create the stream" << std::endl << OpenNI::getExtendedError());
				delete stream;
				stream = NULL;
			}

		} else {
			LOG4CPP_WARN( logger, "Device has no sensor with type: " << (*it)->getKey().getSensorType());
		}
	}

	VideoFrameRef frame;

	while ( !m_bStop )
	{
		int readyStream = -1;
		rc = OpenNI::waitForAnyStream(&connected_streams[0], connected_streams.size(), &readyStream, m_timeout);
		Ubitrack::Measurement::Timestamp timestamp = Ubitrack::Measurement::now();

		if (rc != STATUS_OK)
		{
			LOG4CPP_WARN( logger, "Wait failed! " << OpenNI::getExtendedError());
			break;
		}

		if ((readyStream >= 0) && (readyStream < connected_streams.size())) {
			connected_streams.at(readyStream)->readFrame(&frame);
			getComponent( connected_components.at(readyStream) )->processImage(timestamp, frame);
		} else {
			LOG4CPP_WARN( logger, "Unknown stream ready for reading ...");
		}
	}

	for (unsigned int i=0; i < connected_streams.size(); ++i) {
		connected_streams.at(i)->stop();
	}

	m_device.close();
	LOG4CPP_DEBUG( logger, "Freenect Thread stopped" );
}

boost::shared_ptr< FreenectModule::ComponentClass > FreenectModule::createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph, const ComponentKey& key, ModuleClass* pModule ) {
	return boost::shared_ptr< ComponentClass >( new FreenectComponent( name, subgraph, key, pModule ) );
}


FreenectComponent::FreenectComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const FreenectComponentKey& componentKey, FreenectModule* pModule )
	: FreenectModule::Component( name, componentKey, pModule )
	, m_outPort( "Output", *this )
{

}

void FreenectComponent::processImage( Measurement::Timestamp ts, const openni::VideoFrameRef& frame) {
	boost::shared_ptr< Vision::Image > pImage;

	bool new_image_data = false;
	switch (frame.getVideoMode().getPixelFormat())
	{
		case PIXEL_FORMAT_DEPTH_1_MM:
		case PIXEL_FORMAT_DEPTH_100_UM:
		case PIXEL_FORMAT_GRAY16:
			pImage.reset(new Vision::Image(frame.getWidth(), frame.getHeight(), 1, IPL_DEPTH_16U));
			pImage->origin = 0;
			memcpy(pImage->imageData, (unsigned char*)frame.getData(), frame.getWidth() * frame.getHeight() * 1);
			new_image_data = true;
			break;


		case PIXEL_FORMAT_RGB888:
			pImage.reset(new Vision::Image(frame.getWidth(), frame.getHeight(), 3, IPL_DEPTH_8U));
			pImage->origin = 0;
			pImage->channelSeq[0] = 'R';
			pImage->channelSeq[1] = 'G';
			pImage->channelSeq[2] = 'B';
			memcpy(pImage->imageData, (unsigned char*)frame.getData(), frame.getWidth() * frame.getHeight() * 3);
			new_image_data = true;
			break;
		default:
			LOG4CPP_WARN( logger, "Unknown pixel format: " << frame.getVideoMode().getPixelFormat());
	}

	// undistort and process here ..

	if (new_image_data) {
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
