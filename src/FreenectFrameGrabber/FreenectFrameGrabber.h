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
 * @ingroup driver_components
 * @file
 * Freenect driver
 * This file contains the driver component to
 * synchronously capture camera images using openni2.
 *
 * The received data is sent via a push interface.
 *
 * @author Ulrich Eck <ueck@net-labs.de>
 */

#ifndef __Freenect_h_INCLUDED__
#define __Freenect_h_INCLUDED__


#include <string>
#include <cstdlib>

#include <iostream>
#include <map>
#include <boost/array.hpp>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/Module.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>

#include <utVision/Image.h>
#include <opencv/cv.h>

#include <OpenNI.h>




namespace {
	class FreenectSensorMap
		: public std::map< std::string, openni::SensorType>
	{
	public:
		FreenectSensorMap()
		{
			using namespace openni;
			
			(*this)[ "IR" ] = SENSOR_IR;
			(*this)[ "COLOR" ] = SENSOR_COLOR;
			(*this)[ "DEPTH" ] = SENSOR_DEPTH;
		}
	};
	static FreenectSensorMap openni2SensorMap;

	class FreenectPixelFormatMap 
		: public std::map< std::string, openni::PixelFormat >
	{
	public:
		FreenectPixelFormatMap()
		{
			using namespace openni;
			
			(*this)[ "DEPTH_1_MM" ] = PIXEL_FORMAT_DEPTH_1_MM;
			(*this)[ "DEPTH_100_UM" ] = PIXEL_FORMAT_DEPTH_100_UM;
			(*this)[ "SHIFT_9_2" ] = PIXEL_FORMAT_SHIFT_9_2;
			(*this)[ "SHIFT_9_3" ] = PIXEL_FORMAT_SHIFT_9_3;
			(*this)[ "RGB8" ] = PIXEL_FORMAT_RGB888;
			(*this)[ "YUV422" ] = PIXEL_FORMAT_YUV422;
			(*this)[ "GRAY8" ] = PIXEL_FORMAT_GRAY8;
			(*this)[ "GRAY16" ] = PIXEL_FORMAT_GRAY16;
			(*this)[ "JPEG" ] = PIXEL_FORMAT_JPEG;
			(*this)[ "YUV" ] = PIXEL_FORMAT_YUYV;
		}
	};
	static FreenectPixelFormatMap openni2PixelFormatMap;

} // anonymous namespace



namespace Ubitrack { namespace Drivers {
using namespace Dataflow;

// forward declaration
class FreenectComponent;

MAKE_NODEATTRIBUTEKEY_DEFAULT( FreenectModuleKey, std::string, "Camera", "deviceUrl", "" );

/**
 * Component key for openni2.
 * Represents the camera
 */
class FreenectComponentKey
{
public:

	FreenectComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: m_sensor_type( openni::SENSOR_DEPTH )
	{

		std::string sSensorType = subgraph->m_DataflowAttributes.getAttributeString( "sensorType" );
		if ( openni2SensorMap.find( sSensorType ) == openni2SensorMap.end() )
			UBITRACK_THROW( "unknown sensor type: \"" + sSensorType + "\"" );
		m_sensor_type = openni2SensorMap[ sSensorType ];
	}

	// construct from sensor type
	FreenectComponentKey( const openni::SensorType a )
		: m_sensor_type( a )
 	{}
	
	openni::SensorType  getSensorType() const {
		return m_sensor_type;
	}

	// less than operator for map
	bool operator<( const FreenectComponentKey& b ) const
    {
		return m_sensor_type < b.m_sensor_type;
    }

protected:
	openni::SensorType m_sensor_type;
};



/**
 * Module for Freenect tracker.
 * Does all the work
 */
class FreenectModule
	: public Module< FreenectModuleKey, FreenectComponentKey, FreenectModule, FreenectComponent >
{
public:
	/** UTQL constructor */
	FreenectModule( const FreenectModuleKey& key, boost::shared_ptr< Graph::UTQLSubgraph >, FactoryHelper* pFactory );

	/** destructor */
	~FreenectModule();

	virtual void startModule();

	virtual void stopModule();

protected:

	// thread main loop
	void ThreadProc();

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	std::string m_device_url;

	/** the device **/
	openni::Device m_device;

	/** create the components **/
	boost::shared_ptr< ComponentClass > createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
														 const ComponentKey& key, ModuleClass* pModule );

	unsigned int m_timeout;
	static unsigned int m_openni_initialized_count;
};

std::ostream& operator<<( std::ostream& s, const FreenectComponentKey& k );

/**
 * Component for Freenect tracker.
 */
class FreenectComponent : public FreenectModule::Component {
public:
	/** constructor */
	FreenectComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const FreenectComponentKey& componentKey, FreenectModule* pModule );

	virtual void processImage( Measurement::Timestamp ts, const openni::VideoFrameRef& frame);

	/** destructor */
	~FreenectComponent() {};


protected:


	// the port
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;
};

} } // namespace Ubitrack::Drivers

#endif
