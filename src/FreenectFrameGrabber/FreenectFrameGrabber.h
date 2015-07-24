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
 * synchronously capture camera images using freenect.
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

#include "freenect_device.hpp"



namespace {
	
	typedef enum  {
		SENSOR_IR = 0,
		SENSOR_RGB = 1,
		SENSOR_DEPTH = 2,
	} SensorType;
	
	
	class FreenectSensorMap
		: public std::map< std::string, SensorType>
	{
	public:
		FreenectSensorMap()
		{
			
			(*this)[ "IR" ] = SENSOR_IR;
			(*this)[ "COLOR" ] = SENSOR_RGB;
			(*this)[ "DEPTH" ] = SENSOR_DEPTH;
		}
	};
	static FreenectSensorMap freenectSensorMap;

	class FreenectColorPixelFormatMap
		: public std::map< std::string, freenect_video_format >
	{
	public:
		FreenectColorPixelFormatMap()
		{
			(*this)[ "RGB" ] = FREENECT_VIDEO_RGB;
			(*this)[ "BAYER" ] = FREENECT_VIDEO_BAYER;
			(*this)[ "IR_8BIT" ] = FREENECT_VIDEO_IR_8BIT;
			(*this)[ "IR_10BIT" ] = FREENECT_VIDEO_IR_10BIT;
			(*this)[ "IR_10BIT_PACKED" ] = FREENECT_VIDEO_IR_10BIT_PACKED;
			(*this)[ "YUV_RGB" ] = FREENECT_VIDEO_YUV_RGB;
			(*this)[ "YUV_RAW" ] = FREENECT_VIDEO_YUV_RAW;
		}
	};
	static FreenectColorPixelFormatMap freenectColorPixelFormatMap;

	class FreenectDepthPixelFormatMap
			: public std::map< std::string, freenect_depth_format >
	{
	public:
		FreenectDepthPixelFormatMap()
		{
			(*this)[ "11BIT" ] = FREENECT_DEPTH_11BIT;
			(*this)[ "10BIT" ] = FREENECT_DEPTH_10BIT;
			(*this)[ "11BIT_PACKED" ] = FREENECT_DEPTH_11BIT_PACKED;
			(*this)[ "10BIT_PACKED" ] = FREENECT_DEPTH_10BIT_PACKED;
			(*this)[ "REGISTERED" ] = FREENECT_DEPTH_REGISTERED;
			(*this)[ "MM" ] = FREENECT_DEPTH_MM;
		}
	};
	static FreenectDepthPixelFormatMap freenectDepthPixelFormatMap;

} // anonymous namespace



namespace Ubitrack { namespace Drivers {
using namespace Dataflow;

// forward declaration
class FreenectComponent;

MAKE_NODEATTRIBUTEKEY_DEFAULT( FreenectModuleKey, std::string, "Camera", "deviceSerial", "" );

/**
 * Component key for freenect.
 * Represents the camera
 */
class FreenectComponentKey
{
public:

	FreenectComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: m_sensor_type( freenect::SENSOR_DEPTH )
	{

		std::string sSensorType = subgraph->m_DataflowAttributes.getAttributeString( "sensorType" );
		if ( freenectSensorMap.find( sSensorType ) == freenectSensorMap.end() )
			UBITRACK_THROW( "unknown sensor type: \"" + sSensorType + "\"" );
		m_sensor_type = freenectSensorMap[ sSensorType ];
	}

	// construct from sensor type
	FreenectComponentKey( const freenect::SensorType a )
		: m_sensor_type( a )
 	{}
	
	freenect::SensorType  getSensorType() const {
		return m_sensor_type;
	}

	// less than operator for map
	bool operator<( const FreenectComponentKey& b ) const
    {
		return m_sensor_type < b.m_sensor_type;
    }

protected:
	freenect::SensorType m_sensor_type;
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

	std::string m_device_id;

	/** the device **/
	freenect_context* m_driver;
	std::vector<std::string> m_device_serials;
	boost::shared_ptr<freenect_camera::FreenectDevice> m_device;
	
	/** create the components **/
	boost::shared_ptr< ComponentClass > createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
														 const ComponentKey& key, ModuleClass* pModule );

};

std::ostream& operator<<( std::ostream& s, const FreenectComponentKey& k );

/**
 * Component for Freenect tracker.
 */
class FreenectComponent : public FreenectModule::Component {
public:
	/** constructor */
	FreenectComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const FreenectComponentKey& componentKey, FreenectModule* pModule );

	void imageCb( const freenect_camera::ImageBuffer& image, void* cookie);

	/** destructor */
	~FreenectComponent() {};


protected:


	// the port
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;
};

} } // namespace Ubitrack::Drivers

#endif
