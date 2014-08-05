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
 * Ubitrack Module for Leap Motion 
 *
 * @author  <pankratz@in.tum.de>, <yuta.itoh@in.tum.de>
 */
#ifndef __LEAP_MOTION_H_INCLUDED__
#define __LEAP_MOTION_H_INCLUDED__

#include <string>
#include <cstdlib>


#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/Module.h>
#include <utMeasurement/Measurement.h>
#include <Leap.h>

namespace Ubitrack { namespace Drivers {

using namespace Dataflow;

// forward declaration
class LeapMotionModuleComponent;


/**
 * Module key
 */
MAKE_NODEIDKEY( LeapMotionKey, "LeapMotion" );

/**
 * Component key
 * Represents what to track
 */
class LeapMotionComponentKey
{
	public:

		/// set priority based on dataflow class as stored in the subgraph by the dataflow network
		LeapMotionComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )			
		{
			
			Graph::UTQLSubgraph::EdgePtr config;

			  if ( subgraph->hasEdge( "LeapToTarget" ) )
				  config = subgraph->getEdge( "LeapToTarget" );	  

			  if ( !config )
			  {
				  UBITRACK_THROW( "LeapMotion Pattern has no \"LeapToTarget\" edge");
			  }

			  config->getAttributeData( "targetID", m_targetID );
			  if ( m_targetID < 0 )
					UBITRACK_THROW( "Missing or invalid \"m_targetID\" attribute on \"LeapToTarget\" edge" );
		
			std::string& dfclass = subgraph->m_DataflowClass;
			
			m_targetClass = 0;
			if ( dfclass == "LeapMotionHand"       ) {
				// do something
				m_targetClass = 1;
			};
			if ( dfclass == "LeapMotionTool"       ) {
				// do something
				m_targetClass = 2;
			}
		}

		LeapMotionComponentKey( int target, int tagetclass) 
			: m_targetID(target)
			, m_targetClass(tagetclass)
		{

		}

		
		bool operator<( const LeapMotionComponentKey& b ) const
		{
			if (m_targetClass == b.m_targetClass) return (m_targetID < b.m_targetID);
			return (m_targetClass < b.m_targetClass);
		}

		int m_targetID;
		int m_targetClass;
};

std::ostream& operator<<( std::ostream& s, const LeapMotionComponentKey& k );

/**
 * Test Module 
 * Does all the work
 */
class LeapMotionModule
	: public Module< LeapMotionKey, LeapMotionComponentKey, LeapMotionModule, LeapMotionModuleComponent >, Leap::Listener
{
public:
	/** UTQL constructor */
	LeapMotionModule( const LeapMotionKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
		: Module< LeapMotionKey, LeapMotionComponentKey, LeapMotionModule, LeapMotionModuleComponent >( key, pFactory )
	{};

	/** destructor */
	~LeapMotionModule()
	{};

	virtual void startModule();

	virtual void stopModule();
	
	virtual boost::shared_ptr< LeapMotionModuleComponent > createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
		const ComponentKey& key, ModuleClass* pModule );

	virtual void onInit(const Leap::Controller&);
	virtual void onConnect(const Leap::Controller&);
	virtual void onDisconnect(const Leap::Controller&);
	virtual void onExit(const Leap::Controller&);
	virtual void onFrame(const Leap::Controller&);
	virtual void onFocusGained(const Leap::Controller&);
	virtual void onFocusLost(const Leap::Controller&);

protected:
    
	
private:
	Leap::Controller m_controller;
};



/**
 * First Component for module test
 * Does nothing but provide a push port
 */
class LeapMotionModuleComponent
	: public LeapMotionModule::Component
{
public:
	/** constructor */
	LeapMotionModuleComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const LeapMotionComponentKey& componentKey, LeapMotionModule* pModule )
		: LeapMotionModule::Component( name, componentKey, pModule )		
	{}
	
	/** destructor */
	~LeapMotionModuleComponent(){
	}

	virtual void sendData(const Leap::Frame frame, Measurement::Timestamp ts)
	{};
		
};

class LeapMotionHandComponent
	: public LeapMotionModuleComponent
{
public:
	/** constructor */
	LeapMotionHandComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const LeapMotionComponentKey& componentKey, LeapMotionModule* pModule )
		: LeapMotionModuleComponent( name, subgraph, componentKey, pModule )
		, m_palmPort( "LeapToTarget", *this )	
		, m_finger0Port( "Finger0Pose", *this )
		, m_finger1Port( "Finger1Pose", *this )
		, m_finger2Port( "Finger2Pose", *this )
		, m_finger3Port( "Finger3Pose", *this )
		, m_finger4Port( "Finger4Pose", *this )
	{
		m_fingerPorts.push_back(&m_finger0Port);
		m_fingerPorts.push_back(&m_finger1Port);
		m_fingerPorts.push_back(&m_finger2Port);
		m_fingerPorts.push_back(&m_finger3Port);
		m_fingerPorts.push_back(&m_finger4Port);
	}
	
	/** destructor */
	~LeapMotionHandComponent(){
	}
	
	virtual void sendData(const Leap::Frame frame, Measurement::Timestamp ts);
protected:
	Dataflow::PushSupplier< Measurement::Pose > m_palmPort;
	Dataflow::PushSupplier< Measurement::Pose > m_finger0Port;
	Dataflow::PushSupplier< Measurement::Pose > m_finger1Port;
	Dataflow::PushSupplier< Measurement::Pose > m_finger2Port;
	Dataflow::PushSupplier< Measurement::Pose > m_finger3Port;
	Dataflow::PushSupplier< Measurement::Pose > m_finger4Port;	
	std::vector<Dataflow::PushSupplier< Measurement::Pose >* > m_fingerPorts;

	
};

class LeapMotionToolComponent
	: public LeapMotionModuleComponent
{
public:
	/** constructor */
	LeapMotionToolComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const LeapMotionComponentKey& componentKey, LeapMotionModule* pModule )
		: LeapMotionModuleComponent( name, subgraph, componentKey, pModule )
		, m_toolPort( "LeapToTarget", *this )		
	{}
	
	/** destructor */
	~LeapMotionToolComponent(){
	}

	virtual void sendData(const Leap::Frame frame, Measurement::Timestamp ts);
protected:		
	Dataflow::PushSupplier< Measurement::Position > m_toolPort;

	
};

} } // namespace Ubitrack::Drivers

#endif
