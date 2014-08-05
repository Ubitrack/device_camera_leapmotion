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

#include "LeapMotion.h"

namespace Ubitrack { namespace Drivers {

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.LeapMotion" ) );

void LeapMotionModule::startModule()
{
	//if( m_controller.isConnected() == false ) 
		m_controller.addListener(*this);
}

void LeapMotionModule::stopModule()
{
	//if( m_controller.isConnected() == true ) 
		m_controller.removeListener(*this);
}

boost::shared_ptr< LeapMotionModuleComponent > LeapMotionModule::createComponent( const std::string& type, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
		const ComponentKey& key, ModuleClass* pModule )
	{ 
		 
	if ( type == "LeapMotionHand" )
		return boost::shared_ptr< LeapMotionModuleComponent >( new LeapMotionHandComponent( name, subgraph, key, pModule ) );
	else if ( type == "LeapMotionTool" )
		return boost::shared_ptr< LeapMotionModuleComponent >( new LeapMotionToolComponent( name, subgraph, key, pModule ) );
	

	UBITRACK_THROW( "Class " + type + " not supported by test module" );
}

void LeapMotionModule::onInit(const Leap::Controller& controller) {
	LOG4CPP_INFO( logger,  "Initialized"  );
}

void LeapMotionModule::onConnect(const Leap::Controller& controller) {
	LOG4CPP_INFO( logger,  "Connected"  );
	//controller.enableGesture(Gesture::TYPE_CIRCLE);
	//controller.enableGesture(Gesture::TYPE_KEY_TAP);
	//controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
	//controller.enableGesture(Gesture::TYPE_SWIPE);
}

void LeapMotionModule::onDisconnect(const Leap::Controller& controller) {
	//Note: not dispatched when running in a debugger.
	LOG4CPP_INFO( logger,  "Disconnected"  );
}

void LeapMotionModule::onExit(const Leap::Controller& controller) {
	LOG4CPP_INFO( logger,  "Exited"  );
}

void LeapMotionModule::onFrame(const Leap::Controller& controller) {
	using namespace Leap;
	// set timestamp
	//Measurement::Timestamp time( Measurement::now() - 10000000L );
	Measurement::Timestamp ts = Measurement::now();

	LOG4CPP_DEBUG( logger, "time = " << ts / 1000000 );

	// Get the most recent frame and report some basic information
	const Frame frame = controller.frame();
	
	LOG4CPP_DEBUG( logger,  "Frame id: " << frame.id()
		<< ", timestamp: " << frame.timestamp()
		<< ", hands: " << frame.hands().count()
		<< ", fingers: " << frame.fingers().count()
		<< ", tools: " << frame.tools().count()
		<< ", gestures: " << frame.gestures().count()  );
	
	HandList hands = frame.hands();
	for(int i=0;i<hands.count();i++){
		LeapMotionComponentKey handKey(i, 1);
		if(hasComponent(handKey)){						
			getComponent(handKey)->sendData(frame,ts);
		}
	}

	ToolList tools = frame.tools();
	for(int i=0;i<tools.count();i++){
		LeapMotionComponentKey toolKey(i, 2);
		if(hasComponent(toolKey)){						
			getComponent(toolKey)->sendData(frame,ts);
		}
	}	
}

void LeapMotionModule::onFocusGained(const Leap::Controller& controller) {
	LOG4CPP_INFO( logger,  "Focus Gained"  );
}

void LeapMotionModule::onFocusLost(const Leap::Controller& controller) {
	LOG4CPP_INFO( logger,  "Focus Lost"  );
}

void LeapMotionHandComponent::sendData(const Leap::Frame frame, Measurement::Timestamp ts)
{
	using namespace Leap;
	int handID = getKey().m_targetID;
	const Hand hand = frame.hands()[handID];	
	
	// Get the hand's sphere radius and palm position
	//LOG4CPP_DEBUG( logger,  "Hand sphere radius: " << hand.sphereRadius()
	//	<< " mm, palm position: " << hand.palmPosition()  );

	// Get the hand's normal vector and direction
	const Vector normal = hand.palmNormal();
	const Vector direction = hand.direction();

	// Calculate the hand's pitch, roll, and yaw angles
	//LOG4CPP_DEBUG( logger,  "Hand pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
	//	<< "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
	//	<< "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees"  );
		
	Math::Quaternion       q( normal.roll(), -direction.yaw(), direction.pitch() );
	Leap::Vector palm_xyz = hand.palmPosition();
	/// scale unit from millimeter to meter
	Math::Vector3d t( palm_xyz[0]/1000.0f, palm_xyz[1]/1000.0f, palm_xyz[2]/1000.0f );

	Math::Quaternion rotQ(-0.7071,0,0,0.7071);
	Math::Quaternion rotQ2(0,1,0,0);
	rotQ = rotQ * rotQ2;
	Math::Vector3d rotT(0,0,0);

	Math::Pose rotationPose(rotQ,rotT);

	Math::Pose palmPose(q,t);

	palmPose = rotationPose * palmPose;

	/// Send palm pose
	if (m_palmPort.isConnected()) {
		
		m_palmPort.send( Measurement::Pose(ts, palmPose) );
	}

	// Check if the hand has any fingers
	const FingerList fingers = hand.fingers();
	for (int i = 0; i < fingers.count(); ++i) {	
		if (m_fingerPorts[i]->isConnected()) {
			const Math::Quaternion q;
			const Leap::Vector &xyz = fingers[i].tipPosition();
			/// scale unit from millimeter to meter
			Math::Vector3d t( xyz[0]/1000.0f, xyz[1]/1000.0f, xyz[2]/1000.0f );
			Math::Pose pose(q,t);
			pose = rotationPose * pose;
			
			pose = ~palmPose * pose;

			m_fingerPorts[i]->send( Measurement::Pose(ts, pose) );
		}
	}

}

void LeapMotionToolComponent::sendData(const Leap::Frame frame, Measurement::Timestamp ts)
{
	using namespace Leap;
	int toolID = getKey().m_targetID;
	const Tool tool = frame.tools()[toolID];


	if (m_toolPort.isConnected()) {		
		Leap::Vector tool_xyz = tool.tipPosition();
		/// scale unit from millimeter to meter
		Math::Vector3d t( tool_xyz[0]/1000.0f, tool_xyz[1]/1000.0f, tool_xyz[2]/1000.0f );

		Math::Quaternion rotQ(-0.7071,0,0,0.7071);
	Math::Quaternion rotQ2(0,1,0,0);
	rotQ = rotQ * rotQ2;
	Math::Vector3d rotT(0,0,0);

	Math::Pose rotationPose(rotQ,rotT);

		t = rotationPose * t;
		m_toolPort.send( Measurement::Position(ts, t) );
	}
}

// register module at factory
UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) {
	std::vector< std::string > moduleComponents;
	moduleComponents.push_back( "LeapMotionHand" );
	moduleComponents.push_back( "LeapMotionTool" );
	

	cf->registerModule< LeapMotionModule > ( moduleComponents );
}

} } // namespace Ubitrack::Drivers