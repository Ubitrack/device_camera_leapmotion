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
* @ingroup sensor_components
* @file
* Reads camera images using OpenCV's HighGUI library.
*
* @author Yuta Itoh <yuta.itoh@in.tum.de>
*/


#define DOUBLE_HANDS /// Hard-coded implementation for the second hand detection 

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <log4cpp/Category.hh>


#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>

#include <Leap.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Sensor.LeapMotionFingerTracker" ) );

using namespace Ubitrack;
using namespace Ubitrack::Measurement;

using namespace Leap;

namespace Ubitrack { namespace Drivers {

class SampleListener : public Listener {
public:
	virtual void onInit(const Controller&);
	virtual void onConnect(const Controller&);
	virtual void onDisconnect(const Controller&);
	virtual void onExit(const Controller&);
	virtual void onFrame(const Controller&);
	virtual void onFocusGained(const Controller&);
	virtual void onFocusLost(const Controller&);
#ifdef DOUBLE_HANDS
	SampleListener( Dataflow::PushSupplier< Measurement::Pose > &palmPort,
		            std::vector< Dataflow::PushSupplier< Measurement::Pose >* > &fingerPorts,
					Dataflow::PushSupplier< Measurement::Pose > &palmPort2,
		            std::vector< Dataflow::PushSupplier< Measurement::Pose >* > &fingerPorts2)
#else
	SampleListener( Dataflow::PushSupplier< Measurement::Pose > &palmPort,
		            std::vector< Dataflow::PushSupplier< Measurement::Pose >* > &fingerPorts)
#endif // DOUBLE_HANDS
		: m_palmPort(palmPort),
		m_fingerPorts(fingerPorts),
#ifdef DOUBLE_HANDS
		m_palmPort2(palmPort2),
		m_fingerPorts2(fingerPorts2),
#else
		m_palmPort(palmPort),
		m_fingerPorts(fingerPorts),
#endif // DOUBLE_HANDS
		m_kScaleMmToMeter(0.001)
	{
	};
	Dataflow::PushSupplier< Measurement::Pose > &m_palmPort;
	std::vector< Dataflow::PushSupplier< Measurement::Pose >* > &m_fingerPorts;
#ifdef DOUBLE_HANDS
	Dataflow::PushSupplier< Measurement::Pose > &m_palmPort2;
	std::vector< Dataflow::PushSupplier< Measurement::Pose >* > &m_fingerPorts2;
#endif // DOUBLE_HANDS
	const double m_kScaleMmToMeter;
};

void SampleListener::onInit(const Controller& controller) {
	LOG4CPP_INFO( logger,  "Initialized"  );
}

void SampleListener::onConnect(const Controller& controller) {
	LOG4CPP_INFO( logger,  "Connected"  );
	controller.enableGesture(Gesture::TYPE_CIRCLE);
	controller.enableGesture(Gesture::TYPE_KEY_TAP);
	controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
	controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
	//Note: not dispatched when running in a debugger.
	LOG4CPP_INFO( logger,  "Disconnected"  );
}

void SampleListener::onExit(const Controller& controller) {
	LOG4CPP_INFO( logger,  "Exited"  );
}

void SampleListener::onFrame(const Controller& controller) {
	// set timestamp
	Measurement::Timestamp time( Measurement::now() - 10000000L );
	LOG4CPP_DEBUG( logger, "time = " << time / 1000000 );

	// Get the most recent frame and report some basic information
	const Frame frame = controller.frame();
	LOG4CPP_INFO( logger,  "Frame id: " << frame.id()
		<< ", timestamp: " << frame.timestamp()
		<< ", hands: " << frame.hands().count()
		<< ", fingers: " << frame.fingers().count()
		<< ", tools: " << frame.tools().count()
		<< ", gestures: " << frame.gestures().count()  );

	if (!frame.hands().empty()) {
		// Get the first hand
		const Hand hand = frame.hands()[0];
		


		// Check if the hand has any fingers
		const FingerList fingers = hand.fingers();
		if (!fingers.empty()) {
			// Calculate the hand's average finger tip position
			Vector avgPos;

			/// Send finger positions
			for (int i = 0; i < fingers.count(); ++i) {
				avgPos += fingers[i].tipPosition();
				if (m_fingerPorts[i]->isConnected()) {
					const Math::Quaternion q;
					const Leap::Vector &xyz = fingers[i].tipPosition();
					/// scale unit from millimeter to meter
					Math::Vector<3,double> t( m_kScaleMmToMeter*xyz[0], m_kScaleMmToMeter*xyz[1], m_kScaleMmToMeter*xyz[2] );
					Math::Pose pose(q,t);
					m_fingerPorts[i]->send( Measurement::Pose(time, pose) );
				}
			}

			avgPos /= (float)fingers.count();
			LOG4CPP_INFO( logger,  "Hand has " << fingers.count()
				<< " fingers, average finger tip position" << avgPos  );
		}

		// Get the hand's sphere radius and palm position
		LOG4CPP_INFO( logger,  "Hand sphere radius: " << hand.sphereRadius()
			<< " mm, palm position: " << hand.palmPosition()  );

		// Get the hand's normal vector and direction
		const Vector normal = hand.palmNormal();
		const Vector direction = hand.direction();

		// Calculate the hand's pitch, roll, and yaw angles
		LOG4CPP_INFO( logger,  "Hand pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
			<< "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
			<< "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees"  );
		
		/// Send palm pose
		if (m_palmPort.isConnected()) {
			Math::Quaternion       q( normal.roll(), -direction.yaw(), direction.pitch() );
			Leap::Vector palm_xyz = hand.palmPosition();
			/// scale unit from millimeter to meter
			Math::Vector<3,double> t( m_kScaleMmToMeter*palm_xyz[0], m_kScaleMmToMeter*palm_xyz[1], m_kScaleMmToMeter*palm_xyz[2] ); 
			Math::Pose pose(q,t);
			m_palmPort.send( Measurement::Pose(time, pose) );
		}

	}
#ifdef DOUBLE_HANDS
	if ( frame.hands().count()>=2) {
		// Get the first hand
		const Hand hand = frame.hands()[1];


		// Check if the hand has any fingers
		const FingerList fingers = hand.fingers();
		if (!fingers.empty()) {
			// Calculate the hand's average finger tip position
			Vector avgPos;

			/// Send finger positions
			for (int i = 0; i < fingers.count(); ++i) {
				avgPos += fingers[i].tipPosition();
				if (m_fingerPorts2[i]->isConnected()) {
					const Math::Quaternion q;
					const Leap::Vector &xyz = fingers[i].tipPosition();
					/// scale unit from millimeter to meter
					Math::Vector<3,double> t( m_kScaleMmToMeter*xyz[0], m_kScaleMmToMeter*xyz[1], m_kScaleMmToMeter*xyz[2] );
					Math::Pose pose(q,t);
					m_fingerPorts2[i]->send( Measurement::Pose(time, pose) );
				}
			}

			avgPos /= (float)fingers.count();
			LOG4CPP_INFO( logger,  "Hand has " << fingers.count()
				<< " fingers, average finger tip position" << avgPos  );
		}

		// Get the hand's sphere radius and palm position
		LOG4CPP_INFO( logger,  "Hand sphere radius: " << hand.sphereRadius()
			<< " mm, palm position: " << hand.palmPosition()  );

		// Get the hand's normal vector and direction
		const Vector normal = hand.palmNormal();
		const Vector direction = hand.direction();

		// Calculate the hand's pitch, roll, and yaw angles
		LOG4CPP_INFO( logger,  "Hand pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
			<< "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
			<< "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees"  );
		
		/// Send palm pose
		if (m_palmPort.isConnected()) {
			Math::Quaternion       q( normal.roll(), -direction.yaw(), direction.pitch() );
			Leap::Vector palm_xyz = hand.palmPosition();
			/// scale unit from millimeter to meter
			Math::Vector<3,double> t( m_kScaleMmToMeter*palm_xyz[0], m_kScaleMmToMeter*palm_xyz[1], m_kScaleMmToMeter*palm_xyz[2] ); 
			Math::Pose pose(q,t);
			m_palmPort2.send( Measurement::Pose(time, pose) );
		}

	}
#endif // DOUBLE_HANDS

	// Get gestures
	const GestureList gestures = frame.gestures();
	for (int g = 0; g < gestures.count(); ++g) {
		Gesture gesture = gestures[g];

		switch (gesture.type()) {
		case Gesture::TYPE_CIRCLE:
			{
				CircleGesture circle = gesture;
				std::string clockwiseness;

				if (circle.pointable().direction().angleTo(circle.normal()) <= PI/4) {
					clockwiseness = "clockwise";
				} else {
					clockwiseness = "counterclockwise";
				}

				// Calculate angle swept since last frame
				float sweptAngle = 0;
				if (circle.state() != Gesture::STATE_START) {
					CircleGesture previousUpdate = CircleGesture(controller.frame(1).gesture(circle.id()));
					sweptAngle = (circle.progress() - previousUpdate.progress()) * 2 * PI;
				}
				LOG4CPP_INFO( logger,  "Circle id: " << gesture.id()
					<< ", state: " << gesture.state()
					<< ", progress: " << circle.progress()
					<< ", radius: " << circle.radius()
					<< ", angle " << sweptAngle * RAD_TO_DEG
					<<  ", " << clockwiseness  );
				break;
			}
		case Gesture::TYPE_SWIPE:
			{
				SwipeGesture swipe = gesture;
				LOG4CPP_INFO( logger,  "Swipe id: " << gesture.id()
					<< ", state: " << gesture.state()
					<< ", direction: " << swipe.direction()
					<< ", speed: " << swipe.speed()  );
				break;
			}
		case Gesture::TYPE_KEY_TAP:
			{
				KeyTapGesture tap = gesture;
				LOG4CPP_INFO( logger,  "Key Tap id: " << gesture.id()
					<< ", state: " << gesture.state()
					<< ", position: " << tap.position()
					<< ", direction: " << tap.direction() );
				break;
			}
		case Gesture::TYPE_SCREEN_TAP:
			{
				ScreenTapGesture screentap = gesture;
				LOG4CPP_INFO( logger,  "Screen Tap id: " << gesture.id()
					<< ", state: " << gesture.state()
					<< ", position: " << screentap.position()
					<< ", direction: " << screentap.direction() );
				break;
			}
		default:
			LOG4CPP_INFO( logger,  "Unknown gesture type."  );
			break;
		}
	}

	if (!frame.hands().empty() || !gestures.empty()) {
		LOG4CPP_INFO( logger, "" );
	}

}

void SampleListener::onFocusGained(const Controller& controller) {
	LOG4CPP_INFO( logger,  "Focus Gained"  );
}

void SampleListener::onFocusLost(const Controller& controller) {
	LOG4CPP_INFO( logger,  "Focus Lost"  );
}


/**
* @ingroup sensor_components
* Pushes hand poses from Leap Motion
*
* @par Input Ports
* None.
*
* @par Output Ports
* \c Output push port of type Measurement
Measurement::Pose.
*
* @par Configuration
* \verbatim <Configuration index="0"/> \endverbatim
* \c index can be skipped if the default camera is sufficient
*/
class LeapMotionFingerTracker
	: public Dataflow::Component
{
public:

	/** constructor */
	LeapMotionFingerTracker( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~LeapMotionFingerTracker();

	/** Component start method. starts the thread */
	virtual void start();

	/** Component stop method, stops thread */
	virtual void stop();

protected:
	// thread main loop
	void ThreadProc();

	// camera index
	int m_cameraIndex;

	//image counter
	int m_counter;

	// divisor
	int m_divisor;

	/**
	* New property values	
	*/
	// int m_hoge;


	// stop the thread?
	volatile bool m_bStop;

	// the ports
	static const int kFINGER_NUM = 5;
	Dataflow::PushSupplier< Measurement::Pose > m_palmPort;
	Dataflow::PushSupplier< Measurement::Pose > m_thumbFingerPort;
	Dataflow::PushSupplier< Measurement::Pose > m_indexFingerPort;
	Dataflow::PushSupplier< Measurement::Pose > m_middleFingerPort;
	Dataflow::PushSupplier< Measurement::Pose > m_ringFingerPort;
	Dataflow::PushSupplier< Measurement::Pose > m_littleFingerPort;
	std::vector< Dataflow::PushSupplier< Measurement::Pose >* > m_fingerPorts;
#ifdef DOUBLE_HANDS
	Dataflow::PushSupplier< Measurement::Pose > m_palmPort2;
	Dataflow::PushSupplier< Measurement::Pose > m_thumbFingerPort2;
	Dataflow::PushSupplier< Measurement::Pose > m_indexFingerPort2;
	Dataflow::PushSupplier< Measurement::Pose > m_middleFingerPort2;
	Dataflow::PushSupplier< Measurement::Pose > m_ringFingerPort2;
	Dataflow::PushSupplier< Measurement::Pose > m_littleFingerPort2;
	std::vector< Dataflow::PushSupplier< Measurement::Pose >* > m_fingerPorts2;
#endif // DOUBLE_HANDS

	/// Leap Motion classes
	// Create a sample listener and controller
	SampleListener m_listener;
	Controller m_controller;
};

LeapMotionFingerTracker::LeapMotionFingerTracker( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName ),
	m_counter( 0 ),
	m_bStop( true ),
	m_palmPort( "PalmPose", *this ),
	m_thumbFingerPort( "ThumbPose", *this ),
	m_indexFingerPort( "IndexFingerPose", *this ),
	m_middleFingerPort( "MiddleFingerPose", *this ),
	m_ringFingerPort( "RingFingerPose", *this ),
	m_littleFingerPort( "LittleFingerPose", *this ),
#ifdef DOUBLE_HANDS
	m_palmPort2( "PalmPose2", *this ),
	m_thumbFingerPort2( "ThumbPose2", *this ),
	m_indexFingerPort2( "IndexFingerPose2", *this ),
	m_middleFingerPort2( "MiddleFingerPose2", *this ),
	m_ringFingerPort2( "RingFingerPose2", *this ),
	m_littleFingerPort2( "LittleFingerPose2", *this ),
#endif // DOUBLE_HANDS
	/**
	I wish I could use C++0x notations....
	, m_fingerPorts( {
	Dataflow::PushSupplier< Measurement::Pose >( "1", *this ),
	Dataflow::PushSupplier< Measurement::Pose >( "1", *this ),
	Dataflow::PushSupplier< Measurement::Pose >( "1", *this ) } )
	**/
#ifdef DOUBLE_HANDS
	m_listener( m_palmPort, m_fingerPorts, m_palmPort2, m_fingerPorts2)
#else
	m_listener( m_palmPort, m_fingerPorts)
#endif // DOUBLE_HANDS
{
	//	subgraph->m_DataflowAttributes.getAttributeData( "highguiCameraIndex", m_cameraIndex );
	
	m_fingerPorts.clear();
	m_fingerPorts.reserve(kFINGER_NUM);
	m_fingerPorts.push_back( &m_thumbFingerPort );
	m_fingerPorts.push_back( &m_indexFingerPort );
	m_fingerPorts.push_back( &m_middleFingerPort );
	m_fingerPorts.push_back( &m_ringFingerPort );
	m_fingerPorts.push_back( &m_littleFingerPort );
#ifdef DOUBLE_HANDS
	m_fingerPorts2.push_back( &m_thumbFingerPort2 );
	m_fingerPorts2.push_back( &m_indexFingerPort2 );
	m_fingerPorts2.push_back( &m_middleFingerPort2 );
	m_fingerPorts2.push_back( &m_ringFingerPort2 );
	m_fingerPorts2.push_back( &m_littleFingerPort2 );
#endif // DOUBLE_HANDS


	stop();
}


void LeapMotionFingerTracker::stop()
{
	if( m_controller.isConnected() == true ) m_controller.removeListener(m_listener);
}

void LeapMotionFingerTracker::start()
{

	// Have the sample listener receive events from the controller
	if( m_controller.isConnected() == false ) m_controller.addListener(m_listener);

}




LeapMotionFingerTracker::~LeapMotionFingerTracker()
{
	stop();
}


void LeapMotionFingerTracker::ThreadProc()
{
	LOG4CPP_INFO( logger, "Trying to open Leap Motion #" << m_cameraIndex );

	// Have the sample listener receive events from the controller
	m_controller.addListener(m_listener);

	while ( !m_bStop )
	{
		/// sleep?
		Measurement::Timestamp time( Measurement::now() - 10000000L );
		LOG4CPP_DEBUG( logger, "time = " << time / 1000000 );
		if (m_palmPort.isConnected()) {
			//boost::shared_ptr< Math::Pose > pose
			m_palmPort.send( Measurement::Pose(time,Math::Pose()) );
		}
#ifdef DOUBLE_HANDS
		if (m_palmPort2.isConnected()) {
			//boost::shared_ptr< Math::Pose > pose
			m_palmPort2.send( Measurement::Pose(time,Math::Pose()) );
		}
#endif // DOUBLE_HANDS
	}

	// Remove the sample listener when done
	m_controller.removeListener(m_listener);
}

} } // namespace Ubitrack::Driver

/// one call per dll
UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::LeapMotionFingerTracker > ( "LeapMotionFingerTracker" );
}
