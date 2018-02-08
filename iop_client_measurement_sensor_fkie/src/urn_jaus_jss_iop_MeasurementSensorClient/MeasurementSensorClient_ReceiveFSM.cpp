/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#include "urn_jaus_jss_iop_MeasurementSensorClient/MeasurementSensorClient_ReceiveFSM.h"

#include <tf/transform_datatypes.h>
#include <gps_common/conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <iop_builder_fkie/timestamp.h>
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_iop_MeasurementSensorClient
{



MeasurementSensorClient_ReceiveFSM::MeasurementSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new MeasurementSensorClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
//	p_tf_frame_world = "world";
//	p_tf_frame_robot = "measurement_link";
	p_has_access = false;
	p_hz = 0.0;
}



MeasurementSensorClient_ReceiveFSM::~MeasurementSensorClient_ReceiveFSM()
{
	if (p_query_timer.isValid()) {
		p_query_timer.stop();
	}
	delete context;
}

void MeasurementSensorClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_MeasurementSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_MeasurementSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "MeasurementSensorClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "MeasurementSensorClient_ReceiveFSM");
	iop::Config cfg("~MeasurementSensorClient");
//	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
//	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("hz", p_hz, p_hz, false, false);
	p_pub_meas = cfg.advertise<iop_msgs_fkie::Measurement>("measurement", 10, false);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:iop:MeasurementSensor", 1, 255);
}

void MeasurementSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:iop:MeasurementSensor") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[MeasurementSensorClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void MeasurementSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void MeasurementSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void MeasurementSensorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		if (p_hz > 0) {
			ROS_INFO_NAMED("MeasurementSensorClient", "create QUERY timer to get measurement from %s", component.str().c_str());
			p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &MeasurementSensorClient_ReceiveFSM::pQueryCallback, this);
		} else {
			ROS_WARN_NAMED("MeasurementSensorClient", "invalid hz %.2f for QUERY timer to get measurement from %s", p_hz, component.str().c_str());
		}
	} else {
		ROS_INFO_NAMED("MeasurementSensorClient", "create EVENT to get measurement from %s", component.str().c_str());
		pEventsClient_ReceiveFSM->create_event(*this, component, p_query_measurement, p_hz);
	}
}

void MeasurementSensorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		p_query_timer.stop();
	} else {
		ROS_INFO_NAMED("MeasurementSensorClient", "cancel EVENT for measurement @ %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_measurement);
	}
}

void MeasurementSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_measurement, p_remote_addr);
	}
}

void MeasurementSensorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportMeasurement report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportMeasurementAction(report, transport_data);
}

void MeasurementSensorClient_ReceiveFSM::handleReportMeasurementAction(ReportMeasurement msg, Receive::Body::ReceiveRec transportData)
{
	iop_msgs_fkie::Measurement rosmsg;
	rosmsg.device_name = msg.getBody()->getMeasurementSeq()->getDeviceRec()->getDeviceName();
	if (msg.getBody()->getMeasurementSeq()->getDeviceRec()->isDeviceDesignationValid()) {
		rosmsg.device_designation = msg.getBody()->getMeasurementSeq()->getDeviceRec()->getDeviceDesignation();
	}
	if (msg.getBody()->getMeasurementSeq()->getDeviceRec()->isClassificationValid()) {
		rosmsg.classification = msg.getBody()->getMeasurementSeq()->getDeviceRec()->getClassification();
	}
	if (msg.getBody()->getMeasurementSeq()->isGlobalPoseRecValid()) {
		rosmsg.latitude = msg.getBody()->getMeasurementSeq()->getGlobalPoseRec()->getLatitude();
		rosmsg.longitude = msg.getBody()->getMeasurementSeq()->getGlobalPoseRec()->getLongitude();
		rosmsg.altitude = msg.getBody()->getMeasurementSeq()->getGlobalPoseRec()->getAltitude();
	}
	if (msg.getBody()->getMeasurementSeq()->isTimestampRecValid()) {
		// get timestamp
		ReportMeasurement::Body::MeasurementSeq::TimestampRec::TimeStamp *ts = msg.getBody()->getMeasurementSeq()->getTimestampRec()->getTimeStamp();
		iop::Timestamp stamp(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
		rosmsg.header.stamp = stamp.ros_time;
	} else {
		rosmsg.header.stamp = ros::Time::now();
	}
	ReportMeasurement::Body::MeasurementSeq::ReadingsList* rlist = msg.getBody()->getMeasurementSeq()->getReadingsList();
	for (unsigned int i = 0; i < rlist->getNumberOfElements(); i++) {
		ReportMeasurement::Body::MeasurementSeq::ReadingsList::ReadingSeq* valseq = rlist->getElement(i);
		iop_msgs_fkie::MeasurementValue mval;
		mval.sensor = valseq->getReadingRec()->getSensor();
		mval.source = valseq->getReadingRec()->getSource();
		mval.type = valseq->getReadingRec()->getType();
		mval.unit = valseq->getReadingRec()->getUnit();
		mval.min = valseq->getReadingRec()->getMinimum();
		mval.max = valseq->getReadingRec()->getMaximum();
		mval.avg = valseq->getReadingRec()->getAvarage();
		mval.alert_level = valseq->getReadingRec()->getAlertLevel();
		mval.alert_explanation = valseq->getReadingRec()->getAlertExplanation();
		mval.extended_info = valseq->getReadingRec()->getExtendedInfo();
		for (unsigned int v = 0; v < valseq->getValueList()->getNumberOfElements(); i++) {
			mval.value.push_back(valseq->getValueList()->getElement(v)->getValue());
		}
		rosmsg.values.push_back(mval);
	}

//	rosmsg.header.frame_id = p_tf_frame_robot;
//	tf::StampedTransform transform;
//	tf::Quaternion quat;
//	tf::Transform btTrans;
//	double northing, easting;
//	std::string zone;
//	gps_common::LLtoUTM(rosmsg.latitude, rosmsg.longitude, northing, easting, zone);
//	tf::Vector3 translation(easting, northing, 0.0);
//	btTrans = tf::Transform(quat, translation);
//	transform.stamp_ = fix.header.stamp;
//	transform.setData(btTrans);
//	transform.frame_id_ = this->p_tf_frame_world;
//	transform.child_frame_id_ = this->p_tf_frame_robot;
//	ROS_DEBUG_NAMED("MeasurementSensorClient", "tf %s -> %s", this->p_tf_frame_world.c_str(), this->p_tf_frame_robot.c_str());
//	if (! transform.child_frame_id_.empty()) {
//		p_tf_broadcaster.sendTransform(transform);
//	}
	p_pub_meas.publish(rosmsg);
}


};

