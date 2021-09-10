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
#include <fkie_iop_component/iop_config.hpp>

#include <fkie_iop_component/timestamp.hpp>

using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_iop_MeasurementSensorClient
{



MeasurementSensorClient_ReceiveFSM::MeasurementSensorClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "RangeSensorClient", 2.0),
  logger(cmp->get_logger().get_child("MeasurementSensorClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new MeasurementSensorClient_ReceiveFSMContext(*this);

	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_hz = 1.0;
}



MeasurementSensorClient_ReceiveFSM::~MeasurementSensorClient_ReceiveFSM()
{
	delete context;
}

void MeasurementSensorClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_MeasurementSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_MeasurementSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "MeasurementSensorClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "MeasurementSensorClient_ReceiveFSM");

}


void MeasurementSensorClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "MeasurementSensorClient");
//	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
//	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 1.0");

	cfg.param("hz", p_hz, p_hz, false);
	p_pub_meas = cfg.create_publisher<fkie_iop_msgs::msg::Measurement>("measurement", 10);
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:iop:MeasurementSensor", 1, 255);
}

void MeasurementSensorClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_measurement, 0.0);
}

void MeasurementSensorClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_measurement);
	stop_query(remote_addr);
}

void MeasurementSensorClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	sendJausMessage(p_query_measurement, p_remote_addr);
}

void MeasurementSensorClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
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
	auto rosmsg = fkie_iop_msgs::msg::Measurement();
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
		iop::Timestamp stamp(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds(), cmp->now(), true);
		rosmsg.header.stamp = stamp.ros_time;
	} else {
		rosmsg.header.stamp = cmp->now();
	}
	ReportMeasurement::Body::MeasurementSeq::ReadingsList* rlist = msg.getBody()->getMeasurementSeq()->getReadingsList();
	for (unsigned int i = 0; i < rlist->getNumberOfElements(); i++) {
		ReportMeasurement::Body::MeasurementSeq::ReadingsList::ReadingSeq* valseq = rlist->getElement(i);
		auto mval = fkie_iop_msgs::msg::MeasurementValue();
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
		for (unsigned int v = 0; v < valseq->getValueList()->getNumberOfElements(); v++) {
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
//	RCLCPP_DEBUG(logger, "tf %s -> %s", this->p_tf_frame_world.c_str(), this->p_tf_frame_robot.c_str());
//	if (! transform.child_frame_id_.empty()) {
//		p_tf_broadcaster.sendTransform(transform);
//	}
	p_pub_meas->publish(rosmsg);
}


}

