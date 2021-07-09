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


#include "urn_jaus_jss_iop_PathReporterClient/PathReporterClient_ReceiveFSM.h"

#include "urn_jaus_jss_iop_PathReporterClient/PathReporterClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/gps_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_iop_PathReporterClient
{



PathReporterClient_ReceiveFSM::PathReporterClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "PathReporterClient", 1.0),
  logger(cmp->get_logger().get_child("PathReporterClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PathReporterClient_ReceiveFSMContext(*this);

	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_tf_frame_world = "world";
	p_tf_frame_odom = "odom";
	p_query_state = 0;
	p_by_query = false;
	p_hz = 1.0;
}



PathReporterClient_ReceiveFSM::~PathReporterClient_ReceiveFSM()
{
	delete context;
}

void PathReporterClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PathReporterClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PathReporterClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "PathReporterClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "PathReporterClient_ReceiveFSM");

}


void PathReporterClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "PathReporterClient");
	cfg.declare_param<std::string>("tf_frame_world", p_tf_frame_world, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id used in ROS for global coordinates.",
		"Default: 'world'");
	cfg.declare_param<std::string>("tf_frame_odom", p_tf_frame_odom, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id of the odometry.",
		"Default: 'odom'");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 1.0");

	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
	cfg.param("hz", p_hz, p_hz, false);
	p_pub_planned_local_path = cfg.create_publisher<nav_msgs::msg::Path>("planned_local_path", 10);
	p_pub_historical_local_path = cfg.create_publisher<nav_msgs::msg::Path>("historical_local_path", 10);
	p_pub_planned_global_path = cfg.create_publisher<nav_msgs::msg::Path>("planned_global_path", 10);
//	p_pub_historical_global_path = cfg.create_publisher<nav_msgs::msg::Path>("historical_global_path", 10);
	p_pub_planned_global_geopath = cfg.create_publisher<geographic_msgs::msg::GeoPath>("planned_global_geopath", 10);
//	p_pub_historical_global_geopath = cfg.create_publisher<geographic_msgs::msg::GeoPath>("historical_global_geopath", 10);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:iop:PathReporter", 1, 255);
	this->set_event_name("PathReporter capabilities");
	this->set_query_before_event(true, 1.0);
}

void PathReporterClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	p_query_path.getBody()->getQueryPathRec()->setPathType(HISTORICAL_LOCAL_PATH);
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_path, 0.0);
}

void PathReporterClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_path);
	stop_query(remote_addr);
}

void PathReporterClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	if (p_query_state == 0) {
		sendJausMessage(p_query_cap, remote_addr);
	} else {
		if (p_available_paths.find(HISTORICAL_GLOBAL_PATH) != p_available_paths.end()) {
			RCLCPP_DEBUG(logger, "send query for HISTORICAL_GLOBAL_PATH to PathReporter @ %s", remote_addr.str().c_str());
			// request HistoricalGlobalPath
			QueryPath query_path;
			query_path.getBody()->getQueryPathRec()->setPathType(HISTORICAL_GLOBAL_PATH);
			sendJausMessage(query_path, remote_addr);
		}
		if (p_by_query && p_available_paths.find(HISTORICAL_LOCAL_PATH) != p_available_paths.end()) {
			RCLCPP_DEBUG(logger, "send query for HISTORICAL_LOCAL_PATH to PathReporter @ %s", remote_addr.str().c_str());
			// request HistoricalLocalPath
			QueryPath query_path;
			query_path.getBody()->getQueryPathRec()->setPathType(HISTORICAL_LOCAL_PATH);
			sendJausMessage(query_path, remote_addr);
		}
		if (p_available_paths.find(PLANNED_GLOBAL_PATH) != p_available_paths.end()) {
			// request PlannedGlobalPath
			RCLCPP_DEBUG(logger, "send query for PLANNED_GLOBAL_PATH to PathReporter @ %s", remote_addr.str().c_str());
			QueryPath query_path;
			query_path.getBody()->getQueryPathRec()->setPathType(PLANNED_GLOBAL_PATH);
			sendJausMessage(query_path, remote_addr);
		}
		if (p_available_paths.find(PLANNED_LOCAL_PATH) != p_available_paths.end()) {
			RCLCPP_DEBUG(logger, "send query for PLANNED_LOCAL_PATH to PathReporter @ %s", remote_addr.str().c_str());
			// request PlannedLocalPath
			QueryPath query_path;
			query_path.getBody()->getQueryPathRec()->setPathType(PLANNED_LOCAL_PATH);
			sendJausMessage(query_path, remote_addr);
		}
	}
}

void PathReporterClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
	p_query_state = 0;
	p_available_paths.clear();
	this->set_event_name("PathReporter capabilities");
	this->set_query_before_event(true, 1.0);
}

void PathReporterClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportPath report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportPathAction(report, transport_data);
}


void PathReporterClient_ReceiveFSM::handleReportPathAction(ReportPath msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	int path_type = msg.getBody()->getPathVar()->getFieldValue();
	RCLCPP_DEBUG(logger, "received path of type %d from PathReporter @ %s", path_type, sender.str().c_str());
	if (sender != p_remote_addr) {
		RCLCPP_DEBUG(logger, "  it is not the current controller, ignore!");
		return;
	}
	if (path_type == HISTORICAL_GLOBAL_PATH) {
		if (p_pub_historical_global_path->get_subscription_count() == 0 && p_pub_historical_global_geopath->get_subscription_count() == 0) {
			// we have no subscriptions, save cpu
			return;
		}
		pPublishHistoricalGlobalPath(msg.getBody()->getPathVar()->getHistoricalGlobalPath());
	}
	if (path_type == HISTORICAL_LOCAL_PATH) {
		if (p_pub_historical_local_path->get_subscription_count() == 0) {
			// we have no subscriptions, save cpu
			return;
		}
		pPublishHistoricalLocalPath(msg.getBody()->getPathVar()->getHistoricalLocalPath());
	}
	if (path_type == PLANNED_GLOBAL_PATH) {
		if (p_pub_planned_global_geopath->get_subscription_count() == 0 && p_pub_planned_global_path->get_subscription_count() == 0) {
			// we have no subscriptions, save cpu
			return;
		}
		pPublishPlannedGlobalPath(msg.getBody()->getPathVar()->getPlannedGlobalPath());
	}
	if (path_type == PLANNED_LOCAL_PATH) {
		if (p_pub_planned_local_path->get_subscription_count() == 0) {
			// we have no subscriptions, save cpu
			return;
		}
		pPublishPlannedLocalPath(msg.getBody()->getPathVar()->getPlannedLocalPath());
	}
}

void PathReporterClient_ReceiveFSM::handleReportPathReporterCapabilitiesAction(ReportPathReporterCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "received capabilities from PathReporter @ %s", sender.str().c_str());
	ReportPathReporterCapabilities::Body::PathReporterCapabilitiesList* cap_list = msg.getBody()->getPathReporterCapabilitiesList();
	p_available_paths.clear();
	for (unsigned int i = 0; i < cap_list->getNumberOfElements(); i++) {
		// get available path types
		RCLCPP_INFO(logger, "add path type %d for PathReporter @ %s", (int)cap_list->getElement(i)->getPathType(), sender.str().c_str());
		p_available_paths.insert(cap_list->getElement(i)->getPathType());
		// TODO: check for other specifications, e.g. target resolution
	}
	p_query_state = 1;
	// force event for request sensor data
	this->set_event_name("path");
	this->set_query_before_event(false);
}

void PathReporterClient_ReceiveFSM::pPublishHistoricalGlobalPath(ReportPath::Body::PathVar::HistoricalGlobalPath* path)
{
	auto geopath = geographic_msgs::msg::GeoPath();
	geopath.header.stamp = cmp->now();
	geopath.header.frame_id = this->p_tf_frame_world;
	auto navpath = nav_msgs::msg::Path();
	navpath.header = geopath.header;
	for (unsigned int i = 0; i < path->getNumberOfElements(); i++) {
		double lat, lon, alt = 0.0;
		double roll, pitch, yaw = 0.0;
		ReportPath::Body::PathVar::HistoricalGlobalPath::GlobalPoseRec* pose = path->getElement(i);
		// create GeoPath
		auto geop = geographic_msgs::msg::GeoPoseStamped();
		geop.header = geopath.header;
		if (pose->isTimeStampValid()) {
			// get timestamp
			ReportPath::Body::PathVar::HistoricalGlobalPath::GlobalPoseRec::TimeStamp* ts = pose->getTimeStamp();
			iop::Timestamp stamp = cmp->from_iop(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
			geop.header.stamp = stamp.ros_time;
		}
		if (pose->isAltitudeValid()) {
			alt = pose->getAltitude();
			geop.pose.position.altitude = alt;
		}
		if (pose->isLatitudeValid()) {
			lat = pose->getLatitude();
			geop.pose.position.latitude = lat;
		}
		if (pose->isLongitudeValid()) {
			lon = pose->getLongitude();
			geop.pose.position.longitude = lon;
		}

		if (pose->isRollValid()) {
			roll = pose->getRoll();
		}
		if (pose->isPitchValid()) {
			pitch = pose->getPitch();
		}
		if (pose->isYawValid()) {
			yaw = pose->getYaw();
		}
		tf2::Quaternion quat;
		quat.setRPY(roll, pitch, yaw);
		if (p_pub_historical_global_geopath->get_subscription_count() > 0) {
			geop.pose.orientation.x = quat.x();
			geop.pose.orientation.y = quat.y();
			geop.pose.orientation.z = quat.z();
			geop.pose.orientation.w = quat.w();
			geopath.poses.push_back(geop);
		}
		// create Path
		if (p_pub_historical_global_path->get_subscription_count() > 0) {
			if (lat != 0.0 && lon != 0.0) {
				double northing, easting;
				std::string zone;
				gps_common::LLtoUTM(lat, lon, northing, easting, zone);
				tf2::Quaternion q;
				q.setRPY(roll, pitch, yaw);
				auto npose = geometry_msgs::msg::PoseStamped();
				npose.header = geop.header;
				npose.pose.position.x = easting;
				npose.pose.position.y = northing;
				npose.pose.position.z = alt;
				npose.pose.orientation.x = q.x();
				npose.pose.orientation.y = q.y();
				npose.pose.orientation.z = q.z();
				npose.pose.orientation.w = q.w();
				navpath.poses.push_back(npose);
			}
		}
	}
	if (p_pub_historical_global_path->get_subscription_count() > 0) {
		p_pub_historical_global_path->publish(navpath);
	}
	if (p_pub_historical_global_geopath->get_subscription_count() > 0) {
		p_pub_historical_global_geopath->publish(geopath);
	}
}

void PathReporterClient_ReceiveFSM::pPublishHistoricalLocalPath(ReportPath::Body::PathVar::HistoricalLocalPath* path)
{
	if (p_pub_historical_local_path->get_subscription_count() == 0) {
		return;
	}
	auto navpath = nav_msgs::msg::Path();
	navpath.header.stamp = cmp->now();
	navpath.header.frame_id = p_tf_frame_odom;
	for (unsigned int i = 0; i < path->getNumberOfElements(); i++) {
		double roll, pitch, yaw = 0.0;
		ReportPath::Body::PathVar::HistoricalLocalPath::LocalPoseRec* pose = path->getElement(i);
		if (pose->isRollValid()) {
			roll = pose->getRoll();
		}
		if (pose->isPitchValid()) {
			pitch = pose->getPitch();
		}
		if (pose->isYawValid()) {
			yaw = pose->getYaw();
		}
		// create Path
		tf2::Quaternion quat;
		quat.setRPY(roll, pitch, yaw);
		auto npose = geometry_msgs::msg::PoseStamped();
		if (pose->isTimeStampValid()) {
			// get timestamp
			ReportPath::Body::PathVar::HistoricalLocalPath::LocalPoseRec::TimeStamp* ts = pose->getTimeStamp();
			iop::Timestamp stamp = cmp->from_iop(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
			npose.header.stamp = stamp.ros_time;
		}
		npose.header.frame_id = p_tf_frame_odom;
		npose.pose.position.x = pose->getX();
		npose.pose.position.y = pose->getY();
		npose.pose.position.z = pose->getZ();
		npose.pose.orientation.x = quat.x();
		npose.pose.orientation.y = quat.y();
		npose.pose.orientation.z = quat.z();
		npose.pose.orientation.w = quat.w();
		navpath.poses.push_back(npose);
	}
	p_pub_historical_local_path->publish(navpath);
}

void PathReporterClient_ReceiveFSM::pPublishPlannedGlobalPath(ReportPath::Body::PathVar::PlannedGlobalPath* path)
{
	auto geopath = geographic_msgs::msg::GeoPath();
	geopath.header.stamp = cmp->now();
	geopath.header.frame_id = this->p_tf_frame_world;
	nav_msgs::msg::Path navpath;
	navpath.header = geopath.header;
	for (unsigned int i = 0; i < path->getNumberOfElements(); i++) {
		double lat, lon, alt = 0.0;
		double roll, pitch, yaw = 0.0;
		ReportPath::Body::PathVar::PlannedGlobalPath::GlobalPoseRec* pose = path->getElement(i);
		// create GeoPath
		auto geop = geographic_msgs::msg::GeoPoseStamped();
		geop.header = geopath.header;
		if (pose->isTimeStampValid()) {
			// get timestamp
			ReportPath::Body::PathVar::PlannedGlobalPath::GlobalPoseRec::TimeStamp* ts = pose->getTimeStamp();
			iop::Timestamp stamp = cmp->from_iop(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
			geop.header.stamp = stamp.ros_time;
		}
		if (pose->isAltitudeValid()) {
			alt = pose->getAltitude();
			geop.pose.position.altitude = alt;
		}
		if (pose->isLatitudeValid()) {
			lat = pose->getLatitude();
			geop.pose.position.latitude = lat;
		}
		if (pose->isLongitudeValid()) {
			lon = pose->getLongitude();
			geop.pose.position.longitude = lon;
		}

		if (pose->isRollValid()) {
			roll = pose->getRoll();
		}
		if (pose->isPitchValid()) {
			pitch = pose->getPitch();
		}
		if (pose->isYawValid()) {
			yaw = pose->getYaw();
		}
		tf2::Quaternion quat;
		quat.setRPY(roll, pitch, yaw);
		if (p_pub_planned_global_geopath->get_subscription_count() > 0) {
			geop.pose.orientation.x = quat.x();
			geop.pose.orientation.y = quat.y();
			geop.pose.orientation.z = quat.z();
			geop.pose.orientation.w = quat.w();
			geopath.poses.push_back(geop);
		}
		// create Path
		if (p_pub_planned_global_path->get_subscription_count() > 0) {
			if (lat != 0.0 && lon != 0.0) {
				double northing, easting;
				std::string zone;
				gps_common::LLtoUTM(lat, lon, northing, easting, zone);
				tf2::Quaternion q;
				q.setRPY(roll, pitch, yaw);

				auto npose = geometry_msgs::msg::PoseStamped();
				npose.header = geop.header;
				npose.pose.position.x = easting;
				npose.pose.position.y = northing;
				npose.pose.position.z = alt;
				npose.pose.orientation.x = q.x();
				npose.pose.orientation.y = q.y();
				npose.pose.orientation.z = q.z();
				npose.pose.orientation.w = q.w();
				navpath.poses.push_back(npose);
			}
		}
	}
	if (p_pub_planned_global_path->get_subscription_count() > 0) {
		p_pub_planned_global_path->publish(navpath);
	}
	if (p_pub_planned_global_geopath->get_subscription_count() > 0) {
		p_pub_planned_global_geopath->publish(geopath);
	}
}

void PathReporterClient_ReceiveFSM::pPublishPlannedLocalPath(ReportPath::Body::PathVar::PlannedLocalPath* path)
{
	if (p_pub_planned_local_path->get_subscription_count() == 0) {
		return;
	}
	auto navpath = nav_msgs::msg::Path();
	navpath.header.stamp = cmp->now();
	navpath.header.frame_id = p_tf_frame_odom;
	for (unsigned int i = 0; i < path->getNumberOfElements(); i++) {
		double roll, pitch, yaw = 0.0;
		ReportPath::Body::PathVar::PlannedLocalPath::LocalPoseRec* pose = path->getElement(i);
		if (pose->isRollValid()) {
			roll = pose->getRoll();
		}
		if (pose->isPitchValid()) {
			pitch = pose->getPitch();
		}
		if (pose->isYawValid()) {
			yaw = pose->getYaw();
		}
		// create Path
		tf2::Quaternion quat;
		quat.setRPY(roll, pitch, yaw);
		auto npose = geometry_msgs::msg::PoseStamped();
		if (pose->isTimeStampValid()) {
			// get timestamp
			ReportPath::Body::PathVar::PlannedLocalPath::LocalPoseRec::TimeStamp* ts = pose->getTimeStamp();
			iop::Timestamp stamp = cmp->from_iop(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
			npose.header.stamp = stamp.ros_time;
		}
		npose.header.frame_id = p_tf_frame_odom;
		npose.pose.position.x = pose->getX();
		npose.pose.position.y = pose->getY();
		npose.pose.position.z = pose->getZ();
		npose.pose.orientation.x = quat.x();
		npose.pose.orientation.y = quat.y();
		npose.pose.orientation.z = quat.z();
		npose.pose.orientation.w = quat.w();
		navpath.poses.push_back(npose);
	}
	p_pub_planned_local_path->publish(navpath);
}

}

