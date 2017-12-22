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

#include <tf/transform_datatypes.h>
#include <gps_common/conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <iop_builder_fkie/timestamp.h>
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_iop_PathReporterClient
{

PathReporterClient_ReceiveFSM::PathReporterClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PathReporterClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	p_tf_frame_world = "/world";
//	p_tf_frame_robot = "measurement_link";
	p_has_access = false;
	p_query_state = 0;
	p_by_query = false;
	p_hz = 0.0;
}



PathReporterClient_ReceiveFSM::~PathReporterClient_ReceiveFSM()
{
	if (p_query_timer.isValid()) {
		p_query_timer.stop();
	}
	delete context;
}

void PathReporterClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PathReporterClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PathReporterClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "PathReporterClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "PathReporterClient_ReceiveFSM");
	iop::Config cfg("~PathReporterClient");
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	// TODO: cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("hz", p_hz, p_hz, false, false);
	p_pub_planned_global_path = cfg.advertise<nav_msgs::Path>("planned_global_path", 10, false);
	p_pub_historical_global_path = cfg.advertise<nav_msgs::Path>("historical_global_path", 10, false);
	p_pub_planned_global_geopath = cfg.advertise<geographic_msgs::GeoPath>("planned_global_geopath", 10, false);
	p_pub_historical_global_geopath = cfg.advertise<geographic_msgs::GeoPath>("historical_global_geopath", 10, false);
	// TODO: p_pub_historical_local_path = cfg.advertise<nav_msgs::Path>("historical_local_path", 10, false);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:iop:PathReporter", 1, 1);
}

void PathReporterClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:iop:PathReporter") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("PathReporterClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PathReporterClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void PathReporterClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void PathReporterClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	ROS_INFO_NAMED("PathReporterClient", "get capabilities from PathReporter @ %s", p_remote_addr.str().c_str());
	sendJausMessage(p_query_cap, component);
	p_query_timer = p_nh.createTimer(ros::Duration(3), &PathReporterClient_ReceiveFSM::pQueryCallback, this);
}

void PathReporterClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (p_by_query) {
		p_query_timer.stop();
	} else {
		ROS_INFO_NAMED("PathReporterClient", "cancel EVENT for path for PathReporter @ %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_path);
	}
	p_query_state = 0;
	p_available_paths.clear();
}

void PathReporterClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		if (p_query_state == 0) {
			sendJausMessage(p_query_cap, p_remote_addr);
		} else {
			if (p_available_paths.find(HISTORICAL_GLOBAL_PATH) != p_available_paths.end()) {
				// request HistoricalGlobalPath
				p_query_path.getBody()->getQueryPathRec()->setPathType(HISTORICAL_GLOBAL_PATH);
				sendJausMessage(p_query_path, p_remote_addr);
			}
//			if (p_available_paths.find(HISTORICAL_LOCAL_PATH) != p_available_paths.end()) {
//				// request HistoricalLocalPath
//				p_query_path.getBody()->getQueryPathRec()->setPathType(HISTORICAL_LOCAL_PATH);
//				sendJausMessage(p_query_path, p_remote_addr);
//			}
			if (p_available_paths.find(PLANNED_GLOBAL_PATH) != p_available_paths.end()) {
				// request PlannedGlobalPath
				p_query_path.getBody()->getQueryPathRec()->setPathType(PLANNED_GLOBAL_PATH);
				sendJausMessage(p_query_path, p_remote_addr);
			}
//			if (p_available_paths.find(PLANNED_LOCAL_PATH) != p_available_paths.end()) {
//				// request PlannedLocalPath
//				p_query_path.getBody()->getQueryPathRec()->setPathType(PLANNED_LOCAL_PATH);
//				sendJausMessage(p_query_path, p_remote_addr);
//			}
		}
	}
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
	ROS_DEBUG_NAMED("PathReporterClient", "received path of type %d from PathReporter @ %s", path_type, sender.str().c_str());
	if (sender != p_remote_addr) {
		ROS_DEBUG_NAMED("PathReporterClient", "  it is not the current controller, ignore!");
		return;
	}
	if (path_type == HISTORICAL_GLOBAL_PATH) {
		if (p_pub_historical_global_path.getNumSubscribers() == 0 && p_pub_historical_global_geopath.getNumSubscribers() == 0) {
			// we have no subscriptions, save cpu
			return;
		}
		pPublishHistoricalGlobalPath(msg.getBody()->getPathVar()->getHistoricalGlobalPath());
	}
	if (path_type == PLANNED_GLOBAL_PATH) {
		if (p_pub_planned_global_geopath.getNumSubscribers() == 0 && p_pub_planned_global_path.getNumSubscribers() == 0) {
			// we have no subscriptions, save cpu
			return;
		}
		pPublishPlannedGlobalPath(msg.getBody()->getPathVar()->getPlannedGlobalPath());
	}
}

void PathReporterClient_ReceiveFSM::handleReportPathReporterCapabilitiesAction(ReportPathReporterCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PathReporterClient", "received capabilities from PathReporter @ %s", sender.str().c_str());
	ReportPathReporterCapabilities::Body::PathReporterCapabilitiesList* cap_list = msg.getBody()->getPathReporterCapabilitiesList();
	p_available_paths.clear();
	for (unsigned int i = 0; i < cap_list->getNumberOfElements(); i++) {
		// get available path types
		p_available_paths.insert(cap_list->getElement(i)->getPathType());
		// TODO: check for other specifications, e.g. target resolution
	}
	p_query_state = 1;
	p_query_timer.stop();
	if (p_remote_addr.get() != 0) {
		if (p_by_query) {
			if (p_hz > 0) {
				ROS_INFO_NAMED("PathReporterClient", "create QUERY timer to get path from PathReporter @ %s", p_remote_addr.str().c_str());
				p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &PathReporterClient_ReceiveFSM::pQueryCallback, this);
			} else {
				ROS_WARN_NAMED("PathReporterClient", "invalid hz %.2f for QUERY timer to get path from PathReporter @ %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			ROS_INFO_NAMED("PathReporterClient", "create EVENT to get path from PathReporter @ %s", p_remote_addr.str().c_str());
			// there is no way to request all paths, so we decide for planned global path. Other paths are requested by query.
			p_query_path.getBody()->getQueryPathRec()->setPathType(HISTORICAL_GLOBAL_PATH);
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_path, 0.0);
		}
	}
}

void PathReporterClient_ReceiveFSM::pPublishHistoricalGlobalPath(ReportPath::Body::PathVar::HistoricalGlobalPath* path)
{
	geographic_msgs::GeoPath geopath;
	geopath.header.stamp = ros::Time::now();
	geopath.header.frame_id = this->p_tf_frame_world;
	nav_msgs::Path navpath;
	navpath.header = geopath.header;
	for (unsigned int i = 0; i < path->getNumberOfElements(); i++) {
		double lat, lon, alt = 0.0;
		double roll, pitch, yaw = 0.0;
		ReportPath::Body::PathVar::HistoricalGlobalPath::GlobalPoseRec* pose = path->getElement(i);
		// create GeoPath
		geographic_msgs::GeoPoseStamped geop;
		geop.header = geopath.header;
		if (pose->isTimeStampValid()) {
			// get timestamp
			ReportPath::Body::PathVar::HistoricalGlobalPath::GlobalPoseRec::TimeStamp* ts = pose->getTimeStamp();
			iop::Timestamp stamp(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
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
		tf::Quaternion quat;
		quat.setRPY(roll, pitch, yaw);
		if (p_pub_historical_global_geopath.getNumSubscribers() > 0) {
			geop.pose.orientation.x = quat.x();
			geop.pose.orientation.y = quat.y();
			geop.pose.orientation.z = quat.z();
			geop.pose.orientation.w = quat.w();
			geopath.poses.push_back(geop);
		}
		// create Path
		if (p_pub_historical_global_path.getNumSubscribers() > 0) {
			if (lat != 0.0 && lon != 0.0) {
				double northing, easting;
				std::string zone;
				gps_common::LLtoUTM(lat, lon, northing, easting, zone);
				tf::Quaternion quat;
				quat.setRPY(roll, pitch, yaw);

				geometry_msgs::PoseStamped npose;
				npose.header = geop.header;
				npose.pose.position.x = easting;
				npose.pose.position.y = northing;
				npose.pose.position.z = alt;
				npose.pose.orientation.x = quat.x();
				npose.pose.orientation.y = quat.y();
				npose.pose.orientation.z = quat.z();
				npose.pose.orientation.w = quat.w();
				navpath.poses.push_back(npose);
			}
		}
	}
	if (p_pub_historical_global_path.getNumSubscribers() > 0) {
		p_pub_historical_global_path.publish(navpath);
	}
	if (p_pub_historical_global_geopath.getNumSubscribers() > 0) {
		p_pub_historical_global_geopath.publish(geopath);
	}
}

void PathReporterClient_ReceiveFSM::pPublishPlannedGlobalPath(ReportPath::Body::PathVar::PlannedGlobalPath* path)
{
	geographic_msgs::GeoPath geopath;
	geopath.header.stamp = ros::Time::now();
	geopath.header.frame_id = this->p_tf_frame_world;
	nav_msgs::Path navpath;
	navpath.header = geopath.header;
	for (unsigned int i = 0; i < path->getNumberOfElements(); i++) {
		double lat, lon, alt = 0.0;
		double roll, pitch, yaw = 0.0;
		ReportPath::Body::PathVar::PlannedGlobalPath::GlobalPoseRec* pose = path->getElement(i);
		// create GeoPath
		geographic_msgs::GeoPoseStamped geop;
		geop.header = geopath.header;
		if (pose->isTimeStampValid()) {
			// get timestamp
			ReportPath::Body::PathVar::PlannedGlobalPath::GlobalPoseRec::TimeStamp* ts = pose->getTimeStamp();
			iop::Timestamp stamp(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
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
		tf::Quaternion quat;
		quat.setRPY(roll, pitch, yaw);
		if (p_pub_planned_global_geopath.getNumSubscribers() > 0) {
			geop.pose.orientation.x = quat.x();
			geop.pose.orientation.y = quat.y();
			geop.pose.orientation.z = quat.z();
			geop.pose.orientation.w = quat.w();
			geopath.poses.push_back(geop);
		}
		// create Path
		if (p_pub_planned_global_path.getNumSubscribers() > 0) {
			if (lat != 0.0 && lon != 0.0) {
				double northing, easting;
				std::string zone;
				gps_common::LLtoUTM(lat, lon, northing, easting, zone);
				tf::Quaternion quat;
				quat.setRPY(roll, pitch, yaw);

				geometry_msgs::PoseStamped npose;
				npose.header = geop.header;
				npose.pose.position.x = easting;
				npose.pose.position.y = northing;
				npose.pose.position.z = alt;
				npose.pose.orientation.x = quat.x();
				npose.pose.orientation.y = quat.y();
				npose.pose.orientation.z = quat.z();
				npose.pose.orientation.w = quat.w();
				navpath.poses.push_back(npose);
			}
		}
	}
	if (p_pub_planned_global_path.getNumSubscribers() > 0) {
		p_pub_planned_global_path.publish(navpath);
	}
	if (p_pub_planned_global_geopath.getNumSubscribers() > 0) {
		p_pub_planned_global_geopath.publish(geopath);
	}
}

};

