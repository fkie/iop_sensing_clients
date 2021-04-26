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


#ifndef PATHREPORTERCLIENT_RECEIVEFSM_H
#define PATHREPORTERCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_PathReporterClient/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_PathReporterClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"


#include <nav_msgs/msg/path.hpp>
#include <geographic_msgs/msg/geo_path.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

#include "PathReporterClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>


namespace urn_jaus_jss_iop_PathReporterClient
{

const int HISTORICAL_GLOBAL_PATH = 0;
const int HISTORICAL_LOCAL_PATH = 1;
const int PLANNED_GLOBAL_PATH = 2;
const int PLANNED_LOCAL_PATH = 3;

class DllExport PathReporterClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	PathReporterClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~PathReporterClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleReportPathAction(ReportPath msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportPathReporterCapabilitiesAction(ReportPathReporterCapabilities msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void register_events(JausAddress remote_addr, double hz);
	void unregister_events(JausAddress remote_addr);
	void send_query(JausAddress remote_addr);
	void stop_query(JausAddress remote_addr);


	PathReporterClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	std::string p_tf_frame_world;
	std::string p_tf_frame_odom;

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pub_planned_local_path;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pub_planned_global_path;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pub_historical_global_path;
	rclcpp::Publisher<geographic_msgs::msg::GeoPath>::SharedPtr p_pub_planned_global_geopath;
	rclcpp::Publisher<geographic_msgs::msg::GeoPath>::SharedPtr p_pub_historical_global_geopath;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pub_historical_local_path;
	bool p_by_query;
	int p_query_state;
	double p_hz;

	QueryPath p_query_path;
	QueryPathReporterCapabilities p_query_cap;
	std::set<int> p_available_paths;

	void pPublishHistoricalGlobalPath(ReportPath::Body::PathVar::HistoricalGlobalPath* path);
	void pPublishHistoricalLocalPath(ReportPath::Body::PathVar::HistoricalLocalPath* path);
	void pPublishPlannedGlobalPath(ReportPath::Body::PathVar::PlannedGlobalPath* path);
	void pPublishPlannedLocalPath(ReportPath::Body::PathVar::PlannedLocalPath* path);

};

}

#endif // PATHREPORTERCLIENT_RECEIVEFSM_H
