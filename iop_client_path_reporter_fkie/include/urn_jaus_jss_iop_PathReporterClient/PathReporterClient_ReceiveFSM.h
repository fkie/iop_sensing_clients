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


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geographic_msgs/GeoPath.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>
#include <iop_events_fkie/EventHandlerInterface.h>

#include "PathReporterClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_iop_PathReporterClient
{

const int HISTORICAL_GLOBAL_PATH = 0;
const int HISTORICAL_LOCAL_PATH = 1;
const int PLANNED_GLOBAL_PATH = 2;
const int PLANNED_LOCAL_PATH = 3;

class DllExport PathReporterClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	PathReporterClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM);
	virtual ~PathReporterClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportPathAction(ReportPath msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportPathReporterCapabilitiesAction(ReportPathReporterCapabilities msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);



	PathReporterClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;

	std::string p_tf_frame_world;
	std::string p_tf_frame_odom;

	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	ros::Publisher p_pub_planned_local_path;
	ros::Publisher p_pub_planned_global_path;
	ros::Publisher p_pub_historical_global_path;
	ros::Publisher p_pub_planned_global_geopath;
	ros::Publisher p_pub_historical_global_geopath;
	ros::Publisher p_pub_historical_local_path;
	bool p_by_query;
	int p_query_state;
	double p_hz;

	QueryPath p_query_path;
	QueryPathReporterCapabilities p_query_cap;
	std::set<int> p_available_paths;
	JausAddress p_remote_addr;
	bool p_has_access;
	void pQueryCallback(const ros::TimerEvent& event);

	void pPublishHistoricalGlobalPath(ReportPath::Body::PathVar::HistoricalGlobalPath* path);
	void pPublishHistoricalLocalPath(ReportPath::Body::PathVar::HistoricalLocalPath* path);
	void pPublishPlannedGlobalPath(ReportPath::Body::PathVar::PlannedGlobalPath* path);
	void pPublishPlannedLocalPath(ReportPath::Body::PathVar::PlannedLocalPath* path);

};

};

#endif // PATHREPORTERCLIENT_RECEIVEFSM_H
