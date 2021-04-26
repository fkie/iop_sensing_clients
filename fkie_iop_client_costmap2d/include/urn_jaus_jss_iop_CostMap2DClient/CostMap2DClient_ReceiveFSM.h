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


#ifndef COSTMAP2DCLIENT_RECEIVEFSM_H
#define COSTMAP2DCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_CostMap2DClient/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_CostMap2DClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/Messages/MessageSet.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"

#include "CostMap2DClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace urn_jaus_jss_iop_CostMap2DClient
{

class DllExport CostMap2DClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	CostMap2DClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~CostMap2DClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleAddNoGoZoneResponseAction(AddNoGoZoneResponse msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportCostMap2DAction(ReportCostMap2D msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportNoGoZonesAction(ReportNoGoZones msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods

	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void register_events(JausAddress remote_addr, double hz);
	void unregister_events(JausAddress remote_addr);
	void send_query(JausAddress remote_addr);
	void stop_query(JausAddress remote_addr);

	CostMap2DClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	std::string p_tf_frame_costmap;
	std::string p_tf_frame_odom;
	bool p_send_inverse_trafo;
	double p_hz;

	tf2_ros::TransformBroadcaster p_tf_broadcaster;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr p_pub_costmap;

	urn_jaus_jss_iop_CostMap2DClient::QueryCostMap2D p_query_costmap2d_msg;

};

}

#endif // COSTMAP2DCLIENT_RECEIVEFSM_H
