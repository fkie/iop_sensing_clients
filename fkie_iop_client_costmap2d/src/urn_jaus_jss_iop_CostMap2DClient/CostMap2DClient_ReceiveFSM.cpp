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


#include "urn_jaus_jss_iop_CostMap2DClient/CostMap2DClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <fkie_iop_builder/util.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_iop_CostMap2DClient
{



CostMap2DClient_ReceiveFSM::CostMap2DClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "CostMap2DClient", 10.0),
  logger(cmp->get_logger().get_child("CostMap2DClient")),
  p_tf_broadcaster(cmp)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new CostMap2DClient_ReceiveFSMContext(*this);

	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_tf_frame_costmap = "costmap";
	p_tf_frame_odom = "odom";
	p_send_inverse_trafo = false;
	p_hz = 0.01;
}



CostMap2DClient_ReceiveFSM::~CostMap2DClient_ReceiveFSM()
{
	delete context;
}

void CostMap2DClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_CostMap2DClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_CostMap2DClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "CostMap2DClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "CostMap2DClient_ReceiveFSM");

}


void CostMap2DClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "CostMap2DClient");
	cfg.declare_param<std::string>("tf_frame_odom", p_tf_frame_odom, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Defines the odometry frame id.",
		"Default: 'odom'");
	cfg.declare_param<std::string>("tf_frame_costmap", p_tf_frame_costmap, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Defines the map frame id.",
		"Default: 'costmap'");
	cfg.declare_param<bool>("send_inverse_trafo", p_send_inverse_trafo, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"Sets the transform direction while publish TF frame. True: tf_frame_odom -> tf_frame_costmap.",
		"Default: false");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 0.01");
	cfg.param("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
	cfg.param("tf_frame_costmap", p_tf_frame_costmap, p_tf_frame_costmap);
	cfg.param("send_inverse_trafo", p_send_inverse_trafo, p_send_inverse_trafo, true);
	cfg.param("hz", p_hz, p_hz, false);
	p_pub_costmap = cfg.create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 1);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:iop:CostMap2D", 1, 255);
	this->set_event_name("velocity state");
}

void CostMap2DClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_costmap2d_msg, p_hz);
}

void CostMap2DClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_costmap2d_msg);
	stop_query(remote_addr);
}

void CostMap2DClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	sendJausMessage(p_query_costmap2d_msg, remote_addr);
}

void CostMap2DClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
}

void CostMap2DClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportCostMap2D report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportCostMap2DAction(report, transport_data);
}

void CostMap2DClient_ReceiveFSM::handleAddNoGoZoneResponseAction(AddNoGoZoneResponse msg, Receive::Body::ReceiveRec transportData)
{
	RCLCPP_WARN(logger, "handleAddNoGoZoneResponseAction not implemented yet!");
}

void CostMap2DClient_ReceiveFSM::handleReportCostMap2DAction(ReportCostMap2D msg, Receive::Body::ReceiveRec transportData)
{
	try {
		auto ros_msg = nav_msgs::msg::OccupancyGrid();
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DRec *map_size = msg.getBody()->getCostMap2DSeq()->getCostMap2DRec();
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DPoseVar *map_pose = msg.getBody()->getCostMap2DSeq()->getCostMap2DPoseVar();
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DDataVar *map_data = msg.getBody()->getCostMap2DSeq()->getCostMap2DDataVar();

		ros_msg.info.width = map_size->getNumberOfColumns();
		ros_msg.info.height = map_size->getNumberOfRows();
		ros_msg.info.resolution = map_size->getMapWidth() / (double)ros_msg.info.width;
		ros_msg.data.resize(ros_msg.info.width * ros_msg.info.height);

		// TODO: adde check for Global Coordinate
		// we have to send a transform from odometry to the origin of the map
		tf2::Quaternion q;
		double yaw = pround(map_pose->getCostMap2DLocalPoseRec()->getMapRotation());
		q.setRPY(0, 0, yaw);
		double x_center = pround(map_pose->getCostMap2DLocalPoseRec()->getMapCenterX());
		double y_center = pround(map_pose->getCostMap2DLocalPoseRec()->getMapCenterY());
		RCLCPP_DEBUG(logger, "map center %.2f, %.2f, yaw: %.2f", x_center, y_center, yaw);
		tf2::Vector3 r(x_center, y_center, 0.0);
		tf2::Transform transform(q, r);
		auto tf_msg = geometry_msgs::msg::TransformStamped();
		tf_msg.header.stamp = cmp->now();
		if (p_send_inverse_trafo) {
			tf2::Transform inverse = transform.inverse();
			tf_msg.transform.translation.x = inverse.getOrigin().getX();
			tf_msg.transform.translation.y = inverse.getOrigin().getY();
			tf_msg.transform.translation.z = inverse.getOrigin().getZ();
			tf_msg.transform.rotation.x = inverse.getRotation().getX();
			tf_msg.transform.rotation.y = inverse.getRotation().getY();
			tf_msg.transform.rotation.z = inverse.getRotation().getZ();
			tf_msg.transform.rotation.w = inverse.getRotation().getW();
			tf_msg.child_frame_id = this->p_tf_frame_costmap;
			tf_msg.header.frame_id = this->p_tf_frame_odom;
		} else {
			tf_msg.transform.translation.x = transform.getOrigin().getX();
			tf_msg.transform.translation.y = transform.getOrigin().getY();
			tf_msg.transform.translation.z = transform.getOrigin().getZ();
			tf_msg.transform.rotation.x = transform.getRotation().getX();
			tf_msg.transform.rotation.y = transform.getRotation().getY();
			tf_msg.transform.rotation.z = transform.getRotation().getZ();
			tf_msg.transform.rotation.w = transform.getRotation().getW();
			tf_msg.header.frame_id = this->p_tf_frame_odom;
			tf_msg.child_frame_id = this->p_tf_frame_costmap;
		}
		if (! tf_msg.child_frame_id.empty() && !tf_msg.header.frame_id.empty()) {
			p_tf_broadcaster.sendTransform(tf_msg);
			RCLCPP_DEBUG(logger, "  tf %s -> %s (%.2f, %.2f), stamp: %d.%d", tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.header.stamp.sec, tf_msg.header.stamp.nanosec);
		}

		// set the origin of the map:
		// in ROS the origin is the cell in (0,0) -> since in IOP the origin is the middle of the map, move the origin
		ros_msg.header.stamp = cmp->now();
		ros_msg.header.frame_id = this->p_tf_frame_costmap;
		double xk = ros_msg.info.width;
		double yk = ros_msg.info.height;
		ros_msg.info.origin.position.x = - xk * ros_msg.info.resolution / 2.0;
		ros_msg.info.origin.position.y = - yk * ros_msg.info.resolution / 2.0;
		ros_msg.info.origin.position.z = 0.0;
		ros_msg.info.origin.orientation.x = 0;
		ros_msg.info.origin.orientation.y = 0;
		ros_msg.info.origin.orientation.z = 0;
		ros_msg.info.origin.orientation.w = 1;

		// set map data
		// We have to flip around the y axis, y for IOP costmap starts at the top and y for map at the bottom
		if (ros_msg.data.size() == map_data->getCostDataList()->getNumberOfElements()) {
			if (map_data->getFieldValue() == 0) {
				unsigned int y_i = ros_msg.info.height-1;
				unsigned int x_i = 0;
				for (unsigned int i = 0; i < map_data->getCostDataList()->getNumberOfElements(); i++) {
					unsigned int idx_map_y = ros_msg.info.width * y_i;
					unsigned int ros_ixd = idx_map_y + x_i;
					ReportCostMap2D::Body::CostMap2DSeq::CostMap2DDataVar::CostDataList::CostDataRec *el = map_data->getCostDataList()->getElement(i);
					unsigned char val = el->getCost();
					if (val == 255) {
						ros_msg.data[ros_ixd] = -1;
					} else {
						ros_msg.data[ros_ixd] = (char)((int)val / 2);
					}
					x_i++;
					if (x_i >= ros_msg.info.width) {
						y_i--;
						x_i = 0;
					}
				}
			} else {
				RCLCPP_WARN(logger, "wrong field value (0 expected): %d", map_data->getFieldValue());
			}
		} else {
			RCLCPP_WARN(logger, "defined size %lu != data size %d", ros_msg.data.size(), map_data->getCostDataList()->getNumberOfElements());
		}
		p_pub_costmap->publish(ros_msg);
	} catch (std::exception &e) {
		RCLCPP_WARN(logger, "can not publish tf or costmap: %s", e.what());
	}
}

void CostMap2DClient_ReceiveFSM::handleReportNoGoZonesAction(ReportNoGoZones msg, Receive::Body::ReceiveRec transportData)
{
	RCLCPP_WARN(logger, "handleReportNoGoZonesAction not implemented yet!");
}


}
