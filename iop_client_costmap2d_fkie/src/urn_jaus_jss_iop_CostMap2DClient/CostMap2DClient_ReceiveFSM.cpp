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

#include <tf/transform_datatypes.h>
#include <iop_builder_fkie/timestamp.h>



using namespace JTS;

namespace urn_jaus_jss_iop_CostMap2DClient
{



CostMap2DClient_ReceiveFSM::CostMap2DClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new CostMap2DClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	p_tf_frame_costmap = "costmap";
	p_tf_frame_odom = "odom";
	p_pnh = ros::NodeHandle("~");
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

	p_pnh.param("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
	ROS_INFO("  tf_frame_odom: %s", p_tf_frame_odom.c_str());
	p_pnh.param("tf_frame_costmap", p_tf_frame_costmap, p_tf_frame_costmap);
	ROS_INFO("  tf_frame_costmap: %s", p_tf_frame_costmap.c_str());
	p_pub_costmap = p_nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);
	p_ocu_control_layer_slave.set_access_state_handler(&CostMap2DClient_ReceiveFSM::pAccessStateHandler, this);
	p_ocu_control_layer_slave.init(*(jausRouter->getJausAddress()), "urn:jaus:jss:iop:CostMap2D", 1, 0);
}

void CostMap2DClient_ReceiveFSM::pAccessStateHandler(JausAddress &address, unsigned char code)
{
	if (code == OcuControlSlave::ACCESS_STATE_CONTROL_ACCEPTED) {
		// create event
		ROS_INFO_NAMED("CostMap2DClient", "create event to get costmap2D from %d.%d.%d",
				address.getSubsystemID(), address.getNodeID(), address.getComponentID());
		pEventsClient_ReceiveFSM->create_event(&CostMap2DClient_ReceiveFSM::pHandleEventReportMap, this, address, p_query_costmap2d_msg, 0, 1);
	} else if (code == OcuControlSlave::ACCESS_CONTROL_RELEASE) {
		pEventsClient_ReceiveFSM->cancel_event(address, p_query_costmap2d_msg);
		ROS_INFO_NAMED("CostMap2DClient", "cancel event for costmap2D by %d.%d.%d",
				address.getSubsystemID(), address.getNodeID(), address.getComponentID());
	}
}

void CostMap2DClient_ReceiveFSM::pHandleEventReportMap(JausAddress &sender, unsigned int reportlen, const unsigned char* reportdata)
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
	/// Insert User Code HERE
	ROS_WARN("CostMap2DClient: handleAddNoGoZoneResponseAction not implemented yet!");
}

void CostMap2DClient_ReceiveFSM::handleReportCostMap2DAction(ReportCostMap2D msg, Receive::Body::ReceiveRec transportData)
{
	try {
		nav_msgs::OccupancyGrid ros_msg;
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DRec *map_size = msg.getBody()->getCostMap2DSeq()->getCostMap2DRec();
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DPoseVar *map_pose = msg.getBody()->getCostMap2DSeq()->getCostMap2DPoseVar();
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DDataVar *map_data = msg.getBody()->getCostMap2DSeq()->getCostMap2DDataVar();

		ros_msg.info.width = map_size->getNumberOfColumns();
		ros_msg.info.height = map_size->getNumberOfRows();
		ros_msg.info.resolution = map_size->getMapWidth() / (double)ros_msg.info.width;
		ros_msg.data.resize(ros_msg.info.width * ros_msg.info.height);

		// we have to send a transform from odometry to the origin of the map
		geometry_msgs::TransformStamped tf_msg;
		tf_msg.transform.translation.x = map_pose->getCostMap2DLocalPoseRec()->getMapCenterX();
		tf_msg.transform.translation.y = map_pose->getCostMap2DLocalPoseRec()->getMapCenterY();
		tf_msg.transform.translation.z = 0.0;
		double roll = 0.0;
		double pitch = 0.0;
		double yaw = map_pose->getCostMap2DLocalPoseRec()->getMapRotation();
		tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
		tf_msg.transform.rotation.x = q.x();
		tf_msg.transform.rotation.y = q.y();
		tf_msg.transform.rotation.z = q.z();
		tf_msg.transform.rotation.w = q.w();
		tf_msg.header.stamp = ros::Time::now();
		tf_msg.header.frame_id = this->p_tf_frame_odom;
		tf_msg.child_frame_id = this->p_tf_frame_costmap;
		ROS_DEBUG_NAMED("CostMap2DClient", "decode map with origin %.2f, %.2f, yaw: %.2f", tf_msg.transform.translation.x, tf_msg.transform.translation.y, yaw);
		ROS_DEBUG_NAMED("CostMap2DClient", "  tf %s -> %s", this->p_tf_frame_odom.c_str(), this->p_tf_frame_costmap.c_str());
		if (! tf_msg.child_frame_id.empty()) {
			p_tf_broadcaster.sendTransform(tf_msg);
		}

		// set the origin of the map:
		// in ROS the origin is the cell in (0,0) -> since in IOP the origin is the middle of the map, move the origin
		ros_msg.header.stamp = ros::Time::now();
		ros_msg.header.frame_id = this->p_tf_frame_costmap;
		double xk = ros_msg.info.width;
		double yk = ros_msg.info.width;
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
				std::cout << "  wrong field value (0 expected) " << map_data->getFieldValue() << std::endl;
			}
		} else {
			std::cout << "  defined size " << ros_msg.data.size() << "!=" << "data size " << map_data->getCostDataList()->getNumberOfElements() << std::endl;
		}
		p_pub_costmap.publish(ros_msg);
	} catch (std::exception &e) {
		ROS_WARN("CostMap2DClient: can not publish tf or costmap: %s", e.what());
	}
}

void CostMap2DClient_ReceiveFSM::handleReportNoGoZonesAction(ReportNoGoZones msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	ROS_WARN("CostMap2DClient: handleReportNoGoZonesAction not implemented yet!");
}


};
