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

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

//#include <tf/transform_datatypes.h>
#include <iop_builder_fkie/timestamp.h>
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>


using namespace JTS;
using namespace iop::ocu;

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
	p_send_inverse_trafo = true;
	p_has_access = false;
	p_hz = 1.0;
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
	iop::Config cfg("~CostMap2DClient");
	cfg.param("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
	cfg.param("tf_frame_costmap", p_tf_frame_costmap, p_tf_frame_costmap);
	cfg.param("send_inverse_trafo", p_send_inverse_trafo, p_send_inverse_trafo);
	cfg.param("hz", p_hz, p_hz, false, false);
	p_pub_costmap = cfg.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:iop:CostMap2D", 1, 255);
}

void CostMap2DClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:iop:CostMap2D") == 0) {
		p_has_access = true;
		p_remote_addr = component;
	} else {
		ROS_WARN_STREAM("[CostMap2DClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void CostMap2DClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void CostMap2DClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_remote_addr = JausAddress(0);
	p_has_access = false;
}

void CostMap2DClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		if (p_hz > 0) {
			ROS_INFO_NAMED("CostMap2DClient", "create QUERY timer to get costmap2D from %s", component.str().c_str());
			p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &CostMap2DClient_ReceiveFSM::pQueryCallback, this);
		} else {
			ROS_WARN_NAMED("CostMap2DClient", "invalid hz %.2f for QUERY timer to get costmap2D from %s", p_hz, component.str().c_str());
		}
	} else {
		ROS_INFO_NAMED("CostMap2DClient", "create EVENT to get costmap2D from %s", component.str().c_str());
		pEventsClient_ReceiveFSM->create_event(*this, component, p_query_costmap2d_msg, p_hz);
	}
}

void CostMap2DClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		p_query_timer.stop();
	} else {
		ROS_INFO_NAMED("CostMap2DClient", "cancel EVENT for costmap2D by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_costmap2d_msg);
	}
}

void CostMap2DClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_costmap2d_msg, p_remote_addr);
	}
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
		tf2::Quaternion q;
		q.setRPY(0, 0, -map_pose->getCostMap2DLocalPoseRec()->getMapRotation());
		tf2::Vector3 r(map_pose->getCostMap2DLocalPoseRec()->getMapCenterX(), map_pose->getCostMap2DLocalPoseRec()->getMapCenterY(), 0);
		tf2::Transform transform( q,r);
		geometry_msgs::TransformStamped tf_msg;
		tf_msg.header.stamp = ros::Time::now();
		double yaw = map_pose->getCostMap2DLocalPoseRec()->getMapRotation();
		ROS_DEBUG_NAMED("CostMap2DClient", "decode map with origin %.2f, %.2f, yaw: %.2f", tf_msg.transform.translation.x, tf_msg.transform.translation.y, yaw);
		if (!p_send_inverse_trafo) {
			tf_msg.transform.translation.x = transform.getOrigin().getX();
			tf_msg.transform.translation.y = transform.getOrigin().getY();
			tf_msg.transform.translation.z = transform.getOrigin().getZ();
			tf_msg.transform.rotation.x = transform.getRotation().getX();
			tf_msg.transform.rotation.y = transform.getRotation().getY();
			tf_msg.transform.rotation.z = transform.getRotation().getZ();
			tf_msg.transform.rotation.w = transform.getRotation().getW();
			tf_msg.child_frame_id = this->p_tf_frame_costmap;
			tf_msg.header.frame_id = this->p_tf_frame_odom;
			ROS_DEBUG_NAMED("CostMap2DClient", "  tf %s -> %s, stamp: %d.%d", this->p_tf_frame_costmap.c_str(), this->p_tf_frame_odom.c_str(), tf_msg.header.stamp.sec, tf_msg.header.stamp.nsec);
		} else {
			tf2::Transform inverse = transform.inverse();
			tf_msg.transform.translation.x = inverse.getOrigin().getX();
			tf_msg.transform.translation.y = inverse.getOrigin().getY();
			tf_msg.transform.translation.z = inverse.getOrigin().getZ();
			tf_msg.transform.rotation.x = inverse.getRotation().getX();
			tf_msg.transform.rotation.y = inverse.getRotation().getY();
			tf_msg.transform.rotation.z = inverse.getRotation().getZ();
			tf_msg.transform.rotation.w = inverse.getRotation().getW();
			tf_msg.header.frame_id = this->p_tf_frame_odom;
			tf_msg.child_frame_id = this->p_tf_frame_costmap;
			ROS_DEBUG_NAMED("CostMap2DClient", "  tf %s -> %s, stamp: %d.%d", this->p_tf_frame_odom.c_str(), this->p_tf_frame_costmap.c_str(), tf_msg.header.stamp.sec, tf_msg.header.stamp.nsec);
		}
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
				ROS_WARN("wrong field value (0 expected): %d", map_data->getFieldValue());
			}
		} else {
			ROS_WARN("defined size %lu != data size %d", ros_msg.data.size(), map_data->getCostDataList()->getNumberOfElements());
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
