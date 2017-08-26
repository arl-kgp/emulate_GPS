/**
 * @brief Offboard control plugin
 * @file offboard.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 *
 * @addtogroup plugin
 */
/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

namespace mavplugin {

class OffboardPlugin : public MavRosPlugin {
public:
	OffboardPlugin():
	  nh("~"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;
		ROS_INFO_NAMED("Offboard", "initialize");

		/*=================== VERY IMPORTANT =========================
		* This topic should be published at < 5 Hz 
		*/
		offboard_odom_sub = nh.subscribe("/albatross/ground_truth/odometry", 1, &OffboardPlugin::offboard_odom_cb, this);
	}

	/**
	 * This function returns message<->handler mapping
	 *
	 * Each entry defined by @a MESSAGE_HANDLER() macro
	 */
	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_HEARTBEAT, &OffboardPlugin::handle_heartbeat),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_SYS_STATUS, &OffboardPlugin::handle_sys_status),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_STATUSTEXT, &OffboardPlugin::handle_statustext)
		};
	}

private:
	ros::NodeHandle nh;
	UAS *uas;
	ros::Subscriber offboard_odom_sub;

	void handle_heartbeat(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		// ROS_INFO_NAMED("Offboard", "Offboard::handle_heartbeat(%p, %u, %u)",
		// 		msg, sysid, compid);
	}

	void handle_sys_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		// ROS_INFO_NAMED("Offboard", "Offboard::handle_sys_status(%p, %u, %u)",
		// 		msg, sysid, compid);
	}

	void handle_statustext(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		// ROS_INFO_NAMED("Offboard", "Offboard::handle_statustext(%p, %u, %u)",
		// 		msg, sysid, compid);
	}

	/* -*- low-level send -*- */

	// perhaps better add send_ prefix, naming: message name in lower case
	void gps_position_estimate(const nav_msgs::OdometryConstPtr req) {
		/*
 * @brief Pack a gps_raw_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)

static inline uint16_t mavlink_msg_gps_raw_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
{*/

		double start_lat = -35.363261; double start_lon = 149.165230;
		double current_lat = start_lat + (180.0/M_PI)*(req->pose.pose.position.x/6378137.0);
    	double current_lon = start_lon + (180.0/M_PI)*(req->pose.pose.position.y/6378137.0)/cos(start_lat);

    	//double current_lat = start_lat;
    	//double current_lon = start_lon;

		mavlink_message_t packet;
		mavlink_msg_gps_raw_int_pack(0, 0, &packet,
																 ros::Time::now().nsec*1000,
																 3,
																 (int32_t)(current_lat*10000000),
																 (int32_t)(current_lon*10000000),
																 (int32_t)(req->pose.pose.position.z*100),
																 10,
																 10,
																 UINT16_MAX,
																 UINT16_MAX,
																 14);

		uas->fcu_link->send_message(&packet);
	}

	/* -*- callbacks -*- */

	// Did you really need service?
	// I think topic subscriber are better for this case
	void offboard_odom_cb(const nav_msgs::OdometryConstPtr req) {

		gps_position_estimate(req);
	}

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::OffboardPlugin, mavplugin::MavRosPlugin)
