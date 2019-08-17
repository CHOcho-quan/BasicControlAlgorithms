#include "sectormap.h"

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/WaypointList.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <GeographicLib/Geocentric.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros/frame_tf.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/PositionTarget.h>

SectorMap *smap;
uint32_t init_mask = 0;

// Getting GPS information
Eigen::Vector3d current_gps;
void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	init_mask |= 1;
	current_gps = { msg->latitude, msg->longitude, msg->altitude };
}

// Getting Current State
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	init_mask |= 1 << 1;
	current_state = *msg;
}

// Getting initial position
mavros_msgs::HomePosition home_pos;
void home_pos_cb(const mavros_msgs::HomePosition::ConstPtr& msg) {
	init_mask |= 1 << 2;
	home_pos = *msg;
}

geometry_msgs::PoseStamped local_pos;
Eigen::Vector3d current_local_pos;
bool local_pos_updated = false;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	init_mask |= 1 << 3;
	current_local_pos = mavros::ftf::to_eigen(msg->pose.position);
	local_pos = *msg;
	Point2D pt2d;
	pt2d.x = current_local_pos.x();
	pt2d.y = current_local_pos.y();
	smap->SetUavPosition(pt2d);
	local_pos_updated = true;
}

// Getting Waypoints
mavros_msgs::WaypointList waypoints;
void waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg) {
	init_mask |= 1 << 4;
	waypoints = *msg;
}

// Getting Laser Data
bool scan_updated = false;
void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//激光雷达数据回调函数
	init_mask |= 1 << 5;
	smap->ComputeMV(msg->ranges);
	scan_updated = true;
}

// Getting Yaw
bool heading_updated = false;
void heading_cb(const std_msgs::Float64::ConstPtr &msg)
{
	init_mask |= 1 << 6;
	smap->SetUavHeading(msg->data);
	heading_updated = true;
}

int main(int argc, char **argv)
{
	// Initialization
	smap = new SectorMap;

	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;

	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 100, local_pos_cb);
	ros::Subscriber home_pos_sub = nh.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 100, home_pos_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
	ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 100, waypoints_cb);
	ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 100, scan_cb);
	ros::Subscriber gps_sub = nh.subscribe("mavros/global_position/global", 100, gps_cb);
	ros::Subscriber heading_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 100, heading_cb);
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 100);

	ros::Rate rate(20.0);

	while (ros::ok() && !current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}

	while (ros::ok())
	{
		if (init_mask == 0x7f)
			break;

		ros::spinOnce();
		rate.sleep();
	}
	printf("init ok!\n");

	// Main part of the algorithm
	while (ros::ok())
	{
		while (ros::ok() && current_state.mode != "GUIDED")
		{
			ros::spinOnce();
			rate.sleep();
		}
		printf("guild ok\n");

		std::vector<geometry_msgs::PoseStamped> pose;
		printf("wp size=%d\n", waypoints.waypoints.size());
		for (int index = 0; index < waypoints.waypoints.size(); index++)
		{
			geometry_msgs::PoseStamped p;
			GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

			// Getting goal place by GPS
			Eigen::Vector3d goal_gps(waypoints.waypoints[index].x_lat, waypoints.waypoints[index].y_long, 0);

			// Calculating GPS point at ECEF coordinate considering earth rotation
			Eigen::Vector3d current_ecef;

			earth.Forward(current_gps.x(), current_gps.y(), current_gps.z(),

				current_ecef.x(), current_ecef.y(), current_ecef.z());

			Eigen::Vector3d goal_ecef;

			earth.Forward(goal_gps.x(), goal_gps.y(), goal_gps.z(),

				goal_ecef.x(), goal_ecef.y(), goal_ecef.z());

			Eigen::Vector3d ecef_offset = goal_ecef - current_ecef;

			Eigen::Vector3d enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, current_gps);

			Eigen::Affine3d sp;

			Eigen::Quaterniond q;

			q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())

				* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())

				* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

			sp.translation() = current_local_pos + enu_offset;

			sp.linear() = q.toRotationMatrix();

			Eigen::Vector3d testv(sp.translation());
			p.pose.position.x = testv[0];
			p.pose.position.y = testv[1];
			printf("%f %f\n", testv[0], testv[1]);
			pose.push_back(p);
		}

		for (int i = 1; i < pose.size(); i++)
		{
			while (ros::ok()) {
				local_pos_updated = false;
				scan_updated = false;
				heading_updated = false;

				while (ros::ok())
				{
					ros::spinOnce();
					if (local_pos_updated && scan_updated && heading_updated)
						break;
					rate.sleep();
				}

				if (current_state.mode != "GUIDED")
					break;

				// Successfully arrived at the goal position
				if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
					fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
				{
					break;
				}

				// Calculating Goal position
				Point2D goal;
				goal.x = pose[i].pose.position.x;
				goal.y = pose[i].pose.position.y;
				float direction = smap->CalculDirection(goal);

				if (direction >= -0.5)
				{
					if (direction > 180)
					{
						direction -= 360;
					}
					float arc = 3.1415 / 180 * direction;

					mavros_msgs::PositionTarget pos_target;
					pos_target.coordinate_frame = 1;
					pos_target.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
					pos_target.velocity.x = 0.5 * cos(arc);
					pos_target.velocity.y = 0.5 * sin(arc);
					local_pos_pub.publish(pos_target);

					ros::Time last_request = ros::Time::now();
					while (ros::ok()) {
						local_pos_updated = false;
						scan_updated = false;
						heading_updated = false;

						while (ros::ok())
						{
							ros::spinOnce();
							if (local_pos_updated && scan_updated && heading_updated)
								break;
							rate.sleep();
						}

						if (current_state.mode != "GUIDED")
							break;

						if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
							fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
						{
							break;
						}

						if (ros::Time::now() - last_request > ros::Duration(4.0))
							break;

						if (smap->IsFrontSafety() == false)
							break;

						local_pos_pub.publish(pos_target);
					}
				}
				else
				{
					// TOO DANGEROUS & STOP
					mavros_msgs::PositionTarget pos_target;
					pos_target.coordinate_frame = 1;
					pos_target.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
					pos_target.velocity.x = 0;
					pos_target.velocity.y = 0;
					local_pos_pub.publish(pos_target);
				}
			}
		}

		mavros_msgs::PositionTarget pos_target;
		pos_target.coordinate_frame = 1;
		pos_target.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
		pos_target.velocity.x = 0;
		pos_target.velocity.y = 0;
		local_pos_pub.publish(pos_target);

		printf("task over\n");
		while (ros::ok() && current_state.mode == "GUIDED")
		{
			ros::spinOnce();
			rate.sleep();
		}
	}

	return 0;
}
