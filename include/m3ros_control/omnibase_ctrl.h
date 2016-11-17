/*
 * omnibase_ctrl.h
 *
 *  Created on: Feb 11, 2016
 *      Author: plueckin
 */

////////// M3
#include <m3/vehicles/omnibase.h>
#include <m3/vehicles/omnibase_shm.h>
#include <m3/vehicles/omnibase_shm_sds.h>

#include <m3meka_msgs/M3ControlStates.h>
#include <m3/meka_omnibase_control/meka_omnibase_control.hpp>

#include <thread>
#include <ros/ros.h>

#ifndef M3ROS_CONTROL_OMNIBASE_CTRL_H_
#define M3ROS_CONTROL_OMNIBASE_CTRL_H_

namespace ros_control_component {

class OmnibaseCtrl {
public:

	enum BASE_CTRL_MODE {
	    STD, // standard. omnibase control via shared data service
		VCTRL // velocity controlled via meka omnibase control
	};

	OmnibaseCtrl(m3::M3Omnibase* obase_shr_ptr, m3::M3OmnibaseShm* obase_shm_shr_ptr, m3::M3JointArray* obase_ja_shr_ptr, std::string nodename, BASE_CTRL_MODE mode = STD);
	OmnibaseCtrl(m3_obase_ctrl::MekaOmnibaseControl* obase_vctrl_shr_ptr, std::string nodename, BASE_CTRL_MODE mode = VCTRL);

	~OmnibaseCtrl();

	void shutdown();

	// try change the state to requested state_cmd
    int changeState(const int state_cmd);

	void getPublishableState(m3meka_msgs::M3ControlStates &msg);

	bool is_running();
	bool is_sds_ended();

protected:

private:

	bool is_sds_active();
    void enable_ros2sds();
    void disable_ros2sds();

	m3::M3Omnibase* obase_shr_ptr_;
	m3::M3OmnibaseShm* obase_shm_shr_ptr_;
	m3::M3JointArray* obase_ja_shr_ptr_;
	m3_obase_ctrl::MekaOmnibaseControl* obase_vctrl_shr_ptr_;
    M3Sds* sys; //not in m3 namespace

    ros::NodeHandle* ros_nh_ptr_;

    ros::Publisher odom_pub;
    ros::Subscriber cmdvel_sub;

    bool running;
    std::string name, node_name;
    int ctrl_state;
    bool enabled;
    long hst;

    BASE_CTRL_MODE ctrl_mode;

	std::thread detach_sds_th_;
	std::thread bridge_th_;

	void init_sds();
	void init_vctrl_bridge();
	void cmd_vel_cb(geometry_msgs::Twist cmd_vel);
	void vctrl_step();

	double max_lin;
	double max_ang;
	int diag_i;

	geometry_msgs::Twist last_cmd_vel;
	ros::Time last_cmd;
	ros::Duration timeout;

	static tf::TransformBroadcaster tf_bc;

};

}



#endif /* M3ROS_CONTROL_OMNIBASE_CTRL_H_ */
