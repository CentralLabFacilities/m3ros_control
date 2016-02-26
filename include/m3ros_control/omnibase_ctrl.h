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

#include <thread>
#include <ros/ros.h>

#ifndef M3ROS_CONTROL_OMNIBASE_CTRL_H_
#define M3ROS_CONTROL_OMNIBASE_CTRL_H_

namespace ros_control_component {

class OmnibaseCtrl {
public:

	enum BASE_CTRL_MODE {
		SDS, //shared data service
		SHR  //shared pointer
	};

	OmnibaseCtrl(m3::M3Omnibase* obase_shr_ptr, m3::M3OmnibaseShm* obase_shm_shr_ptr, m3::M3JointArray* obase_ja_shr_ptr, std::string nodename, BASE_CTRL_MODE mode = SDS);

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
    M3Sds* sys; //dunno why this is not in m3 namespace..

    bool running;
    std::string name, node_name;
    int ctrl_state;
    long hst;
    BASE_CTRL_MODE ctrl_mode;

	ros::NodeHandle* ros_nh_ptr_;

	std::thread detach_sds_th_;

	void init_sds();

};

}



#endif /* M3ROS_CONTROL_OMNIBASE_CTRL_H_ */
