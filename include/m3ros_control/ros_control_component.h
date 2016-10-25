#ifndef ROS_CONTROL_COMPONENT_H
#define ROS_CONTROL_COMPONENT_H

extern "C"
{
#include <stdio.h>
#include <signal.h>
#include <rtai_sched.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
}

////////// M3RT
#include <m3rt/base/component.h>
#include <m3rt/base/component_shm.h>
#include <m3rt/base/m3ec_def.h>
#include <m3rt/base/m3rt_def.h>
#include <m3rt/base/component_factory.h>

////////// Google protobuff
#include <google/protobuf/message.h>
#include "m3ros_control/ros_control_component.pb.h"

////////// ROS/ROS_CONTROL
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <realtime_tools/realtime_publisher.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/hardware_interface.h>

#include "m3ros_control/meka_robot_hw.h"
#include "m3ros_control/omnibase_ctrl.h"

////////// Activate some timing infos
//#define TIMING
#define NANO2SEC(a) a/1e9
#define SEC2NANO(a) a*1e9

static int tmp_dt_status_;
static int tmp_dt_cmd_;

//static long long start_dt_status_, end_dt_status_, elapsed_dt_status_;
//static long long start_dt_cmd_, end_dt_cmd_, elapsed_dt_cmd_;

#ifndef NDEBUG
#define TIME_ACTIVE 1
#else
#define TIME_ACTIVE 0
#endif

#define INIT_CNT(cnt) do { if (TIME_ACTIVE) (cnt) = 0; } while (0) 
#define SAVE_TIME(out) do { if (TIME_ACTIVE) getCpuCount((out)); } while (0)
#define PRINT_TIME(T_start,T_end,cnt,string) do { if (TIME_ACTIVE) if ((cnt)%100==0) ROS_INFO("%s: %fs",string,count2Sec(((T_end) - (T_start)))); cnt = cnt++ & INT_MAX;} while (0)

inline void getCpuCount(long long& out)
{
    out = nano2count(rt_get_cpu_time_ns());
}

inline double count2Sec(const long long in)
{
    return (NANO2SEC((double )count2nano(in)));
}

namespace ros_control_component
{

using namespace controller_manager;

class RosControlComponent: public m3rt::M3Component
{
public:
    RosControlComponent();
    ~RosControlComponent();

    google::protobuf::Message* GetCommand();
    google::protobuf::Message* GetStatus();
    google::protobuf::Message* GetParam();

    SEM *state_mutex_;
    ros::CallbackQueue* cb_queue_ptr; // Used to separate this node queue from the global one

    bool was_estop_;
    bool spinner_running_;


protected:
    bool LinkDependentComponents();
    void Startup();
    void Shutdown();
    bool ReadConfig(const char* filename);
    void StepStatus();
    void StepCommand();

    RosControlComponentStatus status_;
    RosControlComponentCommand cmd_;
    RosControlComponentParam param_;

    M3BaseStatus* GetBaseStatus();

    bool RosInit();
    void RosShutdown();

private:
    std::string bot_name_, zlift_name_, pwr_name_, obase_name_, obase_shm_name_, obase_jointarray_name_, hw_interface_mode_;
    bool ctrl_obase_;
    
    // acceptable errors between controller output and current state
    // to verify controllers were reset to current state
    double accept_ang_pos_;
    double accept_ang_vel_;
    double accept_torque_;
    double accept_lin_pos_;
    double accept_lin_vel_;
    double accept_force_;

    std::vector<std::string> controller_list_;
    m3::M3Humanoid* bot_shr_ptr_;
    m3::M3JointZLift* zlift_shr_ptr_;
    m3::M3Pwr* pwr_shr_ptr_;
    m3::M3Omnibase* obase_shr_ptr_;
    m3::M3OmnibaseShm* obase_shm_shr_ptr_;
    m3::M3JointArray* obase_ja_shr_ptr_;

    long rc,mrc;
    ros::Duration period_;
    ros::NodeHandle* ros_nh_ptr_, *ros_nh_ptr2_;
    ros::ServiceServer srv_;
    ros::AsyncSpinner* spinner_ptr_; // Used to keep alive the ros services in the controller manager
    realtime_tools::RealtimePublisher<m3meka_msgs::M3ControlStates> *realtime_pub_ptr_;

    MekaRobotHW* hw_ptr_;
    OmnibaseCtrl* obase_ptr_;

    controller_manager::ControllerManager* cm_ptr_;
    enum
    {
        DEFAULT
    };
    bool skip_loop_;
    long long loop_cnt_;

    void PreLoadControllers();
    void UnloadControllers();

    // calback function for the service
    bool changeStateCallback(m3meka_msgs::M3ControlStateChange::Request &req,
            m3meka_msgs::M3ControlStateChange::Response &res);
};

}

#endif

