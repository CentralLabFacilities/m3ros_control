/*
 * omnibase_hw.cpp
 *
 *  Created on: Feb 11, 2016
 *      Author: plueckin
 */

#include "m3ros_control/omnibase_ctrl.h"

#include <atomic>
#include <mutex>

#include <stdio.h>
#include <assert.h>
#include <rtai_shm.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <realtime_tools/realtime_publisher.h>

#include <m3/toolbox/toolbox.h>
#include "m3/vehicles/omnibase.pb.h"

#define RT_TASK_FREQUENCY_MEKA_OMNI_SHM 100
#define RT_TIMER_TICKS_NS_MEKA_OMNI_SHM (1000000000 / RT_TASK_FREQUENCY_MEKA_OMNI_SHM)      //Period of rt-timer

#define MEKA_ODOM_SHM "OSHMM"
#define MEKA_ODOM_CMD_SEM "OSHMC"
#define MEKA_ODOM_STATUS_SEM "OSHMS"
#define MEKA_OMNIBASE_SHM "OSHMP"

#define MEKA_ODOM_SHM_TO 100000
#define CYCLE_TIME_SEC 4
#define VEL_TIMEOUT_SEC 1.0

std::atomic<bool> ros_to_sds_ { false };
std::atomic<bool> sys_thread_active { false };
std::atomic<bool> sys_thread_end { false };

static M3OmnibaseShmSdsCommand cmd;
static M3OmnibaseShmSdsStatus status;
static int64_t last_cmd_ts;

nav_msgs::Odometry odom_g;
realtime_tools::RealtimePublisher<nav_msgs::Odometry>* realtime_pub;
ros::Subscriber cmd_sub_g;
boost::shared_ptr<tf::TransformBroadcaster> odom_broadcaster_ptr;

std::mutex rtai_to_ros_offset_mutex;
std::mutex rtai_to_shm_offset_mutex;

// master states

#define STATE_UNKNOWN   0
#define STATE_DISABLE   1
#define STATE_ENABLE    2
#define STATE_ESTOP     3
#define STATE_STANDBY   4
#define STATE_READY     5
#define STATE_RUNNING   6

// state transitions command

#define STATE_CMD_DISABLE   1
#define STATE_CMD_ENABLE    2
#define STATE_CMD_ESTOP     3
#define STATE_CMD_STOP      4
#define STATE_CMD_FREEZE    5
#define STATE_CMD_START     6

using namespace std;

namespace ros_control_component {

////////////////////////// TIMESTAMPS /////////////////////////////

void set_timestamp(int64_t timestamp) {
    cmd.timestamp = timestamp;
    return;
}

int64_t get_timestamp() {
    return status.timestamp;
}

////////////////////////// MAIN SDS DATA EXCHANGE METHODs///////////////////////
void step_ros(int cntr, ros::Duration & rtai_to_ros_offset) {

    if (!status.calibrated) {
        rt_printk("Omnibase is not calibrated. Please calibrate and run again. Omnibase control exiting.\n");
        sys_thread_end = true;
        ros_to_sds_ = false;
        return;
    }

    if (ros_to_sds_.load()) {
        set_timestamp(get_timestamp()); //Pass back timestamp as a heartbeat
    }

    if (rtai_to_ros_offset.toSec() == 0.0) {
        rt_printk("Time not in sync yet. Waiting to publish for next call.\n");
        return;
    }

    //reconstruct/guess wall time based on offsets calculated in the non-rt thread
    ros::Time rtai_now;
    rtai_to_ros_offset_mutex.lock();
    rtai_now.fromNSec(get_timestamp() * 1000L);
    ros::Time ros_now = rtai_now + rtai_to_ros_offset;
    rtai_to_ros_offset_mutex.unlock();

    // get from status
    double x = status.x;
    double y = status.y;
    double th = status.yaw;
    double vx = status.x_dot;
    double vy = status.y_dot;
    double vth = status.yaw_dot;
    //ROS_INFO("[STATUS] x,y,th:[%f,%f,%f]",x,y,th);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = ros_now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_ptr->sendTransform(odom_trans);

    //then, publish the pose and twist (movement) to ros
    odom_g.header.stamp = ros_now;
    odom_g.header.frame_id = "odom";
    //set the position
    odom_g.pose.pose.position.x = x;
    odom_g.pose.pose.position.y = y;
    odom_g.pose.pose.position.z = 0.0;
    odom_g.pose.pose.orientation = odom_quat;
    //set the velocity
    odom_g.child_frame_id = "base_link";
    odom_g.twist.twist.linear.x = vx;
    odom_g.twist.twist.linear.y = vy;
    odom_g.twist.twist.angular.z = vth;

    if (realtime_pub->trylock()) {
        realtime_pub->msg_ = odom_g;
        realtime_pub->unlockAndPublish();
    }

    if (status.timestamp - last_cmd_ts > VEL_TIMEOUT_SEC * 1000000.0) {
        cmd.x_velocity = 0.;
        cmd.y_velocity = 0.;
        cmd.yaw_velocity = 0.;
    }

    /* if (cntr % 100 == 0)
     {
     if (1)
     {
     printf("********************************\n");
     printf("timestamp: %ld\n", status.timestamp);
     {
     //printf("JOINT %d\n", i);
     printf("------------------------------\n");
     printf("X: %f\n", odom_g.pose.pose.position.x);
     printf("Y: %f\n", odom_g.pose.pose.position.y);
     printf("YAW: %f\n", th);
     printf("Vx: %f\n", odom_g.twist.twist.linear.x);
     printf("Vy: %f\n", odom_g.twist.twist.linear.y);
     printf("Va: %f\n", odom_g.twist.twist.angular.z);
     printf("------------------------------\n");
     printf("\n");
     }
     }
     }*/

}

void step_sds(const geometry_msgs::TwistConstPtr& msg) {

    if (ros_to_sds_.load()) {
        //put the movements by ros into the cmd struct of the meka
        cmd.x_velocity = msg->linear.x;
        cmd.y_velocity = msg->linear.y;
        cmd.yaw_velocity = msg->angular.z;

        last_cmd_ts = status.timestamp; //if the offset between actual time and this timestamp gets high enough, the base is turned off. see M3OmnibaseShm
    }
}

/**
 *
 * This function is run in a non rt thread to sync time with the rt system.
 *
 */
void update_rtai_to_ros_offset(ros::Duration & rtai_to_ros_offset, ros::Duration & rtai_to_shm_offset) {
    ros::Duration result, result_old;
    double smoothing = 0.95;
    ros::Duration rtai_to_shm_offset_copy;

    while (!sys_thread_end.load()) {

        // ~2Hz update rate
        usleep(500000);

        // Work with a copy to hold the mutex to access the offset only for the time of reading.
        // Sharing a mutex between rt and non rt might be bad but not holding it might be even worse.
        rtai_to_shm_offset_mutex.lock();
        rtai_to_shm_offset_copy = rtai_to_shm_offset;
        rtai_to_shm_offset_mutex.unlock();

        if (rtai_to_shm_offset_copy.toSec() == 0.0) {
            sleep(1);
            continue;
        }

        //NOTE: this call(ros::Time::now()) is not RT safe. This is why we do it in this Thread.
        ros::Duration ros_start_time(ros::Time::now().toSec());
        RTIME now_ns = rt_get_time_ns();

        ros::Duration rtai_start_time;
        rtai_start_time.fromNSec(now_ns);

        result_old = result;
        result = (ros_start_time - rtai_start_time) + rtai_to_shm_offset_copy;

        //apply a low pass filter on time changes to prevent bigger time jumps.
        if (result_old.toSec() != 0.0 && smoothing != 0.0) {
            result = result_old * smoothing + result * (1 - smoothing);
        }

        // again mutex only for writing.
        rtai_to_ros_offset_mutex.lock();
        rtai_to_ros_offset = result;
        rtai_to_ros_offset_mutex.unlock();
    }

    return static_cast<void>(0);

}

/**
 *
 * This function is responsible for the data exchange of the shared data structure in the m3 system.
 *
 */
void* rt_system_thread(void * arg) {

    SEM * status_sem;
    SEM * command_sem;
    RT_TASK * task;

    int cntr = 0;
    M3Sds * sds = (M3Sds *) arg;
    rt_printk("Starting ros sds real-time thread\n");

    const int sds_status_size = sizeof(M3OmnibaseShmSdsStatus);
    const int sds_cmd_size = sizeof(M3OmnibaseShmSdsCommand);

    memset(&cmd, 0, sds_cmd_size);

    task = rt_task_init_schmod(nam2num(MEKA_OMNIBASE_SHM), 3, 0, 0,
    SCHED_FIFO, 0xF);

    rt_allow_nonroot_hrt();

    if (task == NULL) {
        rt_printk("Failed to create RT-TASK %s \n", MEKA_OMNIBASE_SHM);
        return 0;
    }

    status_sem = (SEM*) rt_get_adr(nam2num(MEKA_ODOM_STATUS_SEM));

    if (!status_sem) {
        rt_printk("Unable to find the %s status semaphore.\n", MEKA_ODOM_STATUS_SEM);
        rt_task_delete(task);
        return 0;
    }

    command_sem = (SEM*) rt_get_adr(nam2num(MEKA_ODOM_CMD_SEM));

    if (!command_sem) {
        rt_printk("Unable to find the %s command semaphore.\n", MEKA_ODOM_CMD_SEM);
        rt_task_delete(task);
        return 0;
    }

    RTIME tick_period = nano2count(RT_TIMER_TICKS_NS_MEKA_OMNI_SHM);
    RTIME now = rt_get_time();

    /*
     * Explanations why and how we do the following time magic:
     *
     * Calling ros::Time::now() is not RT safe (and seems to cause crashes sometimes)
     *
     * Unfortuantely there seems to be no way to fetch a wall clock time in rtai.
     * Therefore we synchronize the time outside the rt system in a none rt thread.
     *
     * Unfortuantely the incoming time from the shm struct (which is in us by the way)
     * is not equal to this rtai time. There is an offset that correlates between system
     * bootup (or board calibration) and start of this thread. This leads to offset #2
     *
     * It remains the problem that we do not use the time of reading the encoder value to publish
     * but the current time. That shifts all timestamps a little bit into the future.
     */

    //here we store the offset described as #1
    ros::Duration rtai_to_ros_offset;
    ros::Duration rtai_to_shm_offset;
    std::thread t1(update_rtai_to_ros_offset, std::ref(rtai_to_ros_offset), std::ref(rtai_to_shm_offset));
    t1.detach(); // detach for rt. otherwise the task overruns on joining..
    rt_task_make_periodic(task, now + tick_period, tick_period);
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_hard_real_time();
    long long start_time, end_time, dt;
    long long step_cnt = 0;
    sys_thread_active = true;

    while (!sys_thread_end.load()) {
        //on system startup this shm timestamp is not initialized
        //if ((rtai_to_shm_offset.sec == 0) && (rtai_to_shm_offset.nsec == 0)){
        //wait until shm time has a value
        int64_t shm_time = get_timestamp();
        if (shm_time != 0.0) {
            //fetch offset between rtai time and shm timestamp as described as offset #2 above
            int64_t rt_time = rt_get_time_ns();
            rtai_to_shm_offset_mutex.lock();
            rtai_to_shm_offset.fromNSec(rt_time - (shm_time * 1000));
            rtai_to_shm_offset_mutex.unlock();
            //now handle this offset as well:
        }

        start_time = nano2count(rt_get_cpu_time_ns());
        rt_sem_wait(status_sem);
        memcpy(&status, sds->status, sds_status_size);
        rt_sem_signal(status_sem);

        step_ros(cntr, rtai_to_ros_offset);

        rt_sem_wait(command_sem);
        memcpy(sds->cmd, &cmd, sds_cmd_size);
        rt_sem_signal(command_sem);

        end_time = nano2count(rt_get_cpu_time_ns());
        dt = end_time - start_time;
        if (step_cnt % 50 == 0) {
            //WARNING: we are not sure if this printing is rt safe... seems to work but we do not know...
#if 0
            printf("rt time ns = %lld ns\n", rt_get_time_ns());
            printf("get timestamp = %ld us\n", get_timestamp());
            printf("ros time on start %f s\n", ros_start_time.toSec());
            printf("rtai time on start = %f s\n", rtai_start_time.toSec());
            printf("rtai_to_shm_offset %f s\n", rtai_to_shm_offset.toSec());
            printf("rtai_to_ros offset %f s\n", rtai_to_ros_offset.toSec());
            printf("sta[%ld us,%f,%f,%f]\n",status.timestamp,status.x,status.y,status.yaw);
            printf("cmd[%ld us,%f,%f,%f]\n",cmd.timestamp,cmd.x_velocity,cmd.y_velocity,cmd.yaw_velocity);
#endif
        }
        /*
         Check the time it takes to run components, and if it takes longer
         than our period, make us run slower. Otherwise this task locks
         up the CPU.*/
        if (dt > tick_period && step_cnt > 10) {
            //WARNING: we are not sure if this printing is rt safe... seems to work but we do not know...
            rt_printk("Step %lld: Computation time of components is too long..\n", step_cnt);
            rt_printk("Previous period: %f. New period: %f\n", (double) count2nano(tick_period),
                    (double) count2nano(dt));

            tick_period = dt;
            //rt_task_make_periodic(task, end + tick_period,tick_period);
        }
        step_cnt++;
        if (cntr++ == CYCLE_TIME_SEC * 2 * RT_TIMER_TICKS_NS_MEKA_OMNI_SHM) {
            cntr = 0;
        }
        rt_task_wait_period();
    }
    rt_printk("Exiting RealTime Thread...\n");
    rt_make_soft_real_time();
    rt_task_delete(task);
    sys_thread_active = false;
    rt_printk("Success!\n");
    return static_cast<void *>(0);
}

/////////////////////// NON-MEMBER THREADING STUFF END /////////////////////////

OmnibaseCtrl::OmnibaseCtrl(m3::M3Omnibase* obase_shr_ptr, m3::M3OmnibaseShm* obase_shm_shr_ptr,
        m3::M3JointArray* obase_ja_shr_ptr, std::string nodename, BASE_CTRL_MODE mode) :
        obase_shr_ptr_(NULL), obase_shm_shr_ptr_(NULL), obase_ja_shr_ptr_(NULL), obase_vctrl_shr_ptr_(NULL), sys(NULL), ros_nh_ptr_(
        NULL), running(false), name("base"), node_name(nodename), ctrl_state(STATE_DISABLE), enabled(false), hst(-1), ctrl_mode(
                mode), max_lin(0.3), max_ang(1.0), diag_i(0) {

    assert(obase_shr_ptr != NULL);
    assert(obase_shm_shr_ptr != NULL);
    assert(obase_ja_shr_ptr != NULL);

    obase_shr_ptr_ = obase_shr_ptr;
    obase_shm_shr_ptr_ = obase_shm_shr_ptr;
    obase_ja_shr_ptr_ = obase_ja_shr_ptr;

    m3rt::M3_INFO("%s: Initializing dispatch thread..\n", name.c_str());

    detach_sds_th_ = std::thread(&OmnibaseCtrl::init_sds, this);
}

OmnibaseCtrl::OmnibaseCtrl(m3::MekaOmnibaseControl* obase_vctrl_shr_ptr, std::string nodename, BASE_CTRL_MODE mode) :
        obase_shr_ptr_(NULL), obase_shm_shr_ptr_(NULL), obase_ja_shr_ptr_(NULL), obase_vctrl_shr_ptr_(NULL), sys(NULL), ros_nh_ptr_(
        NULL), running(false), name("base"), node_name(nodename), ctrl_state(STATE_DISABLE), enabled(false), hst(-1), ctrl_mode(
                mode), max_lin(0.3), max_ang(1.0), diag_i(0) {

    assert(obase_vctrl_shr_ptr != NULL);

    obase_vctrl_shr_ptr_ = obase_vctrl_shr_ptr;

    m3rt::M3_INFO("%s: Initializing bridge to omnibase velocity controller..\n", name.c_str());

    bridge_th_ = std::thread(&OmnibaseCtrl::init_vctrl_bridge, this);
}

OmnibaseCtrl::~OmnibaseCtrl() {
    if (ros_nh_ptr_ != NULL)
        delete ros_nh_ptr_;
}

bool OmnibaseCtrl::is_running() {
    if (running) {
        return true;
    } else {
        return false;
    }
}

bool OmnibaseCtrl::is_sds_ended() {
    if (sys_thread_end.load()) {
        return true;
    } else {
        return false;
    }
}

bool OmnibaseCtrl::is_sds_active() {
    if (sys_thread_active.load()) {
        return true;
    } else {
        return false;
    }
}

void OmnibaseCtrl::enable_ros2sds() {
    if (!ros_to_sds_.load()) {
        m3rt::M3_INFO("%s: Enabling ROS2SDS!\n", name.c_str());
        ros_to_sds_ = true;
    }
}

void OmnibaseCtrl::disable_ros2sds() {
    if (ros_to_sds_.load()) {
        m3rt::M3_INFO("%s: Disabling ROS2SDS!\n", name.c_str());
        ros_to_sds_ = false;
    }
}

void OmnibaseCtrl::shutdown() {
    switch (ctrl_mode) {
    case (STD):
        if (hst) {
            sys_thread_end = true;
            rt_thread_join(hst);
            rt_shm_free(nam2num(MEKA_ODOM_SHM));
            hst = -1;
        }
        detach_sds_th_.join(); //should already be ended. anyways...
        m3rt::M3_INFO("SDS thread ended.\n");
        break;
    case (VCTRL):
        bridge_th_.join();
        m3rt::M3_INFO("VCTRL thread ended.\n");
        break;
    }
    running = false;
}

void OmnibaseCtrl::init_sds() {
    while (1) {
        M3OmnibaseStatus *status = (M3OmnibaseStatus*) obase_shr_ptr_->GetStatus();
        int64_t timestamp = (status->mutable_base())->timestamp();
        if (timestamp) {
            m3rt::M3_INFO("Timestamp found, initializing SDS for Omnibase control in 5 seconds.\n");
            sleep(5);
            break;
        }
        sleep(1);
    }

    ros::AsyncSpinner spinner(1); // Use 1 thread - check if you actually need this for only publishing
    spinner.start();

    ros_nh_ptr_ = new ros::NodeHandle(node_name + "_base_controller");

    odom_broadcaster_ptr.reset(new tf::TransformBroadcaster);
    cmd_sub_g = ros_nh_ptr_->subscribe("cmd_vel", 1, step_sds);
    //ros_nh_ptr_->advertise<nav_msgs::Odometry>("odom", 1);
    realtime_pub = new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(*ros_nh_ptr_, "odom", 1);

    rt_allow_nonroot_hrt();
    // Create a rt thread for ros omnibase control via sds
    sys = (M3Sds*) rt_shm_alloc(nam2num(MEKA_ODOM_SHM), sizeof(M3Sds), USE_VMALLOC);
    if (sys) {
        m3rt::M3_INFO("Found shared memory starting omnibase control via shared data service.\n");
        rt_allow_nonroot_hrt();
        hst = rt_thread_create((void*) rt_system_thread, sys, 10000);
        running = true;
    } else {
        m3rt::M3_ERR("Rtai_malloc failure for %s\n", MEKA_ODOM_SHM);
    }

    return static_cast<void>(0);
}

void OmnibaseCtrl::init_vctrl_bridge() {
    // ROS stuff
    ros_nh_ptr_ = new ros::NodeHandle(node_name + "_base_controller");

    cmdvel_sub = ros_nh_ptr_->subscribe("smooth_cmd_vel", 1, boost::bind(&OmnibaseCtrl::cmd_vel_cb, this, _1));
    odom_pub = ros_nh_ptr_->advertise<nav_msgs::Odometry>("odom", 1);

    ros::param<double>("~max_lin_vel", max_lin, 0.30);
    ros::param<double>("~max_ang_vel", max_ang, 1.00);
    last_cmd = ros::Time(0);
    timeout = ros::Duration(0.25);

    ros::Rate loop_rate = ros::Rate(50);

    running = true;

    while (true) {
        if (ros_to_sds_.load()) { //TODO: renaming..
            vctrl_step();
        }
        loop_rate.sleep();
    }

    return static_cast<void>(0);
}

void OmnibaseCtrl::vctrl_step() {

    ros::Time now = ros::Time::now();
    ros::Time elapsed = now - last_cmd;

    // Make the transform available in the 'future' to avoid extrapolation errors
    // Same thing AMCL and robot_state_publisher for static transform do.
    ros::Time odom_time = now;

    //rospy.logdebug("Time elapsed since last command: " + str(elapsed.to_sec()))
    if (elapsed > timeout) {
        // Too much time passed since last command, zero it.
        last_cmd_vel = geometry_msgs::Twist();
    }

    double xd = last_cmd_vel.linear.x;
    double yd = last_cmd_vel.linear.y;
    double td = last_cmd_vel.angular.z;

    double xda = fabs(last_cmd_vel.linear.x);
    double yda = fabs(last_cmd_vel.linear.y);
    double tda = fabs(last_cmd_vel.angular.z);

    // Saturate according to velocity limits:
    if (xda >= max_lin) {
        double xds = last_cmd_vel.linear.x / xda;
        xd = xds * max_lin;
    }
    if (yda > max_lin) {
        double yds = last_cmd_vel.linear.y / yda;
        yd = yds * max_lin;
    }
    if (tda > max_ang) {
        double tds = last_cmd_vel.angular.z / tda;
        td = tds * max_ang;
    }

    obase_vctrl_shr_ptr_->SetStateOp();
    obase_vctrl_shr_ptr_->GetCommand()->set_xd_des(0,xd);
    obase_vctrl_shr_ptr_->GetCommand()->set_xd_des(1,yd);
    obase_vctrl_shr_ptr_->GetCommand()->set_xd_des(2,td);

    for(uint i=0; obase_vctrl_shr_ptr_->GetCommand()->tqr_size(); i++) {
        obase_vctrl_shr_ptr_->GetCommand()->set_tqr(i, 1.0);
    }

    // Odometry
    double x = obase_vctrl_shr_ptr_->GetStatus()->g_pos(0);
    double y = obase_vctrl_shr_ptr_->GetStatus()->g_pos(1);
    double t = obase_vctrl_shr_ptr_->GetStatus()->g_pos(2);

    double oxd = obase_vctrl_shr_ptr_->GetStatus()->l_vel(0);
    double oyd = obase_vctrl_shr_ptr_->GetStatus()->l_vel(1);
    double otd = obase_vctrl_shr_ptr_->GetStatus()->l_vel(2);

    tf::Quaternion q = tf::createQuaternionFromRPY(0,0,t);

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    transform.setRotation(q);

    tf_bc.sendTransform(tf::StampedTransform(transform, odom_time, "base_link", "odom"));

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp = odom_time;
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = q;

    odom.twist.twist.linear.x = oxd;
    odom.twist.twist.linear.y = oyd;
    odom.twist.twist.angular.z = otd;

    odom_pub.publish(odom);

    diag_i++;
    if (diag_i >= 5) {
        std::vector<double> alpha;
        std::vector<double> beta;
        std::vector<double> beta_d;
        std::vector<double> phid;
        std::copy(obase_vctrl_shr_ptr_->GetStatus()->alpha().begin(), obase_vctrl_shr_ptr_->GetStatus()->alpha().end(), std::front_inserter(alpha));
        std::copy(obase_vctrl_shr_ptr_->GetStatus()->beta().begin(), obase_vctrl_shr_ptr_->GetStatus()->beta().end(), std::front_inserter(beta));
        std::copy(obase_vctrl_shr_ptr_->GetStatus()->beta_d().begin(), obase_vctrl_shr_ptr_->GetStatus()->beta_d().end(), std::front_inserter(beta_d));
        std::copy(obase_vctrl_shr_ptr_->GetStatus()->phid().begin(), obase_vctrl_shr_ptr_->GetStatus()->phid().end(), std::front_inserter(phid));
        m3rt::M3_INFO("alpha: %d %d %d %d\n", alpha[0],alpha[1],alpha[2],alpha[3]);
        m3rt::M3_INFO("beta: %d %d %d %d\n", beta[0],beta[1],beta[2],beta[3]);
        m3rt::M3_INFO("beta_d: %d %d %d %d\n", beta_d[0],beta_d[1],beta_d[2],beta_d[3]);
        m3rt::M3_INFO("phid: %d %d %d %d\n", phid[0],phid[1],phid[2],phid[3]);

        diag_i = 0;

        std::vector<double> l;
        std::copy(obase_vctrl_shr_ptr_->GetStatus()->l().begin(), obase_vctrl_shr_ptr_->GetStatus()->l().end(), std::front_inserter(l));

        for(int i : l.size()) {
            double alph = obase_vctrl_shr_ptr_->GetStatus()->alpha(i);
            double bet = obase_vctrl_shr_ptr_->GetStatus()->beta(i);
            double xx = l.at(i) * std::cos(alph);
            double yy = l.at(i) * std::sin(alph);

            tf::Quaternion qq = tf::createQuaternionFromRPY(0,0,alph + bet - M_PI/2.0);

            tf::Transform transform;
            transform.setOrigin( tf::Vector3(xx, yy, 0.0) );
            transform.setRotation(qq);

            tf_bc.sendTransform(tf::StampedTransform(transform, odom_time, "caster_"+std::str(i), "odom"));

            ros::Duration(0.01).sleep();
        }
    }


}

void OmnibaseCtrl::cmd_vel_cb(geometry_msgs::Twist cmd_vel) {
    last_cmd_vel = cmd_vel;
    last_cmd = ros::Time::now();
}

void OmnibaseCtrl::getPublishableState(m3meka_msgs::M3ControlStates &msg) {
    //always last element. change when integrate omnibasectrl in mekarobothw
    msg.group_name.back() = name;
    if (enabled)
        msg.state.back() = ctrl_state;
    else
        msg.state.back() = STATE_DISABLE;
}

int OmnibaseCtrl::changeState(const int state_cmd) {
    int ret = 0;
    if (!is_sds_active()) { //base sds thread is not active anyways..
        //m3rt::M3_INFO("%s: SDS is not active!\n", name.c_str());
        ret = -3;
    } else {
        switch (state_cmd) {
        case STATE_CMD_DISABLE:
            enabled = false;
            if (ctrl_state == STATE_RUNNING) {
                disable_ros2sds();
                m3rt::M3_INFO("%s: You should switch controllers off !\n", name.c_str());
            }

            if (ctrl_state != STATE_ESTOP) {
                m3rt::M3_INFO("%s: Disabled but in ctrl state Standby \n", name.c_str());
                ctrl_state = STATE_STANDBY;
            }
            break;
        case STATE_CMD_ENABLE:
            if (!enabled)
                enabled = true;
            break;
        case STATE_CMD_ESTOP:
            if (enabled) {
                if (ctrl_state == STATE_RUNNING)
                    disable_ros2sds();
            }
            if (ctrl_state != STATE_ESTOP)
                m3rt::M3_INFO("%s: ESTOP detected\n", name.c_str());
            ctrl_state = STATE_ESTOP;
            break;
        case STATE_CMD_STOP:
            if (ctrl_state == STATE_RUNNING)
                disable_ros2sds();
            ctrl_state = STATE_STANDBY;
            if (enabled)
                m3rt::M3_INFO("%s: in standby state\n ", name.c_str());
            break;
        case STATE_CMD_FREEZE: //no freeze for base.
            if (enabled) {
                if (ctrl_state == STATE_ESTOP)
                    ret = -3;
                else {
                    disable_ros2sds();
                    ctrl_state = STATE_STANDBY;
                    m3rt::M3_INFO("%s: in standby state\n ", name.c_str());
                }
            }
            break;
        case STATE_CMD_START:
            if (enabled) {
                if (ctrl_state != STATE_RUNNING) {
                    // cannot go to run if previously in e-stop
                    // go to standby first
                    if (ctrl_state == STATE_ESTOP) {
                        ret = -3;
                        break;
                    }
                    enable_ros2sds();
                    ctrl_state = STATE_RUNNING;
                    m3rt::M3_INFO("%s: putting in running state\n ", name.c_str());
                    break;
                }
            }
        }
    }
    return ret;
    }
}

