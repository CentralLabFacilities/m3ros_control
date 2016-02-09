#include "m3ros_control/ros_control_component.h"

#include <ctime>
#include <mutex>
#include <thread>
#include <rtai_shm.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define RT_TASK_FREQUENCY_MEKA_OMNI_SHM 100
#define RT_TIMER_TICKS_NS_MEKA_OMNI_SHM (1000000000 / RT_TASK_FREQUENCY_MEKA_OMNI_SHM)      //Period of rt-timer

#define MEKA_ODOM_SHM "OSHMM"
#define MEKA_ODOM_CMD_SEM "OSHMC"
#define MEKA_ODOM_STATUS_SEM "OSHMS"
#define MEKA_OMNIBASE_SHM "OSHMP"

#define ROS_ASYNC_SP "ROSSPI"
#define ROS_MAIN_ASYNC_SP "ROSMSP"

#define CYCLE_TIME_SEC 4
#define VEL_TIMEOUT_SEC 1.0

// make shared global volatile ?
static int ros_to_sds_ = 0;
static int end_sds_ = 0;
static int sys_thread_active = 0;
static int sys_thread_end = 0;
static int sds_status_size;
static int sds_cmd_size;
static M3OmnibaseShmSdsCommand cmd;
static M3OmnibaseShmSdsStatus status;
static int64_t last_cmd_ts;

nav_msgs::Odometry odom_g;
ros::Publisher odom_publisher_g;
ros::Subscriber cmd_sub_g;
boost::shared_ptr<tf::TransformBroadcaster> odom_broadcaster_ptr;

static void end_sds_th(int dummy) {
    std::cout << "END SDS THREAD\n";
    end_sds_ = 1;
}

std::mutex ros_to_sds_mtx;
std::mutex rtai_to_ros_offset_mutex;
std::mutex rtai_to_shm_offset_mutex;

namespace ros_control_component {

using namespace m3rt;
using namespace std;
using namespace m3;

////////////////////////// NON-MEMBER THREADING STUFF /////////////////////////

// asynchronous spinner for ROS implemented as an rt_task to access rt mutex
// has priority one lower than the current task
// the service used in the component accesses a mutex of the higher priority task
// there might be priority inversions to take care of.
void *ros_async_spinner(void * arg) {
    RosControlComponent * ros_comp_ptr = (RosControlComponent *) arg;
    int prio = ros_comp_ptr->GetPriority();

    m3rt::M3_INFO("Starting async spinner thread with priority %d.\n", prio);

    RT_TASK *task;
    task = rt_task_init_schmod(nam2num(ROS_ASYNC_SP), prio, 0, 0, SCHED_FIFO,
            0xFF);
    if (task == NULL) {
        m3rt::M3_ERR("Failed to create RT-TASK ROSSPI\n", 0);
        return 0;
    }

    rt_allow_nonroot_hrt();
    // should not be done twice ?
    // mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_soft_real_time();

    // create a mutex for the state resource
    ros_comp_ptr->state_mutex_ = rt_typed_sem_init(nam2num("MUTEX"), 1, BIN_SEM);

    while (ros_comp_ptr->spinner_running_) {
        // call all the cb from the callback queue
        ros_comp_ptr->cb_queue_ptr->callAvailable(ros::WallDuration());
        rt_sleep(nano2count(500000000));
    }

    // destroy the mutex
    if (ros_comp_ptr->state_mutex_ != NULL) {
        rt_sem_delete(ros_comp_ptr->state_mutex_);
    }

    rt_task_delete(task);

    return static_cast<void *>(0);
}

// asynchronous spinner for ROS implemented as an rt_task to avoid problems of AsyncSpinner
// has priority 2 lower than the current task
void *rosmain_async_spinner(void * arg) {
    RosControlComponent * ros_comp_ptr = (RosControlComponent *) arg;
    int prio = std::max(1, ros_comp_ptr->GetPriority() - 2);

    m3rt::M3_INFO("Starting main async spinner thread with priority %d.\n",
            prio);

    RT_TASK *task;
    task = rt_task_init_schmod(nam2num(ROS_MAIN_ASYNC_SP), prio, 0, 0,
            SCHED_FIFO, 0xFF);
    if (task == NULL) {
        m3rt::M3_ERR("Failed to create RT-TASK ROSMSP\n", 0);
        return 0;
    }

    rt_allow_nonroot_hrt();
    // should not be done twice ?
    //mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_soft_real_time();

    while (ros_comp_ptr->spinner_running_) {
        // call all the cb from the callback queue
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
        rt_sleep(nano2count(100000000));
    }

    rt_task_delete(task);

    return static_cast<void *>(0);
}

////////////////////////// TIMESTAMPS /////////////////////////////

void SetTimestamp(int64_t timestamp) {
    cmd.timestamp = timestamp;
    return;
}

int64_t GetTimestamp() {
    return status.timestamp;
}

////////////////////////// MAIN COMPUTATION METHOD /////////////////////////////
//  integrate all this this into meka_robot_hw? or own class?
void stepRos(int cntr, ros::Duration & rtai_to_ros_offset) {
    SetTimestamp(GetTimestamp()); //Pass back timestamp as a heartbeat

    if (!status.calibrated) {
        printf(
                "Omnibase is not calibrated. Please calibrate and run again. Exiting.\n");
        end_sds_th(1);
    }
    if (rtai_to_ros_offset.toSec() == 0.0) {
        //printf("Time not in sync yet. Waiting to publish for next call.\n");
        return;
    }

    //reconstruct/guess wall time based on offsets calculated in the non-rt thread
    ros::Time rtai_now;
    rtai_to_ros_offset_mutex.lock();
    rtai_now.fromNSec(GetTimestamp() * 1000L);
    ros::Time ros_now = rtai_now + rtai_to_ros_offset;
    rtai_to_ros_offset_mutex.unlock();

    odom_g.header.stamp = ros_now;

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
    odom_publisher_g.publish(odom_g);

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

void stepSds(const geometry_msgs::TwistConstPtr& msg) {

    ros_to_sds_mtx.lock();
    if(ros_to_sds_) {
        //put the movements by ros into the cmd struct of the meka
        cmd.x_velocity = msg->linear.x;
        cmd.y_velocity = msg->linear.y;
        cmd.yaw_velocity = msg->angular.z;

        printf("x: %f\n", cmd.x_velocity);
        printf("y: %f\n", cmd.y_velocity);
        printf("a: %f\n", cmd.yaw_velocity);

        last_cmd_ts = status.timestamp; //if the offset between actual time and this timestamp gets high enough, the base is turned off. see M3OmnibaseShm
    }
    ros_to_sds_mtx.unlock();
}

/**
 *
 * This function is run in a non rt thread to sync time with the rt system.
 *
 */
void update_rtai_to_ros_offset(ros::Duration & rtai_to_ros_offset,
        ros::Duration & rtai_to_shm_offset) {
    ros::Duration result, result_old;
    double smoothing = 0.95;
    ros::Duration rtai_to_shm_offset_copy;

    while (true) {

        // ~2Hz update rate
        usleep(500000);

        // Work with a copy to hold the mutex to access the offset only for the time of reading.
        // Sharing a mutex between rt and non rt might be bad but not holding it might be even worse.
        rtai_to_shm_offset_mutex.lock();
        rtai_to_shm_offset_copy = rtai_to_shm_offset;
        rtai_to_shm_offset_mutex.unlock();

        if (rtai_to_shm_offset_copy.toSec() == 0.0) {
            //TODO: Error print: offset not yet available
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
}

/**
 *
 * This function is responsible for the data exchange of the shared data structure in the m3 system.
 *
 */
void* rt_system_thread(void * arg) {

    rt_allow_nonroot_hrt();

    int cntr = 0;
    M3Sds * sds = (M3Sds *) arg;
    printf("Starting ros sds real-time thread\n");

    sds_status_size = sizeof(M3OmnibaseShmSdsStatus);
    sds_cmd_size = sizeof(M3OmnibaseShmSdsCommand);

    memset(&cmd, 0, sds_cmd_size);

    RT_TASK * task = rt_task_init_schmod(nam2num(MEKA_OMNIBASE_SHM), 3, 0, 0, SCHED_FIFO,
            0xF);

    if (task == NULL) {
        printf("Failed to create RT-TASK OSHMP\n");
        return 0;
    }

    SEM * status_sem = (SEM*) rt_get_adr(nam2num(MEKA_ODOM_STATUS_SEM));
    SEM * command_sem = (SEM*) rt_get_adr(nam2num(MEKA_ODOM_CMD_SEM));

    if (!status_sem) {
        printf("Unable to find the %s semaphore.\n", MEKA_ODOM_STATUS_SEM);
        rt_task_delete(task);
        return 0;
    }
    if (!command_sem) {
        printf("Unable to find the %s semaphore.\n", MEKA_ODOM_CMD_SEM);
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
    std::thread t1(update_rtai_to_ros_offset, std::ref(rtai_to_ros_offset),
            std::ref(rtai_to_shm_offset));

    rt_task_make_periodic(task, now + tick_period, tick_period);
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_hard_real_time();
    long long start_time, end_time, dt;
    long long step_cnt = 0;
    sys_thread_active = 1;

    while (!sys_thread_end) {
        //on system startup this shm timestamp is not initialized
        //if ((rtai_to_shm_offset.sec == 0) && (rtai_to_shm_offset.nsec == 0)){
        //wait until shm time has a value
        int64_t shm_time = GetTimestamp();
        if (shm_time != 0.0) {
            //fetch offset between rtai time and shm timestamp as described as offset #2 above
            int64_t rt_time = rt_get_time_ns();
            rtai_to_shm_offset_mutex.lock();
            rtai_to_shm_offset.fromNSec(rt_time - (shm_time * 1000));
            rtai_to_shm_offset_mutex.unlock();
            //now handle this offset as well:
        }
        //}

        start_time = nano2count(rt_get_cpu_time_ns());
        rt_sem_wait(status_sem);
        memcpy(&status, sds->status, sds_status_size);
        rt_sem_signal(status_sem);

        stepRos(cntr, rtai_to_ros_offset);

        rt_sem_wait(command_sem);
        memcpy(sds->cmd, &cmd, sds_cmd_size);
        rt_sem_signal(command_sem);

        end_time = nano2count(rt_get_cpu_time_ns());
        dt = end_time - start_time;
        if (step_cnt % 50 == 0) {
            //WARNING: we are not sure if this printing is rt safe... seems to work but we do not know...
#if 0
            printf("rt time ns = %lld ns\n", rt_get_time_ns());
            printf("get timestamp = %ld us\n", GetTimestamp());
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
            printf(
                    "Step %lld: Computation time of components is too long. Forcing all components to state SafeOp.\n",
                    step_cnt);
            printf("Previous period: %f. New period: %f\n",
                    (double) count2nano(tick_period), (double) count2nano(dt));

            tick_period = dt;
            //rt_task_make_periodic(task, end + tick_period,tick_period);
        }
        step_cnt++;
        if (cntr++ == CYCLE_TIME_SEC * 2 * RT_TIMER_TICKS_NS_MEKA_OMNI_SHM) {
            cntr = 0;
        }
        rt_task_wait_period();
    }
    printf("Exiting RealTime Thread...\n");
    rt_make_soft_real_time();
    rt_task_delete(task);
    sys_thread_active = 0;
    return 0;
}

/////////////////////// NON-MEMBER THREADING STUFF END /////////////////////////

RosControlComponent::RosControlComponent() :
        m3rt::M3Component(MAX_PRIORITY), state_mutex_(NULL), spinner_running_(
                false), was_estop_(true), cb_queue_ptr(NULL), accept_ang_pos_(
                0.0), accept_ang_vel_(0.0), accept_torque_(0.0), accept_lin_pos_(
                0.0), accept_lin_vel_(0.0), accept_force_(0.0), bot_shr_ptr_(
        NULL), zlift_shr_ptr_(NULL), pwr_shr_ptr_(NULL), ros_nh_ptr_(
        NULL), ros_nh_ptr2_(NULL), spinner_ptr_(NULL), realtime_pub_ptr_(
        NULL), hw_ptr_(NULL), cm_ptr_(NULL), skip_loop_(false), loop_cnt_(0) {
    RegisterVersion("default", DEFAULT);
}

RosControlComponent::~RosControlComponent() {

    if (cm_ptr_ != NULL)
        delete cm_ptr_;

    if (realtime_pub_ptr_ != NULL)
        delete realtime_pub_ptr_;

    if (hw_ptr_ != NULL)
        delete hw_ptr_;

    if (spinner_ptr_ != NULL)
        delete spinner_ptr_;

    if (ros_nh_ptr_ != NULL)
        delete ros_nh_ptr_;

    if (ros_nh_ptr2_ != NULL)
        delete ros_nh_ptr2_;

    if (cb_queue_ptr != NULL)
        delete cb_queue_ptr;

}

google::protobuf::Message* RosControlComponent::GetCommand() {
    return &status_;
}
google::protobuf::Message* RosControlComponent::GetStatus() {
    return &cmd_;
}
google::protobuf::Message* RosControlComponent::GetParam() {
    return &param_;
}

bool RosControlComponent::LinkDependentComponents() {
    bot_shr_ptr_ = (m3::M3Humanoid*) factory->GetComponent(bot_name_);
    if (bot_shr_ptr_ == NULL) {
        m3rt::M3_INFO("M3Humanoid component %s not found for component %s\n",
                bot_name_.c_str(), GetName().c_str());
        return false;
    }
    zlift_shr_ptr_ = (m3::M3JointZLift*) factory->GetComponent(zlift_name_);
    if (zlift_shr_ptr_ == NULL) {
        m3rt::M3_INFO("M3JointZLift component %s not found for component %s\n",
                zlift_name_.c_str(), GetName().c_str());
        return false;
    }
    pwr_shr_ptr_ = (m3::M3Pwr*) factory->GetComponent(pwr_name_);
    if (pwr_shr_ptr_ == NULL) {
        m3rt::M3_INFO("M3Pwr component %s not found for component %s\n",
                pwr_name_.c_str(), GetName().c_str());
        return false;
    }

    return true;
}

void RosControlComponent::Startup() {
    period_.fromSec(1.0 / static_cast<double>(RT_TASK_FREQUENCY));

    if (!RosInit(bot_shr_ptr_, zlift_shr_ptr_)) //NOTE here the bot_shr_ptr_ is correctly loaded
        skip_loop_ = true;
    INIT_CNT(tmp_dt_status_);
    INIT_CNT(tmp_dt_cmd_);

    SetStateSafeOp();
}

void RosControlComponent::Shutdown() {
    RosShutdown();
}

bool RosControlComponent::ReadConfig(const char* cfg_filename) {
    if (!M3Component::ReadConfig(cfg_filename))
        return false;
    doc["humanoid"] >> bot_name_;
    doc["zlift"] >> zlift_name_;
    doc["pwr_component"] >> pwr_name_;
    doc["hw_interface_mode"] >> hw_interface_mode_;

    if (hw_interface_mode_ == "effort" || hw_interface_mode_ == "position")
        M3_INFO("Selected hardware interface mode %s for component %s\n",
                hw_interface_mode_.c_str(), GetName().c_str());
    else {
        M3_INFO("Wrong hardware interface mode %s for component %s\n",
                hw_interface_mode_.c_str(), GetName().c_str());
        return false;
    }

    if (doc["ctrl_acceptable_mirror_error"]) {
        YAML::Node ctrl_err_node =
                doc["ctrl_acceptable_mirror_error"]["angular"];
        if (ctrl_err_node) {
            try {
                (ctrl_err_node)["position"] >> accept_ang_pos_;
            } catch (YAML::KeyNotFound& e) {
                accept_ang_pos_ = 0.017; //default value
            }

            try {
                (ctrl_err_node)["velocity"] >> accept_ang_vel_;
            } catch (YAML::KeyNotFound& e) {
                accept_ang_vel_ = 0.005; //default value
            }

            try {
                (ctrl_err_node)["effort"] >> accept_torque_;
            } catch (YAML::KeyNotFound& e) {
                accept_torque_ = 0.1; //default value
            }
        }

        ctrl_err_node = doc["ctrl_acceptable_mirror_error"]["linear"];
        if (ctrl_err_node) {
            try {
                (ctrl_err_node)["position"] >> accept_lin_pos_;
            } catch (YAML::KeyNotFound& e) {
                accept_lin_pos_ = 0.01; //default value
            }

            try {
                (ctrl_err_node)["velocity"] >> accept_lin_vel_;
            } catch (YAML::KeyNotFound& e) {
                accept_lin_vel_ = 0.02; //default value
            }

            try {
                (ctrl_err_node)["effort"] >> accept_force_;
            } catch (YAML::KeyNotFound& e) {
                accept_force_ = 1.0; //default value
            }
        }
    }

    if (doc["preload_controllers"]) {
        const YAML::Node& controllers = doc["preload_controllers"];
        controller_list_.resize(controllers.size());
        for (unsigned i = 0; i < controllers.size(); i++) {
            controllers[i] >> controller_list_[i];
        }
    }
    return true;
}

void RosControlComponent::StepStatus() {

    if (end_sds_) {
        sys_thread_end = 1;
    }

    if (!skip_loop_) {
        //SAVE_TIME(start_dt_status_);
        // read from hardware
        hw_ptr_->read();
        // handle e_stop
        if (!pwr_shr_ptr_->IsMotorPowerOn()) {
            rt_sem_wait(state_mutex_);
            hw_ptr_->changeStateAll(STATE_CMD_ESTOP);
            rt_sem_signal(state_mutex_);
            was_estop_ = true;
        } else {//automatic recover to stop after estop
            if (was_estop_) {
                was_estop_ = false;
                rt_sem_wait(state_mutex_);
                hw_ptr_->changeStateAll(STATE_CMD_STOP);
                rt_sem_signal(state_mutex_);
            }
        }
        // controller manager update
        // DO  NOT USE ros::Time::now();      
        const uint64_t one_E9 = 1000000000ULL;

        uint64_t nsec64 = rt_get_cpu_time_ns();

        uint32_t sec32_part = nsec64 / one_E9;
        uint32_t nsec32_part = nsec64 % one_E9;

        ros::Time now(sec32_part, nsec32_part);

        cm_ptr_->update(now, period_);

        //SAVE_TIME(end_dt_status_);
        //PRINT_TIME(start_dt_status_,end_dt_status_,tmp_dt_status_,"status");
    } else {
        if (loop_cnt_ % 1000 == 0) {

            M3_INFO(
                    "Component %s is not running, please check if roscore is started\n",
                    GetName().c_str());

        }
    }

}

void RosControlComponent::StepCommand() {

    if (!skip_loop_) {
        //SAVE_TIME(start_dt_cmd_);
        rt_sem_wait(state_mutex_);
        // write to hardware
        hw_ptr_->write();
        // publish the ctrl state once every 100 loops
        if (loop_cnt_ % 100 == 0) {
            if (realtime_pub_ptr_->trylock()) {
                hw_ptr_->getPublishableState(realtime_pub_ptr_->msg_);
                realtime_pub_ptr_->unlockAndPublish();
            }
        }
        rt_sem_signal(state_mutex_);

        //SAVE_TIME(end_dt_cmd_);
        //PRINT_TIME(start_dt_cmd_,end_dt_cmd_,tmp_dt_cmd_,"cmd");
    }

    loop_cnt_++;
}

M3BaseStatus* RosControlComponent::GetBaseStatus() {
    return status_.mutable_base();
} //NOTE make abstract M3Component happy

bool RosControlComponent::RosInit(m3::M3Humanoid* bot, m3::M3JointZLift* lift) {
    //std::string ros_node_name = GetName();
    std::string ros_node_name = "meka_roscontrol";
    int argc = 1;
    char* arg0 = strdup(ros_node_name.c_str());
    char* argv[] = { arg0, 0 };

    ros::init(argc, argv, ros_node_name, ros::init_options::NoSigintHandler);
    free(arg0);

    m3rt::M3_INFO("Checking for running roscore... %s\n", GetName().c_str());
    if (ros::master::check()) {
        // first node handler for the controller manager and controllers
        ros_nh_ptr_ = new ros::NodeHandle(ros_node_name);
        // second node handler for the state manager with a separate cb queue
        ros_nh_ptr2_ = new ros::NodeHandle(ros_node_name + "_state_manager");
        // third node handler for the base sds stuff
        ros_nh_ptr3_ = new ros::NodeHandle(ros_node_name + "_base_controller");

        cb_queue_ptr = new ros::CallbackQueue();
        ros_nh_ptr2_->setCallbackQueue(cb_queue_ptr);

        // Create a rt thread for state manager service handler
        spinner_running_ = true;
        rc = -1;
        rc = rt_thread_create((void*) ros_async_spinner, (void*) this, 1000000);
        //m3rt::M3_INFO("rc: %d\n",(int)rc);

        // Create a std async spinner for the controller mananager services and callbacks
        // Async spinner fail to cleanly stop with such errors
        // boost::recursive_mutex::~recursive_mutex(): Assertion `!pthread_mutex_destroy(&m)' failed
        // spinner_ptr_ = new ros::AsyncSpinner(1); // Use one thread for the external communications
        // spinner_ptr_->start();
        // THUS use our own main spinner.
        mrc = -1;
        mrc = rt_thread_create((void*) rosmain_async_spinner, (void*) this,
                1000000);

        odom_broadcaster_ptr.reset(new tf::TransformBroadcaster);
        cmd_sub_g = ros_nh_ptr3_->subscribe("omnibase_command", 1, stepSds);
        odom_publisher_g = ros_nh_ptr3_->advertise<nav_msgs::Odometry>(
                "omnibase_odom", 1, true);

        signal(SIGINT, end_sds_th);

        // Create a rt thread for ros omnibase control via sds
        if (sys = ((M3Sds*) rt_shm_alloc(nam2num(MEKA_ODOM_SHM), sizeof(M3Sds),
                USE_VMALLOC))) {
            m3rt::M3_INFO(
                    "Found shared memory starting shm_omnibase_controller.\n");
            hst = rt_thread_create((void*) rt_system_thread, sys, 10000);
        } else {
            printf("Rtai_malloc failure for %s\n", MEKA_ODOM_SHM);
        }

        usleep(100000); //Let start up
        if (!sys_thread_active) {
            //rt_task_delete(task);
            rt_shm_free(nam2num(MEKA_ODOM_SHM));
            printf("Startup of SDS thread failed!\n");
        }

        // Create the Meka Hardware interface
        hw_ptr_ = new MekaRobotHW(bot, lift, hw_interface_mode_);
        hw_ptr_->setCtrlAcceptableMirrorError(accept_ang_pos_, accept_ang_vel_,
                accept_torque_, accept_lin_pos_, accept_lin_vel_,
                accept_force_);

        // Create a realtime publisher for the state
        realtime_pub_ptr_ = new realtime_tools::RealtimePublisher<
                m3meka_msgs::M3ControlStates>(*ros_nh_ptr2_, "state", 4);
        realtime_pub_ptr_->msg_.group_name.resize(hw_ptr_->getNbGroup());
        realtime_pub_ptr_->msg_.state.resize(hw_ptr_->getNbGroup());

        // Create the controller manager
        cm_ptr_ = new controller_manager::ControllerManager(hw_ptr_,
                *ros_nh_ptr_);

        //// Initialize controllers that are already listed in the config file
        PreLoadControllers();

        //// Advertize the change state service in the dedicated nodehandler/spinner
        srv_ = ros_nh_ptr2_->advertiseService("change_state",
                &RosControlComponent::changeStateCallback, this);
    } else {
        //ros_nh_ptr_ = NULL;
        m3rt::M3_ERR(
                "Roscore is not running, can not initializate the controller_manager in component %s...\n",
                GetName().c_str());
        return false;
    }
    return true;
}

void RosControlComponent::RosShutdown() {
    m3rt::M3_INFO("Shutting down ros interface\n");
    //UnloadControllers() is blocked by no CM->update being called anylonger
    // to swap the double-buffered controller list

    //if (spinner_ptr_ != NULL)
    //    spinner_ptr_->stop();

    spinner_running_ = false;
    if (rc) {
        m3rt::M3_INFO("Waiting for RT spinner to stop...\n");
        rt_thread_join(rc);
        rc = -1;
    }

    if (mrc) {
        m3rt::M3_INFO("Waiting for RT main spinner to stop...\n");
        rt_thread_join(mrc);
        mrc = -1;
    }

    if (hst) {
        m3rt::M3_INFO("Removing RT thread...\n");
        sys_thread_end = 1;
        usleep(1250000);
        if (sys_thread_active)
            m3rt::M3_ERR("Real-time thread did not shutdown correctly or was never running...\n");
        //rt_task_delete(task);
        rt_shm_free(nam2num(MEKA_ODOM_SHM));
    }

    srv_.shutdown();
}

void RosControlComponent::PreLoadControllers() {
    for (unsigned i = 0; i < controller_list_.size(); i++) {
        if (cm_ptr_->loadController(controller_list_[i])) {
            M3_INFO("Controller %s pre-loaded\n", controller_list_[i].c_str());
        }
    }
}

void RosControlComponent::UnloadControllers() {
    for (unsigned i = 0; i < controller_list_.size(); i++) {
        if (cm_ptr_->unloadController(controller_list_[i])) {
            M3_INFO("Controller %s unloaded\n", controller_list_[i].c_str());
        }
    }
}

bool RosControlComponent::changeStateCallback(
        m3meka_msgs::M3ControlStateChange::Request &req,
        m3meka_msgs::M3ControlStateChange::Response &res) {
    int ret = 0;
    if (req.command.state.size() > 0
            && req.command.state.size() == req.command.group_name.size()) {
        // E-stop state can be modified only from internal commands
        if (!was_estop_) {
            for (size_t i = 0; i < req.command.group_name.size(); i++) {
                int ret_tmp = 0;
                //special handling of base for now...
                if(req.command.group_name[i] == "base") {
                    if (!sys_thread_active) { //base sds thread is not active anyways..
                        ret_tmp = -3;
                    } else {
                        int state_cmd = req.command.state[i];
                        ros_to_sds_mtx.lock();
                        switch (state_cmd) {
                        case STATE_CMD_ESTOP:
                            ros_to_sds_ = 0;
                            break;
                        case STATE_CMD_FREEZE:
                            ros_to_sds_ = 0;
                            break;
                        case STATE_CMD_STOP:
                            ros_to_sds_ = 0;
                            break;
                        case STATE_CMD_START:
                            ros_to_sds_ = 1;
                            break;
                        }
                        ros_to_sds_mtx.unlock();
                    }

                } else {
                    // during change, no other function must use the ctrl_state.
                    rt_sem_wait(this->state_mutex_);
                    ret_tmp = hw_ptr_->changeState(req.command.state[i],
                            req.command.group_name[i]);
                    rt_sem_signal(this->state_mutex_);


                    res.result.group_name.push_back(req.command.group_name[i]);
                    res.result.state.push_back(
                            hw_ptr_->getCtrlState(req.command.group_name[i]));
                }
                // only consider the worst error
                if (ret_tmp != 0 && ret_tmp < ret) {
                    ret = ret_tmp;
                }
            }
        } else {
            ret = -3;
        }

        if (ret < 0) {
            if (ret == -2)
                res.error_code.val =
                        m3meka_msgs::M3ControlStateErrorCodes::CONTROLLER_NOT_CONVERGED; //hw_ptr_->getCtrlState();
            else
                res.error_code.val =
                        m3meka_msgs::M3ControlStateErrorCodes::FAILURE;
        } else
            res.error_code.val = m3meka_msgs::M3ControlStateErrorCodes::SUCCESS;
    } else
        res.error_code.val = m3meka_msgs::M3ControlStateErrorCodes::FAILURE;
    return true;
}

}
