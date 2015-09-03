#ifndef ROS_CONTROL_COMPONENT_H
#define ROS_CONTROL_COMPONENT_H

extern "C"
{
#include <rtai_sched.h>
#include <stdio.h>
#include <signal.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
}

////////// M3
#include <m3/chains/arm.h>
#include <m3/robots/humanoid.h>
#include <m3/hardware/joint_zlift.h>

////////// M3RT
#include <m3rt/base/component.h>
#include <m3rt/base/component_shm.h>
#include <m3rt/base/m3rt_def.h>
#include <m3rt/base/component_factory.h>

////////// Google protobuff
#include <google/protobuf/message.h>
#include "m3ros_control/ros_control_component.pb.h"

////////// ROS/ROS_CONTROL
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <m3meka_msgs/M3ControlStateChange.h>
#include <m3meka_msgs/M3ControlState.h>
#include <m3meka_msgs/M3ControlStateErrorCodes.h>

//#include <hardware_interface/joint_mode_interface.h>

////////// Some defs
#define mm2m(a) (mReal((a))/1000) //millimeters to meters
#define m2mm(a) (mReal((a))*1000) //meters to millimeters

// master states
#define STATE_ESTOP     0
#define STATE_UNKNOWN   0
#define STATE_STANDBY   1
#define STATE_READY     2
#define STATE_RUNNING   3

// state transitions command
#define STATE_CMD_ESTOP     0
#define STATE_CMD_STOP      1
#define STATE_CMD_FREEZE    2
#define STATE_CMD_START     3

// acceptable errors between controller output and current state
// to verify controllers were reset to current state
#define ACCEPTABLE_VEL_MIRROR       0.05
#define ACCEPTABLE_POS_MIRROR       0.17
#define ACCEPTABLE_EFFORT_MIRROR    1.0

////////// Activate some timing infos
//#define TIMING
#define NANO2SEC(a) a/1e9
#define SEC2NANO(a) a*1e9

static int tmp_dt_status_;
static int tmp_dt_cmd_;

static long long start_dt_status_, end_dt_status_, elapsed_dt_status_;
static long long start_dt_cmd_, end_dt_cmd_, elapsed_dt_cmd_;

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

class MekaRobotHW: public hardware_interface::RobotHW
{
public:

    MekaRobotHW(m3::M3Humanoid* bot_shr_ptr, m3::M3JointZLift* zlift_shr_ptr,
            std::string hw_interface_mode) :
            bot_shr_ptr_(NULL), zlift_shr_ptr_(NULL), ctrl_state_(STATE_ESTOP), frozen_(
                    false), allow_running_(false)
    {
        using namespace hardware_interface;

        assert(bot_shr_ptr != NULL);
        assert(zlift_shr_ptr != NULL);
        bot_shr_ptr_ = bot_shr_ptr;
        zlift_shr_ptr_ = zlift_shr_ptr;

        if (hw_interface_mode == "position")
            joint_mode_ = POSITION;
        else if (hw_interface_mode == "effort")
            joint_mode_ = EFFORT;
        else if (hw_interface_mode == "velocity")
            joint_mode_ = VELOCITY;
        else
            joint_mode_ = POSITION;

        chain_map_["right_arm"] = Chain_(RIGHT_ARM,
                bot_shr_ptr->GetNdof(RIGHT_ARM));
        chain_map_["left_arm"] = Chain_(LEFT_ARM,
                bot_shr_ptr->GetNdof(RIGHT_ARM));
        chain_map_["head"] = Chain_(HEAD, bot_shr_ptr->GetNdof(RIGHT_ARM));
        chain_map_["torso"] = Chain_(TORSO, bot_shr_ptr->GetNdof(RIGHT_ARM));
        chain_map_["right_hand"] = Chain_(RIGHT_HAND,
                bot_shr_ptr->GetNdof(RIGHT_ARM));
        chain_map_["left_hand"] = Chain_(LEFT_HAND,
                bot_shr_ptr->GetNdof(RIGHT_ARM));
        chain_map_["zlift"] = Chain_("zlift", 1);
        
        for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++)
        {
            std::vector<joint_value_> *vals = &it->second.values;
            for (size_t i = 0; i < vals->size(); i++) 
            {
                registerHandles(vals->at(i).name,
                                &vals->at(i).position, &vals->at(i).velocity,
                                &vals->at(i).effort, &vals->at(i).pos_cmd, 
                                &vals->at(i).vel_cmd, &vals->at(i).eff_cmd);
    
            }
        }


        registerInterface(&js_interface_);
        registerInterface(&pj_interface_);
        registerInterface(&ej_interface_);
        registerInterface(&vj_interface_);

    }

    void read()
    {

        for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++)
        {
            std::vector<joint_value_> *vals = &it->second.values;
            if (it->first == "zlift")
            {
                (*vals)[0].position = zlift_shr_ptr_->GetPos();
                (*vals)[0].velocity = zlift_shr_ptr_->GetPosDot();
                (*vals)[0].effort = mm2m(zlift_shr_ptr_->GetForce()); // mNm -> Nm
                continue;
            }
            bool t_ = false;
            if (it->first == "torso")
                t_ = true;
            for (size_t i = 0; i < vals->size(); i++)
            {
                if (t_ && i == 2)
                {
                    vals->at(i).position = DEG2RAD(
                            bot_shr_ptr_->GetThetaDeg(it->second.chain_ref,
                                    i - 1));
                    vals->at(i).velocity = DEG2RAD(
                            bot_shr_ptr_->GetThetaDotDeg(it->second.chain_ref,
                                    i - 1));
                    vals->at(i).effort = mm2m(
                            bot_shr_ptr_->GetTorque_mNm(it->second.chain_ref,
                                    i - 1)); // mNm -> Nm
                }
                else
                {
                    vals->at(i).position = DEG2RAD(
                            bot_shr_ptr_->GetThetaDeg(it->second.chain_ref, i));
                    vals->at(i).velocity = DEG2RAD(
                            bot_shr_ptr_->GetThetaDotDeg(it->second.chain_ref,
                                    i));
                    vals->at(i).effort = mm2m(
                            bot_shr_ptr_->GetTorque_mNm(it->second.chain_ref,
                                    i)); // mNm -> Nm
                }
            }
        }
    }

    void write()
    {
        //if (safety_check())
        // if motor not powered on, isPowerOn cannot see the status of the E-Stop
        bot_shr_ptr_->SetMotorPowerOn();
        // in standby state and above, allow motor power
        /*if(ctrl_state >= STATE_STANDBY)
         {

         }
         else
         {
         bot_shr_ptr_->SetMotorPowerOff();
         }*/

        // in ready state, override joint_commands to freeze movements
        if (ctrl_state_ == STATE_READY)
        {
            checkCtrlConvergence();
            freezeJoints();
        }

        double s_ = 1.0;
        if (ctrl_state_ < STATE_READY)
        {
            s_ = 0.0;
            joint_mode_ = NOT_READY;
        }

        for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++)
        {

            if (!it->second.active)
                continue;

            std::vector<joint_value_> *vals = &it->second.values;

            if (it->first == "zlift")
            {
                zlift_shr_ptr_->SetDesiredStiffness(s_);
                zlift_shr_ptr_->SetSlewRate(
                        s_
                                * ((M3JointParam*) zlift_shr_ptr_->GetParam())->max_q_slew_rate());
                switch (joint_mode_)
                {
                case VELOCITY:
                    zlift_shr_ptr_->SetDesiredControlMode(
                            JOINT_MODE_THETADOT_GC);
                    zlift_shr_ptr_->SetDesiredPosDot((*vals)[0].vel_cmd);
                    break;
                case POSITION:
                    zlift_shr_ptr_->SetDesiredControlMode(JOINT_MODE_THETA_GC);
                    zlift_shr_ptr_->SetDesiredPos((*vals)[0].pos_cmd);
                    break;
                case EFFORT: //not yet implemented
                    break;
                default:
                    break;
                }
                continue;
            }

            //this is necessary because these groups need special treatment right now
            bool t_ = false;
            bool h_ = false;
            bool ha_ = false;
            if (it->first == "torso")
                t_ = true;
            if (it->first == "head")
                h_ = true;
            if (it->first == "right_hand" || it->first == "left_hand")
                ha_ = true;

            for (size_t i = 0; i < vals->size(); i++)
            {
                if (t_ || h_)
                {
                    if (i == 2 && t_) //j1 slave is a copy of j1, do not write j1 slave value to the actuator
                        break;
                    bot_shr_ptr_->SetStiffness(it->second.chain_ref, i, s_);
                    bot_shr_ptr_->SetSlewRateProportional(it->second.chain_ref,
                            i, s_);
                    switch (joint_mode_)
                    {
                    case VELOCITY: //not yet implemented
                        break;
                    case POSITION:
                        bot_shr_ptr_->SetModeThetaGc(it->second.chain_ref, i);
                        bot_shr_ptr_->SetThetaDeg(it->second.chain_ref, i,
                                RAD2DEG(vals->at(i).pos_cmd));
                        break;
                    case EFFORT: //not yet implemented
                        break;
                    default:
                        break;
                    }
                    continue;
                }
                bot_shr_ptr_->SetStiffness(it->second.chain_ref, i, s_);
                bot_shr_ptr_->SetSlewRateProportional(it->second.chain_ref, i,
                        s_);
                if (ha_ && ctrl_state_ < STATE_READY)
                    bot_shr_ptr_->SetModeOff(it->second.chain_ref, i);
                switch (joint_mode_)
                {
                case VELOCITY:
                    bot_shr_ptr_->SetModeThetaDotGc(it->second.chain_ref, i);
                    bot_shr_ptr_->SetThetaDotDeg(it->second.chain_ref, i,
                            RAD2DEG(vals->at(i).vel_cmd));
                    break;
                case POSITION:
                    bot_shr_ptr_->SetModeThetaGc(it->second.chain_ref, i);
                    bot_shr_ptr_->SetThetaDeg(it->second.chain_ref, i,
                            RAD2DEG(vals->at(i).pos_cmd));
                    break;
                case EFFORT:
                    bot_shr_ptr_->SetModeTorqueGc(it->second.chain_ref, i);
                    bot_shr_ptr_->SetTorque_mNm(it->second.chain_ref, i,
                            m2mm(vals->at(i).eff_cmd));
                    break;
                default:
                    break;
                }
            }
        }
    }

private:

    m3::M3Humanoid* bot_shr_ptr_;
    m3::M3JointZLift* zlift_shr_ptr_;

    hardware_interface::JointStateInterface js_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::EffortJointInterface ej_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;

    enum joint_mode_t
    {
        POSITION, EFFORT, VELOCITY, NOT_READY
    };

    int ctrl_state_;
    bool frozen_;
    bool allow_running_;
    joint_mode_t joint_mode_;

    void registerHandles(std::string name, double* pos, double* vel,
            double* eff, double* poscmd, double* effcmd, double* velcmd)
    {
        js_interface_.registerHandle(
                hardware_interface::JointStateHandle(name, pos, vel, eff));
        pj_interface_.registerHandle(
                hardware_interface::JointHandle(js_interface_.getHandle(name),
                        poscmd));
        ej_interface_.registerHandle(
                hardware_interface::JointHandle(js_interface_.getHandle(name),
                        effcmd));
        vj_interface_.registerHandle(
                hardware_interface::JointHandle(js_interface_.getHandle(name),
                        velcmd));

    }

    struct joint_value_
    {
        std::string name;
        double position;
        double velocity;
        double effort;
        double pos_cmd;
        double eff_cmd;
        double vel_cmd;
        double frz_cmd;
        double err;
    };

    struct Chain_
    {
        Chain_();
        
        Chain_(std::string name_, int ndof_, bool active_ = true) :
                name(name_), ndof(ndof_), active(active_)
        {
            values.resize(ndof);
            if (name == "zlift")
                values[0].position = 300.0;
            for (int i = 0; i < ndof; i++)
            {
                values[i].name = name + "_j" + std::to_string(i);
        

            }
        }
        Chain_(M3Chain chain_ref_, int ndof_, bool active_ = true) :
                chain_ref(chain_ref_)
        {
            Chain_(getStringFromEnum(chain_ref), ndof, active_);
        }
        
        std::string getStringFromEnum(M3Chain e)
            {
            switch(e)
            {
                case RIGHT_ARM: return "right_arm";
                case LEFT_ARM: return "left_arm";
                case TORSO: return "torso";
                case HEAD: return "head";
                case RIGHT_HAND: return "right_hand";
                case LEFT_HAND: return "left_hand";
                default: throw "Bad M3Chain Enum!";
            }
        }

        M3Chain chain_ref;
        std::string name;
        int ndof;
        bool active;
        std::vector<joint_value_> values;
    };

    typedef std::map<std::string, Chain_> map_t;
    typedef map_t::iterator map_it_t;

    map_t chain_map_;

    // store a freeze state (cur pos/ zero vel/ cur trq , depends on mode)
    // and override the current commanded pos/vel/trq with the stored one
    void freezeJoints()
    {
        switch (joint_mode_)
        {
        case VELOCITY:
            // freeze means zero velocity
            for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++)
            {
                std::vector<joint_value_> *vals = &it->second.values;
                for (size_t i = 0; i < vals->size(); i++)
                {
                    if (!frozen_)
                        vals->at(i).frz_cmd = 0.0;
                    vals->at(i).vel_cmd = vals->at(i).frz_cmd;
                }
            }
            frozen_ = true;
            break;
        case POSITION:
            // freeze means maintain current position
            for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++)
            {
                std::vector<joint_value_> *vals = &it->second.values;
                for (size_t i = 0; i < vals->size(); i++)
                {
                    if (!frozen_)
                        vals->at(i).frz_cmd = vals->at(i).position;
                    vals->at(i).pos_cmd = vals->at(i).frz_cmd;
                }
            }
            if (!frozen_)
                m3rt::M3_INFO("Frozen to current position\n");
            frozen_ = true;
            break;
        case EFFORT:
            // freeze means maintain current effort ?
            for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++)
            {
                std::vector<joint_value_> *vals = &it->second.values;
                for (size_t i = 0; i < vals->size(); i++)
                {
                    if (!frozen_)
                        vals->at(i).frz_cmd = vals->at(i).effort;
                    vals->at(i).eff_cmd = vals->at(i).frz_cmd;
                }
            }
            frozen_ = true;
            break;
        default:
            break;
        }
    }

    /* measure difference between command and current pos
     to tell if controllers have converged */
    bool checkCtrlConvergence()
    {
        bool converged = true;
        switch (joint_mode_)
        {
        case VELOCITY:
            for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++)
            {
                std::vector<joint_value_> *vals = &it->second.values;
                for (size_t i = 0; i < vals->size(); i++)
                {
                    vals->at(i).err = std::fabs(vals->at(i).vel_cmd);
                    if (vals->at(i).err > ACCEPTABLE_VEL_MIRROR)
                    {
                        converged = false;
                        break;
                    }
                }
            }
            break;
        case POSITION:
            for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++)
            {
                std::vector<joint_value_> *vals = &it->second.values;
                for (size_t i = 0; i < vals->size(); i++)
                {
                    vals->at(i).err = std::fabs(vals->at(i).pos_cmd - vals->at(i).position);
                    if (vals->at(i).err > ACCEPTABLE_POS_MIRROR)
                    {
                        converged = false;
                        if (allow_running_)
                            m3rt::M3_DEBUG(
                                    "joint %d, shows a position difference %f > %f \n",
                                    i, vals->at(i).err, ACCEPTABLE_POS_MIRROR);
                        break;
                    }
                }
            }
            break;
        case EFFORT:
            for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++)
            {
                std::vector<joint_value_> *vals = &it->second.values;
                for (size_t i = 0; i < vals->size(); i++)
                {
                    vals->at(i).err = std::fabs(vals->at(i).eff_cmd - vals->at(i).effort);
                    if (vals->at(i).err > ACCEPTABLE_EFFORT_MIRROR)
                    {
                        converged = false;
                        break;
                    }
                }
            }
            break;
        default:
            converged = false;
            break;
        }
        allow_running_ = converged;
        return converged;
    }

    public:
    // access the current control state
    int getCtrlState()
    {
        return ctrl_state_;
    }

    // try change the state to requested state_cmd
    int changeState(const int state_cmd)
    {
        int ret = 0;
        switch (state_cmd)
        {
        case STATE_CMD_ESTOP:

            if (ctrl_state_ == STATE_RUNNING)
            {
                //switch controller off
                m3rt::M3_INFO("You should switch controllers off !\n");
            }
            if (ctrl_state_ != STATE_ESTOP)
            {
                m3rt::M3_INFO("ESTOP detected\n");
            }

            ctrl_state_ = STATE_ESTOP;
            frozen_ = false;
            allow_running_ = false;
            break;

        case STATE_CMD_STOP:
            if (ctrl_state_ == STATE_RUNNING)
                allow_running_ = false;
            ctrl_state_ = STATE_STANDBY;
            frozen_ = false;
            break;

        case STATE_CMD_FREEZE:
            if (ctrl_state_ == STATE_RUNNING)
                allow_running_ = false;
            // cannot go to freeze if previously in e-stop
            // go to standby first
            if (ctrl_state_ == STATE_ESTOP)
                ret = -3;
            else
                ctrl_state_ = STATE_READY;
            break;

        case STATE_CMD_START:
            if (ctrl_state_ != STATE_RUNNING)
            {
                // cannot go to run if previously in e-stop
                // go to standby first
                if (ctrl_state_ == STATE_ESTOP)
                {
                    ret = -3;
                    break;
                }

                if (allow_running_)
                {
                    ctrl_state_ = STATE_RUNNING;
                    frozen_ = false;
                }
                else
                {
                    // set to ready to allow for convergence
                    // to be checked and freeze to be triggered
                    ctrl_state_ = STATE_READY;
                    m3rt::M3_ERR(
                            "Controller did not converge to freeze position, \
                             cannot switch to running...\n");
                    ret = -2;
                }
            }

            break;
        default:
            m3rt::M3_ERR("Unknown state command %d...\n", state_cmd);
            ret = -1;
            break;
        }
        return ret;
    }

};

void *ros_async_spinner(void * arg);

class RosControlComponent: public m3rt::M3Component
{
public:
    RosControlComponent() :
            m3rt::M3Component(MAX_PRIORITY), bot_shr_ptr_(NULL), zlift_shr_ptr_(
                    NULL), ros_nh_ptr_(NULL), spinner_ptr_(NULL), hw_ptr_(NULL), cm_ptr_(
                    NULL), skip_loop_(false)
    {
        RegisterVersion("default", DEFAULT);
    }
    ~RosControlComponent()
    {
        if (spinner_ptr_ != NULL)
            delete spinner_ptr_;
        if (hw_ptr_ != NULL)
            delete hw_ptr_;
        if (ros_nh_ptr_ != NULL)
            delete ros_nh_ptr_;
        if (cm_ptr_ != NULL)
            delete cm_ptr_;
    }

    google::protobuf::Message* GetCommand()
    {
        return &status_;
    } //NOTE make abstract M3Component happy
    google::protobuf::Message* GetStatus()
    {
        return &cmd_;
    }
    google::protobuf::Message* GetParam()
    {
        return &param_;
    }

    SEM *state_mutex_;
    bool spinner_running_;
    ros::CallbackQueue* cb_queue_ptr; // Used to separate this node queue from the global one

protected:
    bool ReadConfig(const char* filename);
    void Startup();
    void Shutdown();
    void StepStatus();
    void StepCommand();
    bool LinkDependentComponents();

    RosControlComponentStatus status_;
    RosControlComponentCommand cmd_;
    RosControlComponentParam param_;

    M3BaseStatus* GetBaseStatus()
    {
        return status_.mutable_base();
    } //NOTE make abstract M3Component happy

    bool RosInit(m3::M3Humanoid* bot, m3::M3JointZLift* lift);

    void RosShutdown()
    {
        if (spinner_ptr_ != NULL)
            spinner_ptr_->stop();

        spinner_running_ = false;
        if (rc)
        {
            rt_thread_join(rc);
            rc = -1;
        }
        //if (srv_ptr_ != NULL)
        //    srv_ptr_->shutdown();
        if (cb_queue_ptr != NULL)
            delete cb_queue_ptr;
        if (ros_nh_ptr_ != NULL)
            ros_nh_ptr_->shutdown();
    }

private:
    std::string bot_name_, zlift_name_, pwr_name_, hw_interface_mode_;
    m3::M3Humanoid* bot_shr_ptr_;
    m3::M3JointZLift* zlift_shr_ptr_;
    m3::M3Pwr* pwr_shr_ptr_;

    long rc;
    ros::Duration period_;
    ros::NodeHandle* ros_nh_ptr_, *ros_nh_ptr2_;
    ros::ServiceServer srv_;
    ros::AsyncSpinner* spinner_ptr_; // Used to keep alive the ros services in the controller manager
    realtime_tools::RealtimePublisher<m3meka_msgs::M3ControlStates> *realtime_pub_ptr_;

    MekaRobotHW* hw_ptr_;
    controller_manager::ControllerManager* cm_ptr_;
    enum
    {
        DEFAULT
    };
    bool skip_loop_;
    long long loop_cnt_;

    // calback function for the service
    bool changeStateCallback(m3meka_msgs::M3ControlStateChange::Request &req,
            m3meka_msgs::M3ControlStateChange::Response &res)
    {
        // during change, no other function must use the ctrl_state.
        if (req.command.state.size() > 0)
        {
            rt_sem_wait(this->state_mutex_);
            int ret = hw_ptr_->changeState(req.command.state[0]);
            rt_sem_signal(this->state_mutex_);
            if (ret < 0)
            {
                if (ret == -2)
                    res.error_code.val =
                            m3meka_msgs::M3ControlStateErrorCodes::CONTROLLER_NOT_CONVERGED; //hw_ptr_->getCtrlState();
                else
                    res.error_code.val =
                            m3meka_msgs::M3ControlStateErrorCodes::FAILURE;
            }
            else
                res.error_code.val =
                        m3meka_msgs::M3ControlStateErrorCodes::SUCCESS;
        }
        else
            res.error_code.val = m3meka_msgs::M3ControlStateErrorCodes::FAILURE;
        return true;
    }
};

}

#endif

