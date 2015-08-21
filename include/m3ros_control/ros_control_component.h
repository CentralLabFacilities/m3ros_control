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

    typedef std::map<std::string, std::pair<M3Chain, int> > map_t;
    typedef map_t::iterator map_it_t;

    MekaRobotHW(m3::M3Humanoid* bot_shr_ptr, m3::M3JointZLift* zlift_shr_ptr,
            std::string hw_interface_mode) :
            bot_shr_ptr_(NULL), zlift_shr_ptr_(NULL), ctrl_state_(STATE_ESTOP),
            frozen_(false), allow_running_(false)
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

        // Create a map with the ndofs
        //chains_map_["right_arm"] = std::make_pair(RIGHT_ARM,bot_shr_ptr->GetNdof(RIGHT_ARM));
        //chains_map_["left_arm"] = std::make_pair(LEFT_ARM,bot_shr_ptr->GetNdof(LEFT_ARM));

        // Set the number of dof
        ndof_right_arm_ = bot_shr_ptr_->GetNdof(RIGHT_ARM);
        ndof_left_arm_ = bot_shr_ptr_->GetNdof(LEFT_ARM);
        ndof_head_ = bot_shr_ptr_->GetNdof(HEAD);
        ndof_torso_ = bot_shr_ptr_->GetNdof(TORSO);
        ndof_right_hand_ = bot_shr_ptr_->GetNdof(RIGHT_HAND);
        ndof_left_hand_ = bot_shr_ptr_->GetNdof(LEFT_HAND);
        ndof_zlift_ = 1;

        ndof_ = ndof_right_arm_ + ndof_left_arm_ + ndof_head_ + ndof_right_hand_
                + ndof_left_hand_ + ndof_torso_ + ndof_zlift_;

        joint_name_.resize(ndof_);
        joint_pos_.resize(ndof_);
        joint_pos_command_.resize(ndof_);
        joint_vel_.resize(ndof_);
        joint_vel_command_.resize(ndof_);
        joint_effort_.resize(ndof_);
        joint_effort_command_.resize(ndof_);
        joint_err_.resize(ndof_);
        frozen_joint_command_.resize(ndof_);

        //joint_mode_ = new int[ndof_];

        /*joint_pos_.fill(0.0);
         joint_pos_command_.fill(0.0);
         joint_vel_.fill(0.0);
         joint_vel_command_.fill(0.0);
         joint_effort_.fill(0.0);
         joint_effort_command_.fill(0.0);*/

        // Populate hardware interfaces
        // RIGHT_ARM
        istart_ = 0;
        iend_ = ndof_right_arm_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_name_[i] = "right_arm_j" + std::to_string(i);
            registerHandles(joint_name_[i], &joint_pos_[i], &joint_vel_[i],
                    &joint_effort_[i], &joint_pos_command_[i],
                    &joint_effort_command_[i], &joint_vel_command_[i]);
            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        // LEFT_ARM
        istart_ += ndof_right_arm_;
        iend_ += ndof_left_arm_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_name_[i] = "left_arm_j" + std::to_string(i - istart_);
            registerHandles(joint_name_[i], &joint_pos_[i], &joint_vel_[i],
                    &joint_effort_[i], &joint_pos_command_[i],
                    &joint_effort_command_[i], &joint_vel_command_[i]);
            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        // TORSO
        istart_ += ndof_left_arm_;
        iend_ += ndof_torso_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_name_[i] = "torso_j" + std::to_string(i - istart_);
            registerHandles(joint_name_[i], &joint_pos_[i], &joint_vel_[i],
                    &joint_effort_[i], &joint_pos_command_[i],
                    &joint_effort_command_[i], &joint_vel_command_[i]);
            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }

        // HEAD
        istart_ += ndof_torso_;
        iend_ += ndof_head_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_name_[i] = "head_j" + std::to_string(i - istart_);
            registerHandles(joint_name_[i], &joint_pos_[i], &joint_vel_[i],
                    &joint_effort_[i], &joint_pos_command_[i],
                    &joint_effort_command_[i], &joint_vel_command_[i]);
            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        // RIGHT HAND
        istart_ += ndof_head_;
        iend_ += ndof_right_hand_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_name_[i] = "right_hand_j" + std::to_string(i - istart_);
            registerHandles(joint_name_[i], &joint_pos_[i], &joint_vel_[i],
                    &joint_effort_[i], &joint_pos_command_[i],
                    &joint_effort_command_[i], &joint_vel_command_[i]);
            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        // LEFT HAND
        istart_ += ndof_right_hand_;
        iend_ += ndof_left_hand_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_name_[i] = "left_hand_j" + std::to_string(i - istart_);
            registerHandles(joint_name_[i], &joint_pos_[i], &joint_vel_[i],
                    &joint_effort_[i], &joint_pos_command_[i],
                    &joint_effort_command_[i], &joint_vel_command_[i]);
            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        // ZLIFT
        istart_ += ndof_left_hand_;
        iend_ += ndof_zlift_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_name_[i] = "zlift_j" + std::to_string(i - istart_);
            // set position command to 300.0 //current position
            // GetPos is not yet updated here joint_pos_command_[i] = zlift_shr_ptr_->GetPos();
            joint_pos_command_[i] = 300.0;
            registerHandles(joint_name_[i], &joint_pos_[i], &joint_vel_[i],
                    &joint_effort_[i], &joint_pos_command_[i],
                    &joint_effort_command_[i], &joint_vel_command_[i]);
        }
        registerInterface(&js_interface_);
        registerInterface(&pj_interface_);
        registerInterface(&ej_interface_);
        registerInterface(&vj_interface_);

    }

    void read()
    {   // RIGHT_ARM
        istart_ = 0;
        iend_ = ndof_right_arm_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_pos_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDeg(RIGHT_ARM, i));
            joint_vel_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDotDeg(RIGHT_ARM, i));
            joint_effort_[i] = mm2m(bot_shr_ptr_->GetTorque_mNm(RIGHT_ARM, i)); // mNm -> Nm
        }
        // LEFT_ARM
        istart_ += ndof_right_arm_;
        iend_ += ndof_left_arm_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_pos_[i] = DEG2RAD(
                    bot_shr_ptr_->GetThetaDeg(LEFT_ARM, i - istart_));
            joint_vel_[i] = DEG2RAD(
                    bot_shr_ptr_->GetThetaDotDeg(LEFT_ARM, i - istart_));
            joint_effort_[i] = mm2m(
                    bot_shr_ptr_->GetTorque_mNm(LEFT_ARM, i - istart_)); // mNm -> Nm
        }
        // TORSO
        istart_ += ndof_left_arm_;
        iend_ += ndof_torso_;
        for (int i = istart_; i < iend_; i++)
        {
            //j1 slave is a copy of j1
            if (i - istart_ == 2)
            {
                joint_pos_[i] = DEG2RAD(
                        bot_shr_ptr_->GetThetaDeg(TORSO, i - istart_ - 1));
                joint_vel_[i] = DEG2RAD(
                        bot_shr_ptr_->GetThetaDotDeg(TORSO, i - istart_ - 1));
                joint_effort_[i] = mm2m(
                        bot_shr_ptr_->GetTorque_mNm(TORSO, i - istart_ - 1)); // mNm -> Nm
            }
            else
            {
                joint_pos_[i] = DEG2RAD(
                        bot_shr_ptr_->GetThetaDeg(TORSO, i - istart_));
                joint_vel_[i] = DEG2RAD(
                        bot_shr_ptr_->GetThetaDotDeg(TORSO, i - istart_));
                joint_effort_[i] = mm2m(
                        bot_shr_ptr_->GetTorque_mNm(TORSO, i - istart_)); // mNm -> Nm
            }
        }
        // HEAD
        istart_ += ndof_torso_;
        iend_ += ndof_head_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_pos_[i] = DEG2RAD(
                    bot_shr_ptr_->GetThetaDeg(HEAD, i - istart_));
            joint_vel_[i] = DEG2RAD(
                    bot_shr_ptr_->GetThetaDotDeg(HEAD, i - istart_));
            joint_effort_[i] = mm2m(
                    bot_shr_ptr_->GetTorque_mNm(HEAD, i - istart_)); // mNm -> Nm
        }
        // RIGHT HAND
        istart_ += ndof_head_;
        iend_ += ndof_right_hand_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_pos_[i] = DEG2RAD(
                    bot_shr_ptr_->GetThetaDeg(RIGHT_HAND, i - istart_));
            joint_vel_[i] = DEG2RAD(
                    bot_shr_ptr_->GetThetaDotDeg(RIGHT_HAND, i - istart_));
            joint_effort_[i] = mm2m(
                    bot_shr_ptr_->GetTorque_mNm(RIGHT_HAND, i - istart_)); // mNm -> Nm
        }
        // LEFT HAND
        istart_ += ndof_right_hand_;
        iend_ += ndof_left_hand_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_pos_[i] = DEG2RAD(
                    bot_shr_ptr_->GetThetaDeg(LEFT_HAND, i - istart_));
            joint_vel_[i] = DEG2RAD(
                    bot_shr_ptr_->GetThetaDotDeg(LEFT_HAND, i - istart_));
            joint_effort_[i] = mm2m(
                    bot_shr_ptr_->GetTorque_mNm(LEFT_HAND, i - istart_)); // mNm -> Nm
        }
        // ZLIFT
        istart_ += ndof_left_hand_;
        iend_ += ndof_zlift_;
        for (int i = istart_; i < iend_; i++)
        {
            joint_pos_[i] = zlift_shr_ptr_->GetPos();
            joint_vel_[i] = zlift_shr_ptr_->GetPosDot();
            joint_effort_[i] = mm2m(zlift_shr_ptr_->GetForce());    // mNm -> Nm
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
        if(ctrl_state_ == STATE_READY)
        {
            checkCtrlConvergence();
            freezeJoints();
        }

        // in ready state and above, allow writing
        if(ctrl_state_ >= STATE_READY)
        {
            // RIGHT_ARM
            istart_ = 0;
            iend_ = ndof_right_arm_;
            for (int i = istart_; i < iend_; i++)
            {
                bot_shr_ptr_->SetStiffness(RIGHT_ARM, i - istart_, 1.0);
                bot_shr_ptr_->SetSlewRateProportional(RIGHT_ARM, i - istart_, 1.0);
                switch (joint_mode_)
                {
                case VELOCITY:
                    bot_shr_ptr_->SetModeThetaDotGc(RIGHT_ARM, i - istart_);
                    bot_shr_ptr_->SetThetaDotDeg(RIGHT_ARM, i - istart_,
                            RAD2DEG(joint_vel_command_[i]));
                    break;
                case POSITION:
                    bot_shr_ptr_->SetModeThetaGc(RIGHT_ARM, i - istart_);
                    bot_shr_ptr_->SetThetaDeg(RIGHT_ARM, i - istart_,
                            RAD2DEG(joint_pos_command_[i]));
                    break;
                case EFFORT:
                    bot_shr_ptr_->SetModeTorqueGc(RIGHT_ARM, i - istart_);
                    bot_shr_ptr_->SetTorque_mNm(RIGHT_ARM, i - istart_,
                            m2mm(joint_effort_command_[i]));
                    break;
                default:
                    break;
                }
            }
            // LEFT_ARM
            istart_ += ndof_right_arm_;
            iend_ += ndof_left_arm_;
            for (int i = istart_; i < iend_; i++)
            {
                bot_shr_ptr_->SetStiffness(LEFT_ARM, i - istart_, 1.0);
                bot_shr_ptr_->SetSlewRateProportional(LEFT_ARM, i - istart_, 1.0);
                switch (joint_mode_)
                {
                case VELOCITY:
                    bot_shr_ptr_->SetModeThetaDotGc(LEFT_ARM, i - istart_);
                    bot_shr_ptr_->SetThetaDotDeg(LEFT_ARM, i - istart_,
                            RAD2DEG(joint_vel_command_[i]));
                    break;
                case POSITION:
                    bot_shr_ptr_->SetModeThetaGc(LEFT_ARM, i - istart_);
                    bot_shr_ptr_->SetThetaDeg(LEFT_ARM, i - istart_,
                            RAD2DEG(joint_pos_command_[i]));
                    break;
                case EFFORT:
                    bot_shr_ptr_->SetModeTorqueGc(LEFT_ARM, i - istart_);
                    bot_shr_ptr_->SetTorque_mNm(LEFT_ARM, i - istart_,
                            m2mm(joint_effort_command_[i]));
                    break;
                default:
                    break;
                }
            }
            // TORSO
            istart_ += ndof_left_arm_;
            iend_ += ndof_torso_;
            for (int i = istart_; i < iend_; i++)
            {
                //j1 slave is a copy of j1, do not write j1 slave value to the actuator
                if (i - istart_ < 2)
                {
                    bot_shr_ptr_->SetStiffness(TORSO, i - istart_, 1.0);
                    bot_shr_ptr_->SetSlewRateProportional(TORSO, i - istart_, 1.0);
                    switch (joint_mode_)
                    {
                    case POSITION:
                        bot_shr_ptr_->SetModeThetaGc(TORSO, i - istart_);
                        bot_shr_ptr_->SetThetaDeg(TORSO, i - istart_,
                                RAD2DEG(joint_pos_command_[i]));
                        break;
                    default:
                        break;
                    }
                }
            }
            // HEAD
            istart_ += ndof_torso_;
            iend_ += ndof_head_;
            for (int i = istart_; i < iend_; i++)
            {
                bot_shr_ptr_->SetStiffness(HEAD, i - istart_, 1.0);
                bot_shr_ptr_->SetSlewRateProportional(HEAD, i - istart_, 1.0);
                switch (joint_mode_)
                {
                case POSITION:
                    bot_shr_ptr_->SetModeTheta(HEAD, i - istart_);
                    bot_shr_ptr_->SetThetaDeg(HEAD, i - istart_,
                            RAD2DEG(joint_pos_command_[i]));
                    break;
                default:
                    break;
                }
            }
            // RIGHT_HAND
            istart_ += ndof_head_;
            iend_ += ndof_right_hand_;
            for (int i = istart_; i < iend_; i++)
            {
                bot_shr_ptr_->SetStiffness(RIGHT_HAND, i - istart_, 1.0);
                bot_shr_ptr_->SetSlewRateProportional(RIGHT_HAND, i - istart_, 1.0);
                switch (joint_mode_)
                {
                case VELOCITY:
                    bot_shr_ptr_->SetModeThetaDotGc(RIGHT_HAND, i - istart_);
                    bot_shr_ptr_->SetThetaDotDeg(RIGHT_HAND, i - istart_,
                            RAD2DEG(joint_vel_command_[i]));
                    break;
                case POSITION:
                    bot_shr_ptr_->SetModeThetaGc(RIGHT_HAND, i - istart_);
                    bot_shr_ptr_->SetThetaDeg(RIGHT_HAND, i - istart_,
                            RAD2DEG(joint_pos_command_[i]));
                    break;
                case EFFORT:
                    bot_shr_ptr_->SetModeTorqueGc(RIGHT_HAND, i - istart_);
                    bot_shr_ptr_->SetTorque_mNm(RIGHT_HAND, i - istart_,
                            m2mm(joint_effort_command_[i]));
                    break;
                default:
                    break;
                }
            }
            // LEFT_HAND
            istart_ += ndof_right_hand_;
            iend_ += ndof_left_hand_;
            for (int i = istart_; i < iend_; i++)
            {
                bot_shr_ptr_->SetStiffness(LEFT_HAND, i - istart_, 1.0);
                bot_shr_ptr_->SetSlewRateProportional(LEFT_HAND, i - istart_, 1.0);
                switch (joint_mode_)
                {
                case VELOCITY:
                    bot_shr_ptr_->SetModeThetaDotGc(LEFT_HAND, i - istart_);
                    bot_shr_ptr_->SetThetaDotDeg(LEFT_HAND, i - istart_,
                            RAD2DEG(joint_vel_command_[i]));
                    break;
                case POSITION:
                    bot_shr_ptr_->SetModeThetaGc(LEFT_HAND, i - istart_);
                    bot_shr_ptr_->SetThetaDeg(LEFT_HAND, i - istart_,
                            RAD2DEG(joint_pos_command_[i]));
                    break;
                case EFFORT:
                    bot_shr_ptr_->SetModeTorqueGc(LEFT_HAND, i - istart_);
                    bot_shr_ptr_->SetTorque_mNm(LEFT_HAND, i - istart_,
                            m2mm(joint_effort_command_[i]));
                    break;
                default:
                    break;
                }
            }
            // ZLIFT
            istart_ += ndof_left_hand_;
            iend_ += ndof_zlift_;
            for (int i = istart_; i < iend_; i++)
            {
                zlift_shr_ptr_->SetDesiredStiffness(1.0);
                zlift_shr_ptr_->SetSlewRate(
                        1.0
                                * ((M3JointParam*) zlift_shr_ptr_->GetParam())->max_q_slew_rate());
                switch (joint_mode_)
                {
                case VELOCITY:
                    zlift_shr_ptr_->SetDesiredControlMode(JOINT_MODE_THETADOT_GC);
                    zlift_shr_ptr_->SetDesiredPosDot(joint_vel_command_[i]);
                    break;
                case POSITION:
                    zlift_shr_ptr_->SetDesiredControlMode(JOINT_MODE_THETA_GC);

                    zlift_shr_ptr_->SetDesiredPos(joint_pos_command_[i]);
                    break;
                case EFFORT:
                    // P.L. do nothin
                    break;
                default:
                    break;
                }
            }
        }
    }

    

private:

    int istart_, iend_, ndof_right_arm_, ndof_left_arm_, ndof_head_,
            ndof_right_hand_, ndof_left_hand_, ndof_torso_, ndof_zlift_, ndof_;

    m3::M3Humanoid* bot_shr_ptr_;
    m3::M3JointZLift* zlift_shr_ptr_;

    hardware_interface::JointStateInterface js_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::EffortJointInterface ej_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;
    
    enum joint_mode_t
    {
        POSITION, EFFORT, VELOCITY
    };
    
    int ctrl_state_;
    bool frozen_;
    bool allow_running_;
    joint_mode_t joint_mode_;

    std::vector<double> joint_effort_command_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_pos_;
    std::vector<double> joint_pos_command_;
    std::vector<double> joint_vel_;
    std::vector<double> joint_vel_command_;
    std::vector<double> joint_err_;
    std::vector<double> frozen_joint_command_;
    std::vector<std::string> joint_name_;
    
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
    
    // store a freeze state (cur pos/ zero vel/ cur trq , depends on mode) 
    // and override the current commanded pos/vel/trq with the stored one
    void freezeJoints()
    {
        switch (joint_mode_)
        {
        case VELOCITY:
            // freeze means zero velocity
            for (int i = 0; i < ndof_; i++)
            {
                // save current command
                if(!frozen_)
                    frozen_joint_command_[i] = 0.0;
                // enforce to frozen position
                joint_vel_command_[i] = frozen_joint_command_[i];
            }
            frozen_=true;
            break;
        case POSITION:
            // freeze means maintain current position
            for (int i = 0; i < ndof_; i++)
            {
                // save current command
                if(!frozen_)
                    frozen_joint_command_[i] = joint_pos_[i];
                // enforce to frozen position
                joint_pos_command_[i] = frozen_joint_command_[i];
            }
            if(!frozen_)
                m3rt::M3_INFO("Frozen to current position\n");
            frozen_=true;
            break;
        case EFFORT:
            // freeze means maintain current effort ?
            for (int i = 0; i < ndof_; i++)
            {
                // save current command
                if(!frozen_)
                    frozen_joint_command_[i] = joint_effort_[i];
                // enforce to frozen position
                joint_effort_command_[i] = frozen_joint_command_[i];
            }
            frozen_=true;
            break;
        default:
            break;
        }
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
        int ret=0;
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
            frozen_= false;
            allow_running_= false;
            break;
            
        case STATE_CMD_STOP:
            if (ctrl_state_ == STATE_RUNNING)
                allow_running_ = false;
                //switch controller off
            ctrl_state_ = STATE_STANDBY;
            frozen_ = false;
            break;

        case STATE_CMD_FREEZE:
            if (ctrl_state_ == STATE_RUNNING)
                allow_running_ = false;

            ctrl_state_ = STATE_READY;
            break;

        case STATE_CMD_START:
            if (ctrl_state_ != STATE_RUNNING)
            {
                if (allow_running_)
                {
                    ctrl_state_ = STATE_RUNNING;
                    frozen_ = false;
                }
                else
                {
                    m3rt::M3_ERR(
                        "Controller did not converge to freeze position, \
                         cannot switch to running...\n");
                    ret = -2;
                }
            }

            break;
        default:
            m3rt::M3_ERR(
                    "Unknown state command %d...\n",
                    state_cmd);
            ret = -1;
            break;
        }
        return ret;
    }

    /* measure difference between command and current pos
     to tell if controllers have converged */
    bool checkCtrlConvergence()
    {
        bool converged = true;
        switch (joint_mode_)
        {
        case VELOCITY:
            // freeze means zero velocity
            for (int i = 0; i < ndof_; i++)
            {
                // measure error between controller command and freeze command
                joint_err_[i] = std::fabs(joint_vel_command_[i]);
                if (joint_err_[i] > ACCEPTABLE_VEL_MIRROR)
                {
                    converged = false;
                    break;
                }
            }
            break;
        case POSITION:
            // freeze means maintain current position
            for (int i = 0; i < ndof_; i++)
            {
                // measure error between controller command and freeze command
                joint_err_[i] = std::fabs(joint_pos_command_[i]-joint_pos_[i]);
                if (joint_err_[i] > ACCEPTABLE_POS_MIRROR)
                {
                    converged = false;
                    if(allow_running_)
                        m3rt::M3_DEBUG("joint %d, shows a position difference %f > %f \n", i, joint_err_[i], ACCEPTABLE_POS_MIRROR);
                    break;
                }
            }
            break;
        case EFFORT:
            // freeze means maintain current effort ?
            for (int i = 0; i < ndof_; i++)
            {
                // measure error between controller command and freeze command
                joint_err_[i] = std::fabs(joint_effort_command_[i]-joint_effort_[i]);
                if (joint_err_[i] > ACCEPTABLE_EFFORT_MIRROR)
                {
                    converged = false;
                    break;
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
            rc= -1;
        }
        //if (srv_ptr_ != NULL)
        //    srv_ptr_->shutdown();
        if (cb_queue_ptr !=NULL)
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
    bool changeStateCallback(m3meka_msgs::M3ControlStateChange::Request  &req,
                     m3meka_msgs::M3ControlStateChange::Response &res)
    {
        // during change, no other function must use the ctrl_state.
        if (req.command.state.size()>0)
        {
            rt_sem_wait(this->state_mutex_);
            int ret = hw_ptr_->changeState(req.command.state[0]);
            rt_sem_signal(this->state_mutex_);
            if (ret < 0)
            {
                if (ret == -2)
                    res.error_code.val = m3meka_msgs::M3ControlStateErrorCodes::CONTROLLER_NOT_CONVERGED; //hw_ptr_->getCtrlState();
                else
                    res.error_code.val = m3meka_msgs::M3ControlStateErrorCodes::FAILURE; 
            }
            else
                res.error_code.val = m3meka_msgs::M3ControlStateErrorCodes::SUCCESS; 
        }
        else
            res.error_code.val = m3meka_msgs::M3ControlStateErrorCodes::FAILURE; 
        return true;
    }
};



}

#endif

