#ifndef ROS_CONTROL_COMPONENT_H
#define ROS_CONTROL_COMPONENT_H


extern "C"{
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
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/hardware_interface.h>
//#include <hardware_interface/joint_mode_interface.h>

////////// Some defs
#define mm2m(a)	(mReal((a))/1000) //millimeters to meters
#define m2mm(a)	(mReal((a))*1000) //meters to millimeters

////////// Activate some timing infos
//#define TIMING
#define NANO2SEC(a)	a/1e9
#define SEC2NANO(a)	a*1e9

static int tmp_dt_status_;
static int tmp_dt_cmd_;

static long long start_dt_status_,  end_dt_status_, elapsed_dt_status_;
static long long start_dt_cmd_,     end_dt_cmd_,    elapsed_dt_cmd_;

#ifndef NDEBUG
#define TIME_ACTIVE 1
#else
#define TIME_ACTIVE 0
#endif

#define INIT_CNT(cnt) do { if (TIME_ACTIVE) (cnt) = 0; } while (0) 
#define SAVE_TIME(out) do { if (TIME_ACTIVE) getCpuCount((out)); } while (0)
#define PRINT_TIME(T_start,T_end,cnt,string) do { if (TIME_ACTIVE) if ((cnt)%100==0) ROS_INFO("%s: %fs",string,count2Sec(((T_end) - (T_start)))); cnt = cnt++ & INT_MAX;} while (0)

inline void getCpuCount(long long& out){
    out = nano2count(rt_get_cpu_time_ns());
}

inline double count2Sec(const long long in){
    return (NANO2SEC((double)count2nano(in)));
}

namespace ros_control_component
{

using namespace controller_manager;

class MekaRobotHW : public hardware_interface::RobotHW
{
public:

    typedef std::map<std::string,std::pair<M3Chain,int> > map_t;
    typedef map_t::iterator map_it_t;

    MekaRobotHW(m3::M3Humanoid* bot_shr_ptr, std::string hw_interface_mode):bot_shr_ptr_(NULL)
    {
        using namespace hardware_interface;

        assert(bot_shr_ptr != NULL);
        bot_shr_ptr_ = bot_shr_ptr;

        if(hw_interface_mode == "position")
            joint_mode_ = POSITION;
        else if(hw_interface_mode == "effort")
            joint_mode_ = EFFORT;
        else if(hw_interface_mode == "velocity")
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
	std::cout << "#####################################" << std::endl;
	std::cout<<"ndof ra "<<ndof_right_arm_<<" la "<<ndof_left_arm_<<" lh "
        <<ndof_left_hand_<<" rh "<<ndof_right_hand_<<" h "<<ndof_head_<<" torso "<<ndof_torso_<<std::endl;

        ndof_ = ndof_right_arm_ + ndof_left_arm_ + ndof_head_ + ndof_right_hand_ + ndof_left_hand_ + ndof_torso_;

        joint_name_.resize(ndof_);
        joint_position_.resize(ndof_);
        joint_position_command_.resize(ndof_);
        joint_velocity_.resize(ndof_);
        joint_velocity_command_.resize(ndof_);
        joint_effort_.resize(ndof_);
        joint_effort_command_.resize(ndof_);


        //joint_mode_ = new int[ndof_];

        /*joint_position_.fill(0.0);
                joint_position_command_.fill(0.0);
                joint_velocity_.fill(0.0);
                joint_velocity_command_.fill(0.0);
                joint_effort_.fill(0.0);
                joint_effort_command_.fill(0.0);*/

        // Populate hardware interfaces
        // RIGHT_ARM
        istart_ = 0;
        iend_ = ndof_right_arm_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_name_[i] = "right_arm_j"+std::to_string(i);
            js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
            pj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]));
            ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
            vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));

            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        // LEFT_ARM
        istart_+=ndof_right_arm_;
        iend_+=ndof_left_arm_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_name_[i] = "left_arm_j"+std::to_string(i-istart_);
            js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
            pj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]));
            ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
            vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));

            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        // TORSO
        istart_+=ndof_left_arm_;
        iend_+=ndof_torso_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_name_[i] = "torso_j"+std::to_string(i-istart_);
            js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
            pj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]));
            ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
            vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));

            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }

        // HEAD
        istart_+=ndof_torso_;
        iend_+=ndof_head_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_name_[i] = "head_j"+std::to_string(i-istart_);
            js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
            pj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]));
            ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
            vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));

            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        // RIGHT HAND
        istart_+=ndof_head_;
        iend_+=ndof_right_hand_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_name_[i] = "right_hand_j"+std::to_string(i-istart_);
            js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
            pj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]));
            ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
            vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));

            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        // LEFT HAND
        istart_+=ndof_right_hand_;
        iend_+=ndof_left_hand_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_name_[i] = "left_hand_j"+std::to_string(i-istart_);
            js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
            pj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]));
            ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
            vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));

            //jm_interface_.registerHandle(JointModeHandle(joint_name_[i], &joint_mode_[i]));

        }
        registerInterface(&js_interface_);
        registerInterface(&pj_interface_);
        registerInterface(&ej_interface_);
        registerInterface(&vj_interface_);

    }

    void read()
    {	// RIGHT_ARM
        istart_ = 0;
        iend_ = ndof_right_arm_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_position_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDeg(RIGHT_ARM,i));
            joint_velocity_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDotDeg(RIGHT_ARM,i));
            joint_effort_[i] = mm2m(bot_shr_ptr_->GetTorque_mNm(RIGHT_ARM,i));// mNm -> Nm
        }
        // LEFT_ARM
        istart_+=ndof_right_arm_;
        iend_+=ndof_left_arm_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_position_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDeg(LEFT_ARM,i-istart_));
            joint_velocity_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDotDeg(LEFT_ARM,i-istart_));
            joint_effort_[i] = mm2m(bot_shr_ptr_->GetTorque_mNm(LEFT_ARM,i-istart_));// mNm -> Nm
        }
        // TORSO
        istart_+=ndof_left_arm_;
        iend_+=ndof_torso_;
        for(int i=istart_; i<iend_; i++)
        {
            //j1 slave is a copy of j1
            if(i-istart_==2)
            {
                joint_position_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDeg(TORSO,i-istart_-1));
                joint_velocity_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDotDeg(TORSO,i-istart_-1));
                joint_effort_[i] = mm2m(bot_shr_ptr_->GetTorque_mNm(TORSO,i-istart_-1));// mNm -> Nm
            }
            else
            {
                joint_position_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDeg(TORSO,i-istart_));
                joint_velocity_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDotDeg(TORSO,i-istart_));
                joint_effort_[i] = mm2m(bot_shr_ptr_->GetTorque_mNm(TORSO,i-istart_));// mNm -> Nm
            }
        }

        // HEAD
        istart_+=ndof_torso_;
        iend_+=ndof_head_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_position_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDeg(HEAD,i-istart_));
            joint_velocity_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDotDeg(HEAD,i-istart_));
            joint_effort_[i] = mm2m(bot_shr_ptr_->GetTorque_mNm(HEAD,i-istart_));// mNm -> Nm
        }
        // RIGHT HAND
        istart_+=ndof_head_;
        iend_+=ndof_right_hand_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_position_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDeg(RIGHT_HAND,i-istart_));
            joint_velocity_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDotDeg(RIGHT_HAND,i-istart_));
            joint_effort_[i] = mm2m(bot_shr_ptr_->GetTorque_mNm(RIGHT_HAND,i-istart_));// mNm -> Nm
        }
        // LEFT HAND
        istart_+=ndof_right_hand_;
        iend_+=ndof_left_hand_;
        for(int i=istart_; i<iend_; i++)
        {
            joint_position_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDeg(LEFT_HAND,i-istart_));
            joint_velocity_[i] = DEG2RAD(bot_shr_ptr_->GetThetaDotDeg(LEFT_HAND,i-istart_));
            joint_effort_[i] = mm2m(bot_shr_ptr_->GetTorque_mNm(LEFT_HAND,i-istart_));// mNm -> Nm
        }
    }

    void write()
    {
        //if (safety_check())
        bot_shr_ptr_->SetMotorPowerOn();
        // RIGHT_ARM
        istart_ = 0;
        iend_ = ndof_right_arm_;
        for(int i=istart_; i<iend_; i++)
        {
            bot_shr_ptr_->SetStiffness(RIGHT_ARM,i-istart_,1.0);
            bot_shr_ptr_->SetSlewRateProportional(RIGHT_ARM,i-istart_,1.0);
            switch (joint_mode_)
            {
            case VELOCITY:
                bot_shr_ptr_->SetModeThetaDotGc(RIGHT_ARM,i-istart_);
                bot_shr_ptr_->SetThetaDotDeg(RIGHT_ARM,i-istart_,RAD2DEG(joint_velocity_command_[i]));
                break;
            case POSITION:
                bot_shr_ptr_->SetModeThetaGc(RIGHT_ARM,i-istart_);
                bot_shr_ptr_->SetThetaDeg(RIGHT_ARM,i-istart_,RAD2DEG(joint_position_command_[i]));
                break;
            case EFFORT:
                bot_shr_ptr_->SetModeTorqueGc(RIGHT_ARM,i-istart_);
                bot_shr_ptr_->SetTorque_mNm(RIGHT_ARM,i-istart_,m2mm(joint_effort_command_[i]));
                break;
            default:
                break;
            }
        }
        // LEFT_ARM
        istart_+=ndof_right_arm_;
        iend_+=ndof_left_arm_;
        for(int i=istart_; i<iend_; i++)
        {
            bot_shr_ptr_->SetStiffness(LEFT_ARM,i-istart_,1.0);
            bot_shr_ptr_->SetSlewRateProportional(LEFT_ARM,i-istart_,1.0);
            switch (joint_mode_)
            {
            case VELOCITY:
                bot_shr_ptr_->SetModeThetaDotGc(LEFT_ARM,i-istart_);
                bot_shr_ptr_->SetThetaDotDeg(LEFT_ARM,i-istart_,RAD2DEG(joint_velocity_command_[i]));
                break;
            case POSITION:
                bot_shr_ptr_->SetModeThetaGc(LEFT_ARM,i-istart_);
                bot_shr_ptr_->SetThetaDeg(LEFT_ARM,i-istart_,RAD2DEG(joint_position_command_[i]));
                break;
            case EFFORT:
                bot_shr_ptr_->SetModeTorqueGc(LEFT_ARM,i-istart_);
                bot_shr_ptr_->SetTorque_mNm(LEFT_ARM,i-istart_,m2mm(joint_effort_command_[i]));
                break;
            default:
                break;
            }
        }
        // HEAD
        istart_+=ndof_left_arm_;
        iend_+=ndof_head_;
        for(int i=istart_; i<iend_; i++)
        {
            bot_shr_ptr_->SetStiffness(HEAD,i-istart_,1.0);
            bot_shr_ptr_->SetSlewRateProportional(HEAD,i-istart_,1.0);
            switch (joint_mode_)
            {
            case POSITION:
                bot_shr_ptr_->SetModeTheta(HEAD,i-istart_);
                bot_shr_ptr_->SetThetaDeg(HEAD,i-istart_,RAD2DEG(joint_position_command_[i]));
                break;
            default:
                break;
            }
        }
        // RIGHT_HAND
        istart_+=ndof_head_;
        iend_+=ndof_right_hand_;
        for(int i=istart_; i<iend_; i++)
        {
            bot_shr_ptr_->SetStiffness(RIGHT_HAND,i-istart_,1.0);
            bot_shr_ptr_->SetSlewRateProportional(RIGHT_HAND,i-istart_,1.0);
            switch (joint_mode_)
            {
            case VELOCITY:
                bot_shr_ptr_->SetModeThetaDotGc(RIGHT_HAND,i-istart_);
                bot_shr_ptr_->SetThetaDotDeg(RIGHT_HAND,i-istart_,RAD2DEG(joint_velocity_command_[i]));
                break;
            case POSITION:
                bot_shr_ptr_->SetModeThetaGc(RIGHT_HAND,i-istart_);
                bot_shr_ptr_->SetThetaDeg(RIGHT_HAND,i-istart_,RAD2DEG(joint_position_command_[i]));
                break;
            case EFFORT:
                bot_shr_ptr_->SetModeTorqueGc(RIGHT_HAND,i-istart_);
                bot_shr_ptr_->SetTorque_mNm(RIGHT_HAND,i-istart_,m2mm(joint_effort_command_[i]));
                break;
            default:
                break;
            }
        }
        // LEFT_HAND
        istart_+=ndof_right_hand_;
        iend_+=ndof_left_hand_;
        for(int i=istart_; i<iend_; i++)
        {
            bot_shr_ptr_->SetStiffness(LEFT_HAND,i-istart_,1.0);
            bot_shr_ptr_->SetSlewRateProportional(LEFT_HAND,i-istart_,1.0);
            switch (joint_mode_)
            {
            case VELOCITY:
                bot_shr_ptr_->SetModeThetaDotGc(LEFT_HAND,i-istart_);
                bot_shr_ptr_->SetThetaDotDeg(LEFT_HAND,i-istart_,RAD2DEG(joint_velocity_command_[i]));
                break;
            case POSITION:
                bot_shr_ptr_->SetModeThetaGc(LEFT_HAND,i-istart_);
                bot_shr_ptr_->SetThetaDeg(LEFT_HAND,i-istart_,RAD2DEG(joint_position_command_[i]));
                break;
            case EFFORT:
                bot_shr_ptr_->SetModeTorqueGc(LEFT_HAND,i-istart_);
                bot_shr_ptr_->SetTorque_mNm(LEFT_HAND,i-istart_,m2mm(joint_effort_command_[i]));
                break;
            default:
                break;
            }
        }

    }

private:

    int istart_,iend_,ndof_right_arm_, ndof_left_arm_,ndof_head_,ndof_right_hand_,ndof_left_hand_, ndof_torso_ ,ndof_;

    m3::M3Humanoid* bot_shr_ptr_;

    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::EffortJointInterface   ej_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;

    enum joint_mode_t {POSITION,EFFORT,VELOCITY};
    joint_mode_t joint_mode_;

    std::vector<double> joint_effort_command_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_velocity_command_;
    std::vector<std::string> joint_name_;
};


class RosControlComponent : public m3rt::M3Component
{
public:
    RosControlComponent():m3rt::M3Component(MAX_PRIORITY),bot_shr_ptr_(NULL),ros_nh_ptr_(NULL),spinner_ptr_(NULL),hw_ptr_(NULL),cm_ptr_(NULL),skip_loop_(false){RegisterVersion("default",DEFAULT);}
    ~RosControlComponent(){if(spinner_ptr_!=NULL) delete spinner_ptr_; if(hw_ptr_!=NULL) delete hw_ptr_; if(ros_nh_ptr_!=NULL) delete ros_nh_ptr_; if(cm_ptr_!=NULL) delete cm_ptr_;};

    google::protobuf::Message*  GetCommand(){return &status_;} //NOTE make abstract M3Component happy
    google::protobuf::Message*  GetStatus(){return &cmd_;}
    google::protobuf::Message*  GetParam(){return &param_;}

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

    M3BaseStatus* GetBaseStatus(){return status_.mutable_base();} //NOTE make abstract M3Component happy

    bool RosInit(m3::M3Humanoid* bot)
    {
        //std::string ros_node_name = GetName();
        std::string ros_node_name = "";
        int argc = 1;
        char* arg0 = strdup(ros_node_name.c_str());
        char* argv[] = {arg0, 0};

        ros::init(argc, argv, ros_node_name,ros::init_options::NoSigintHandler);
        free (arg0);

        m3rt::M3_INFO("Checking for running roscore... %s\n",GetName().c_str());
        if(ros::master::check()){
            ros_nh_ptr_ = new ros::NodeHandle(ros_node_name);
            spinner_ptr_ = new ros::AsyncSpinner(1); // Use one thread for the external communications
            spinner_ptr_->start();
            // Create the Meka Hardware interface
            hw_ptr_ = new MekaRobotHW(bot,hw_interface_mode_);
            // Create the controller manager
            cm_ptr_ = new controller_manager::ControllerManager(hw_ptr_, *ros_nh_ptr_);
        }
        else
        {
            //ros_nh_ptr_ = NULL;
            m3rt::M3_ERR("Roscore is not running, can not initializate the controller_manager in component %s...\n",GetName().c_str());
            return false;
        }
        return true;
    }

    void RosShutdown()
    {
        if(spinner_ptr_!=NULL)
            spinner_ptr_->stop();
        if(ros_nh_ptr_!=NULL)
            ros_nh_ptr_->shutdown();
    }

private:
    std::string bot_name_, hw_interface_mode_;
    m3::M3Humanoid* bot_shr_ptr_;
    ros::Duration period_;
    ros::NodeHandle* ros_nh_ptr_;
    ros::AsyncSpinner* spinner_ptr_;  // Used to keep alive the ros services in the controller manager
    MekaRobotHW* hw_ptr_;
    controller_manager::ControllerManager* cm_ptr_;
    enum {DEFAULT};
    bool skip_loop_;
    long long loop_cnt_;

};


}

#endif


