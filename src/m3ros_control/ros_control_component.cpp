#include "m3ros_control/ros_control_component.h"

#include <ctime> // Just for monitoring
namespace ros_control_component{

using namespace m3rt;
using namespace std;
using namespace m3;

bool RosControlComponent::LinkDependentComponents()
{
    bot_shr_ptr_=(m3::M3Humanoid*) factory->GetComponent(bot_name_);
    if (bot_shr_ptr_==NULL) {
        m3rt::M3_INFO("M3Humanoid component %s not found for component %s\n",bot_name_.c_str(),GetName().c_str());
    	return false;
	}
    zlift_shr_ptr_ = (m3::M3JointZLift*) factory->GetComponent(zlift_name_);
    if (zlift_shr_ptr_==NULL) {
	m3rt::M3_INFO("M3JointZLift component %s not found for component %s\n",zlift_name_.c_str(),GetName().c_str());
	return false;
	}
    return true;
}

void RosControlComponent::Startup()
{
    period_.fromSec(1.0/static_cast<double>(RT_TASK_FREQUENCY));
    if(!RosInit(bot_shr_ptr_, zlift_shr_ptr_)) //NOTE here the bot_shr_ptr_ is correctly loaded
        skip_loop_ = true;
    INIT_CNT(tmp_dt_status_);
    INIT_CNT(tmp_dt_cmd_);
}

void RosControlComponent::Shutdown()
{
    //RosShutdown();
}

bool RosControlComponent::ReadConfig(const char* cfg_filename)
{
    if (!M3Component::ReadConfig(cfg_filename))
        return false;
    doc["humanoid"] >> bot_name_;
    doc["zlift"] >> zlift_name_;
    doc["hw_interface_mode"] >> hw_interface_mode_;
    if(hw_interface_mode_=="effort" ||  hw_interface_mode_=="position")
        M3_INFO("Selected hardware interface mode %s for component %s\n",hw_interface_mode_.c_str(),GetName().c_str());
    else
    {
        M3_INFO("Wrong hardware interface mode %s for component %s\n",hw_interface_mode_.c_str(),GetName().c_str());
        return false;
    }

    return true;
}

void RosControlComponent::StepStatus()
{
    if(!skip_loop_)
    {
        //SAVE_TIME(start_dt_status_);
        hw_ptr_->read();
        cm_ptr_->update(ros::Time::now(),period_);
        //SAVE_TIME(end_dt_status_);
        //PRINT_TIME(start_dt_status_,end_dt_status_,tmp_dt_status_,"status");
    }
    else
        if(loop_cnt_%1000==0){

            M3_INFO("Component %s is not running, please check if roscore is started\n",GetName().c_str());

        }
}

void RosControlComponent::StepCommand()
{
    if(!skip_loop_)
    {
        //SAVE_TIME(start_dt_cmd_);
        hw_ptr_->write();
        //SAVE_TIME(end_dt_cmd_);
        //PRINT_TIME(start_dt_cmd_,end_dt_cmd_,tmp_dt_cmd_,"cmd");
    }

    loop_cnt_++;
}

}
