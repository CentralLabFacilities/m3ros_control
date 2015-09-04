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
    pwr_shr_ptr_ = (m3::M3Pwr*) factory->GetComponent(pwr_name_);
    if (pwr_shr_ptr_==NULL) {
        m3rt::M3_INFO("M3Pwr component %s not found for component %s\n",pwr_name_.c_str(),GetName().c_str());
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
    doc["pwr_component"] >> pwr_name_;
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
        // read from hardware
        hw_ptr_->read();
        // handle e_stop
        if(!pwr_shr_ptr_->IsMotorPowerOn())
        {
            if(!was_estop_)
            {
                rt_sem_wait(state_mutex_); 
                hw_ptr_->changeStateAll(STATE_CMD_ESTOP);
                rt_sem_signal(state_mutex_); 
                was_estop_ = true;
            }
        }
        else //automatic recover to stop after estop
        {
            if(was_estop_)
            {
                was_estop_ = false;
                rt_sem_wait(state_mutex_); 
                hw_ptr_->changeStateAll(STATE_CMD_STOP);
                rt_sem_signal(state_mutex_); 
            }
        }
        // controller manager update
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
        rt_sem_wait(state_mutex_); 
        // write to hardware
        hw_ptr_->write();
        // publish the ctrl state once every 100 loops
        if(loop_cnt_%100 == 0){
            if (realtime_pub_ptr_->trylock()){
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


bool RosControlComponent::RosInit(m3::M3Humanoid* bot, m3::M3JointZLift* lift)
{
    //std::string ros_node_name = GetName();
    std::string ros_node_name = "meka_roscontrol";
    int argc = 1;
    char* arg0 = strdup(ros_node_name.c_str());
    char* argv[] =
    { arg0, 0 };

    ros::init(argc, argv, ros_node_name,
            ros::init_options::NoSigintHandler);
    free(arg0);
    
    m3rt::M3_INFO("Checking for running roscore... %s\n",
            GetName().c_str());
    if (ros::master::check())
    {
        // first node handler for the controller manager and controllers
        ros_nh_ptr_ = new ros::NodeHandle(ros_node_name);
        // second node handler for the state manager with a separate cb queue
        ros_nh_ptr2_ = new ros::NodeHandle(ros_node_name+"_state_manager");
        cb_queue_ptr = new ros::CallbackQueue();
        ros_nh_ptr2_->setCallbackQueue(cb_queue_ptr);
        
        // Create a rt thread for state manager service handler
        spinner_running_ = true;
        rc=-1;
        rc = rt_thread_create((void*)ros_async_spinner, (void*)this, 1000000);
        //m3rt::M3_INFO("rc: %d\n",(int)rc);
        // Create a std async spinner for the controller mananager services and callbacks
        spinner_ptr_ = new ros::AsyncSpinner(1); // Use one thread for the external communications
        spinner_ptr_->start();
        // Create the Meka Hardware interface
        hw_ptr_ = new MekaRobotHW(bot, lift, hw_interface_mode_);
        
        // Create a realtime publisher for the state
        realtime_pub_ptr_ = new realtime_tools::RealtimePublisher<m3meka_msgs::M3ControlStates>(*ros_nh_ptr2_, "state", 4);
        realtime_pub_ptr_->msg_.group_name.resize(hw_ptr_->getNbGroup());
        realtime_pub_ptr_->msg_.state.resize(hw_ptr_->getNbGroup());
        
        // Create the controller manager
        cm_ptr_ = new controller_manager::ControllerManager(hw_ptr_,
                *ros_nh_ptr_);
        // Advertize the change state service in the dedicated nodehandler/spinner
        srv_ =  ros_nh_ptr2_->advertiseService("change_state", 
            &RosControlComponent::changeStateCallback,this); 
    }
    else
    {
        //ros_nh_ptr_ = NULL;
        m3rt::M3_ERR(
                "Roscore is not running, can not initializate the controller_manager in component %s...\n",
                GetName().c_str());
        return false;
    }
    return true;
}

// asynchronous spinner for ROS implemented as an rt_task to access rt mutex
// has priority one lower than the current task
// the service used in the component accesses a mutex of the higher priority task
// there might be priority inversions to take care of.
void *ros_async_spinner(void * arg)
{
    RosControlComponent * ros_comp_ptr = (RosControlComponent *)arg;
    int prio = ros_comp_ptr->GetPriority();
    
    m3rt::M3_INFO("Starting async spinner thread with priority %d.\n", prio);

    RT_TASK *task;
    task = rt_task_init_schmod(nam2num("ROSSPI"), prio, 0, 0, SCHED_FIFO, 0xFF);
    if (task==NULL)
    {
        m3rt::M3_ERR("Failed to create RT-TASK ROSSPI\n",0);
        return 0;
    }

    rt_allow_nonroot_hrt();
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_soft_real_time();

    // create a mutex for the state resource
    ros_comp_ptr->state_mutex_ = rt_typed_sem_init(nam2num("MUTEX"), 1, BIN_SEM);

    while (ros_comp_ptr->spinner_running_)
    {
        // call all the cb from the callback queue
        ros_comp_ptr->cb_queue_ptr->callAvailable(ros::WallDuration());
        rt_sleep(nano2count(500000000));
    }    

    // destroy the mutex
    if (ros_comp_ptr->state_mutex_ != NULL)
    {
        rt_sem_delete(ros_comp_ptr->state_mutex_);
    }
    return static_cast<void *>(0);
}


}
