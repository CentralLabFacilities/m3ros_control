#include "m3ros_control/ros_control_component.h"

#include <ctime>
#include <rtai_shm.h>

#define RT_TASK_FREQUENCY_MEKA_OMNI_SHM 100
#define RT_TIMER_TICKS_NS_MEKA_OMNI_SHM (1000000000 / RT_TASK_FREQUENCY_MEKA_OMNI_SHM)      //Period of rt-timer

#define ROS_ASYNC_SP "ROSSPI"
#define ROS_MAIN_ASYNC_SP "ROSMSP"

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
	ros_comp_ptr->state_mutex_ = rt_typed_sem_init(nam2num("MUTEX"), 1,
	BIN_SEM);

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

RosControlComponent::RosControlComponent() :
		m3rt::M3Component(MAX_PRIORITY), state_mutex_(NULL), spinner_running_(
				false), was_estop_(true), cb_queue_ptr(NULL), accept_ang_pos_(
				0.0), accept_ang_vel_(0.0), accept_torque_(0.0), accept_lin_pos_(
				0.0), accept_lin_vel_(0.0), accept_force_(0.0), bot_shr_ptr_(
		NULL), zlift_shr_ptr_(NULL), obase_shr_ptr_(NULL), pwr_shr_ptr_(NULL), ros_nh_ptr_(
		NULL), ros_nh_ptr2_(NULL), spinner_ptr_(NULL), realtime_pub_ptr_(
		NULL), hw_ptr_(NULL), cm_ptr_(NULL), skip_loop_(false), loop_cnt_(0), wait_sds_(
				true) {
	RegisterVersion("default", DEFAULT);
}

RosControlComponent::~RosControlComponent() {

	if (cm_ptr_ != NULL)
		delete cm_ptr_;

	if (realtime_pub_ptr_ != NULL)
		delete realtime_pub_ptr_;

	if (hw_ptr_ != NULL)
		delete hw_ptr_;

	if (obase_ptr_ != NULL)
		delete obase_ptr_;

	if (spinner_ptr_ != NULL)
		delete spinner_ptr_;

	if (ros_nh_ptr_ != NULL)
		delete ros_nh_ptr_;

	if (ros_nh_ptr2_ != NULL)
		delete ros_nh_ptr2_;

	if (bot_shr_ptr_ != NULL)
		delete bot_shr_ptr_;

	if (zlift_shr_ptr_ != NULL)
		delete zlift_shr_ptr_;

	if (pwr_shr_ptr_ != NULL)
		delete pwr_shr_ptr_;

	if (obase_shr_ptr_ != NULL)
		delete obase_shr_ptr_;

	if (obase_shm_shr_ptr_ != NULL)
		delete obase_shm_shr_ptr_;

	if (obase_ja_shr_ptr_ != NULL)
		delete obase_ja_shr_ptr_;

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
	obase_shr_ptr_ = (m3::M3Omnibase*) factory->GetComponent(obase_name_);
	if (obase_shr_ptr_ == NULL) {
		m3rt::M3_INFO("M3Omnibase component %s not found for component %s\n",
				obase_name_.c_str(), GetName().c_str());
		return false;
	}
	obase_shm_shr_ptr_ = (m3::M3OmnibaseShm*) factory->GetComponent(
			obase_shm_name_);
	if (obase_shm_shr_ptr_ == NULL) {
		m3rt::M3_INFO("M3OmnibaseShm component %s not found for component %s\n",
				obase_shm_name_.c_str(), GetName().c_str());
		return false;
	}
	obase_ja_shr_ptr_ = (m3::M3JointArray*) factory->GetComponent(
			obase_jointarray_name_);
	if (obase_ja_shr_ptr_ == NULL) {
		m3rt::M3_INFO("M3JointArray component %s not found for component %s\n",
				obase_jointarray_name_.c_str(), GetName().c_str());
		return false;
	}

	return true;
}

void RosControlComponent::Startup() {
	period_.fromSec(1.0 / static_cast<double>(RT_TASK_FREQUENCY));

	if (!RosInit()) //NOTE here the bot_shr_ptr_ is correctly loaded
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
	doc["omnibase"] >> obase_name_;
	doc["omnibase_shm"] >> obase_shm_name_;
	doc["omnibase_jointarray"] >> obase_jointarray_name_;
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

	if (!skip_loop_) {
		//SAVE_TIME(start_dt_status_);
		// read from hardware
		hw_ptr_->read();
		// handle e_stop
		if (!pwr_shr_ptr_->IsMotorPowerOn()) {
			rt_sem_wait(state_mutex_);
			hw_ptr_->changeStateAll(STATE_CMD_ESTOP);
			if(obase_ptr_->is_running()) {
                obase_ptr_->changeState(STATE_CMD_ESTOP);
			}
			rt_sem_signal(state_mutex_);
			was_estop_ = true;
		} else {        //automatic recover to stop after estop
			if (was_estop_) {
				was_estop_ = false;
				rt_sem_wait(state_mutex_);
				hw_ptr_->changeStateAll(STATE_CMD_STOP);
				if(obase_ptr_->is_running()) {
				    obase_ptr_->changeState(STATE_CMD_STOP);
                }
				rt_sem_signal(state_mutex_);
			}
		}
		//check whether obase rt thread is still running
		if(obase_ptr_->is_running() && obase_ptr_->is_sds_ended()) {
		    obase_ptr_->shutdown();
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
				obase_ptr_->getPublishableState(realtime_pub_ptr_->msg_);
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

bool RosControlComponent::RosInit() {
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

		// Create the Meka Hardware interface
		m3rt::M3_INFO("Starting hardware control...\n");
		hw_ptr_ = new MekaRobotHW(bot_shr_ptr_, zlift_shr_ptr_,
				hw_interface_mode_);
		hw_ptr_->setCtrlAcceptableMirrorError(accept_ang_pos_, accept_ang_vel_,
				accept_torque_, accept_lin_pos_, accept_lin_vel_,
				accept_force_);

		// Create the Omnibase control interface
		m3rt::M3_INFO("Starting omnibase control...\n");
		obase_ptr_ = new OmnibaseCtrl(obase_shr_ptr_, obase_shm_shr_ptr_,
				obase_ja_shr_ptr_, ros_node_name);

		// Create a realtime publisher for the state
		realtime_pub_ptr_ = new realtime_tools::RealtimePublisher<
				m3meka_msgs::M3ControlStates>(*ros_nh_ptr2_, "state", 4);
		realtime_pub_ptr_->msg_.group_name.resize(hw_ptr_->getNbGroup()+1); //+1 for base
		realtime_pub_ptr_->msg_.state.resize(hw_ptr_->getNbGroup()+1);

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

	if (obase_ptr_ && obase_ptr_->is_running()) {
		m3rt::M3_INFO("Shutting down omnibase control...\n");
		obase_ptr_->shutdown();
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
				if (req.command.group_name[i] == "base") {
				    rt_sem_wait(this->state_mutex_);
                    ret_tmp = obase_ptr_->changeState(req.command.state[i]);
                    rt_sem_signal(this->state_mutex_);
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
