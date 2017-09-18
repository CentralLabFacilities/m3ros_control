#include "../../include/m3ros_diagnostics/m3ros_monitor.h"

using namespace std;
using namespace m3rt;

namespace m3ros_diagnostics
{

static bool ros_thread_active=false;

static void* ros_diagnostics_thread(void * arg)
{
    M3RosMonitor * monitor = (M3RosMonitor *)arg;
    ros_thread_active=true;

    while(monitor->spinner_running_) {
          for (int i = 0; i < monitor->publishers.size(); i++)
              //monitor->//publish all Analyzer data & stuff
          //ros::spinOnce();
          rt_sleep(nano2count(1000000));//usleep(10 000); // 10 Hz
    }

    M3_INFO("Exiting M3 Ros Monitor Thread\n",0);
    ros_thread_active=false;
    return static_cast<void *>(0);
}


M3RosMonitor::M3RosMonitor() : M3Component(MAX_PRIORITY) {
    RegisterVersion("default", DEFAULT);

}

M3RosMonitor::~M3RosMonitor() {

}

M3BaseStatus* M3RosMonitor::GetBaseStatus() {
    return status_.mutable_base();
}

google::protobuf::Message* M3RosMonitor::GetCommand() {
    return &status_;
}
google::protobuf::Message* M3RosMonitor::GetStatus() {
    return &cmd_;
}
google::protobuf::Message* M3RosMonitor::GetParam() {
    return &param_;
}


bool M3RosMonitor::LinkDependentComponents() {

    //for(int i = 0; i <= factory->GetNumComponents(); i++) {
    //    components.push_back(factory->GetComponent(i));
    //}

    m3rt::M3_INFO("%s: LinkDependentComponents success!\n", GetName().c_str());
    return true;
}

void M3RosMonitor::Startup() {

    if (!RosInit()) {
        SetStateDisabled();
    } else {
        SetStateSafeOp();
    }

}

void M3RosMonitor::Shutdown() {
    RosShutdown();
}

bool M3RosMonitor::ReadConfig(const char* cfg_filename) {
    if (!M3Component::ReadConfig(cfg_filename))
        return false;

    for(int i=0; i<doc["ros_publishers"].size(); i++) {
        string name;
        doc["ros_publishers"][i]>>name;
        AddPublisher(name);
    }

    return true;
}

void M3RosMonitor::StepStatus() {
    //aggregate the information from the RT server -> convert it in another thread to spare the RT system
    if (IsStateError())
        return;

    if (!IsStateDisabled()) {

    }
}

void M3RosMonitor::StepCommand() {
    return;
}

bool M3RosMonitor::AddPublisher(string name) {

    int idx=factory->GetComponentIdx(name);
    if (idx>=0) {
        for(int i=0;i<components.size(); i++) {
            if(components[i]->GetName().compare(name)==0) {
                return true;
            }
        }
        M3_INFO("Providing ROS topic for component: %s\n",name.c_str());
        components.push_back(factory->GetComponent(idx));
        return true;
    }


    M3_WARN("M3RtRosService component not available: %s\n",name.c_str());
    return false;
}

void M3RosMonitor::Publish() {
    for(int i=0;i<components.size(); i++) {

    }
}

bool M3RosMonitor::RosInit() {
    std::string ros_node_name = "m3ros_monitor";
    int argc = 1;
    char* arg0 = strdup(ros_node_name.c_str());
    char* argv[] = { arg0, 0 };

    ros::init(argc, argv, ros_node_name, ros::init_options::NoSigintHandler);
    free(arg0);

    m3rt::M3_INFO("Checking for running roscore... %s\n", GetName().c_str());

    if (ros::master::check()) {
        ros_nh_ptr_ = new ros::NodeHandle(ros_node_name);

        spinner_running_ = true;

        rc = -1;
        rc = rt_thread_create((void*) ros_diagnostics_thread, (void*) this, 10000); // 10 Hz??
        //pthread_create((pthread_t *)&rc, NULL, (void *(*)(void *))ros_diagnostics_thread, (void*)this); // we dont need an rt thread?


    } else {
        m3rt::M3_ERR("Roscore is not running, cannot start %s...\n", GetName().c_str());
        return false;
    }

    return true;
}

void M3RosMonitor::RosShutdown() {
    m3rt::M3_INFO("Shutting down m3ros_monitor interface\n");

    spinner_running_ = false;
    if (rc) {
        m3rt::M3_INFO("Waiting for RT spinner to stop...\n");
        rt_thread_join(rc);
        rc = -1;
    }
}


}
