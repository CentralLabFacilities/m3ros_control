#pragma once

#include <m3rt/base/component.h>


namespace m3ros_diagnostics
{

class M3RosMonitor: public m3rt::M3Component
{
public:
    M3RosMonitor();
    ~M3RosMonitor();

    google::protobuf::Message* GetCommand();
    google::protobuf::Message* GetStatus();
    google::protobuf::Message* GetParam();

    friend void ros_diagnostics_thread(void * arg);

protected:
    bool LinkDependentComponents();
    void Startup();
    void Shutdown();
    bool ReadConfig(const char* filename);
    void StepStatus();
    void StepCommand();

    M3RosMonitorStatus status_;
    M3RosMonitorCommand cmd_;
    M3RosMonitorParam param_;

    M3BaseStatus* GetBaseStatus();

    bool AddPublisher(std::string name);

    bool RosInit();
    void RosShutdown();

private:
    enum{ DEFAULT }; // version

    std::vector<m3rt::M3Component *> components;
    std::vector<ros::Publisher> publishers;

    ros::NodeHandle* ros_nh_ptr_;

    long rc;

    bool spinner_running_;

};

}
