#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/hardware_interface.h>

#include <stdio.h>

namespace ros_control_component {

class MekaRobotHW: public hardware_interface::RobotHW {
public:

    MekaRobotHW(m3::M3Humanoid* bot_shr_ptr, m3::M3JointZLift* zlift_shr_ptr,
            std::string hw_interface_mode);

    ~MekaRobotHW();

    void read();

    void write();

    void setCtrlAcceptableMirrorError(const double accept_ang_pos,
            const double accept_ang_vel, const double accept_torque,
            const double accept_lin_pos, const double accept_lin_vel,
            const double accept_force);

    // Returns the number of groups
    int getNbGroup();

    int getCtrlState(const std::string group_name);

    // fills up the vectors for publishing
    void getPublishableState(m3meka_msgs::M3ControlStates &msg);

    // change state for all the groups
    int changeStateAll(const int state_cmd);

    // try change the state to requested state_cmd for given group
    int changeState(const int state_cmd, std::string group_name);

private:

    m3::M3Humanoid* bot_shr_ptr_;
    m3::M3JointZLift* zlift_shr_ptr_;

    hardware_interface::JointStateInterface js_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::EffortJointInterface ej_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;

    bool converged;
    double epsilon;
    int printcount;

    void registerHandles(std::string name, double* pos, double* vel,
            double* eff, double* poscmd, double* effcmd, double* velcmd);

    struct joint_value_ {
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

    struct Chain_;

    std::map<std::string, Chain_> chain_map_;

    // store a freeze state (cur pos/ zero vel/ cur trq , depends on mode)
    // and a freeze command
    void freezeJoints(std::string group_name);

    /* measure difference between command and current pos
     to tell if controllers have converged */
    bool checkCtrlConvergence(std::string group_name);

};

}
