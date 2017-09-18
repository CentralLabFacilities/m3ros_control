/*
 * meka_robot_hw.cpp
 *
 *  refactored on: Dec 18, 2015
 *      Author: plueckin
 *  
 *  Author: Guillaume Walck 2014
 *  derived from Antoine's Horau original work
 */

#include <m3/robots/humanoid.h>
#include <m3/hardware/joint_zlift.h>

#include <m3meka_msgs/M3ControlStateChange.h>
#include <m3meka_msgs/M3ControlState.h>
#include <m3meka_msgs/M3ControlStates.h>
#include <m3meka_msgs/M3ControlStateErrorCodes.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <controller_manager/controller_manager.h>

#include <stdio.h>

// master states

#define STATE_UNKNOWN   0
#define STATE_DISABLE   1
#define STATE_ENABLE    2
#define STATE_ESTOP     3
#define STATE_STANDBY   4
#define STATE_READY     5
#define STATE_RUNNING   6

// state transitions command

#define STATE_CMD_DISABLE   1
#define STATE_CMD_ENABLE    2
#define STATE_CMD_ESTOP     3
#define STATE_CMD_STOP      4
#define STATE_CMD_FREEZE    5
#define STATE_CMD_START     6

namespace m3ros_control {

class MekaRobotHW: public hardware_interface::RobotHW {
public:

    MekaRobotHW(m3::M3Humanoid* bot_shr_ptr, m3::M3JointZLift* zlift_shr_ptr,
            std::string hw_interface_mode);

    virtual ~MekaRobotHW() {};

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

    void registerHandles(std::string name, double* pos, double* vel, double* eff, 
        double* stiffness, double* poscmd, double* velcmd, double* effcmd, double* stiffcmd);

    struct joint_value_ {
        std::string name;
        double position;
        double velocity;
        double effort;
        double stiffness;
        double pos_cmd;
        double eff_cmd;
        double vel_cmd;
        double frz_cmd;
        double err;
        double stiff_cmd;
    };

    struct Chain_ {

        enum joint_mode_t {
            POSITION, EFFORT, VELOCITY, NOT_READY
        };

        Chain_() {
        }
        ;

        Chain_(std::string name_, int ndof_, joint_mode_t joint_mode_ = NOT_READY,
                int ctrl_state_ = STATE_ESTOP, bool frozen_ = false,
                bool allow_running_ = false, bool enabled_ = false) :
                chain_ref(RIGHT_ARM), //default
                name(name_), ndof(ndof_), joint_mode(joint_mode_), ctrl_state(
                        ctrl_state_), frozen(frozen_), allow_running(allow_running_),
                        enabled(enabled_) {
            values.resize(ndof);
            if (name == "zlift")
                values[0].position = 0.30;
            for (int i = 0; i < ndof; i++) {
                values[i].name = name + "_j" + std::to_string(i);
                values[i].stiff_cmd = 1.0;
                values[i].stiffness = 1.0;
            }
        }
        Chain_(M3Chain chain_ref_, int ndof_, joint_mode_t joint_mode_ = NOT_READY,
                int ctrl_state_ = STATE_ESTOP, bool frozen_ = false,
                bool allow_running_ = false, bool enabled_ = false) :
                chain_ref(chain_ref_), ndof(ndof_), joint_mode(joint_mode_), ctrl_state(
                        ctrl_state_), frozen(frozen_), allow_running(allow_running_),
                        enabled(enabled_) {
            name = getStringFromEnum(chain_ref);
            values.resize(ndof);
            if (name == "zlift")
                values[0].position = 0.30;
            for (int i = 0; i < ndof; i++) {
                values[i].name = name + "_j" + std::to_string(i);
                values[i].stiff_cmd = 1.0;
                values[i].stiffness = 1.0;
            }
        }

        std::string getStringFromEnum(M3Chain e) {
            switch (e) {
            case RIGHT_ARM:
                return "right_arm";
            case LEFT_ARM:
                return "left_arm";
            case TORSO:
                return "torso";
            case HEAD:
                return "head";
            case RIGHT_HAND:
                return "right_hand";
            case LEFT_HAND:
                return "left_hand";
            default:
                throw "Bad M3Chain Enum!";
            }
        }

        M3Chain chain_ref;
        std::string name;
        int ndof;
        joint_mode_t joint_mode;
        int ctrl_state;
        bool frozen;
        bool allow_running;
        std::vector<joint_value_> values;
        bool enabled;
    };

    typedef std::map<MekaRobotHW::Chain_::joint_mode_t, double> mirror_error_map_t;
    struct ctrl_error_ {
        mirror_error_map_t linear;
        mirror_error_map_t angular;
    } ctrl_acc_mirror_error_;

    typedef std::map<std::string, MekaRobotHW::Chain_> map_t;
    typedef map_t::iterator map_it_t;

    std::map<std::string, Chain_> chain_map_;

    // store a freeze state (cur pos/ zero vel/ cur trq , depends on mode)
    // and a freeze command
    void freezeJoints(std::string group_name);

    /* measure difference between command and current pos
     to tell if controllers have converged */
    bool checkCtrlConvergence(std::string group_name);

};

}
