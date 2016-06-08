/*
 * meka_robot_hw.h
 *
 *  refactored on: Dec 18, 2015
 *      Author: plueckin
 *  
 *  Author: Guillaume Walck 2014
 *  derived from Antoine's Horau original work
 */

#include "m3ros_control/meka_robot_hw.h"

////////// M3
#include <m3/chains/arm.h>
#include <m3/robots/humanoid.h>
#include <m3/hardware/joint_zlift.h>

////////// Some defs
#define mm2m(a) (mReal((a))/1000) //millimeters to meters
#define m2mm(a) (mReal((a))*1000) //meters to millimeters

using namespace std;
using namespace controller_manager;
using namespace hardware_interface;

namespace ros_control_component {

MekaRobotHW::MekaRobotHW(m3::M3Humanoid* bot_shr_ptr,
        m3::M3JointZLift* zlift_shr_ptr, string hw_interface_mode) :
        bot_shr_ptr_(NULL), zlift_shr_ptr_(NULL) {

    printcount = 0;

    assert(bot_shr_ptr != NULL);
    assert(zlift_shr_ptr != NULL);
    bot_shr_ptr_ = bot_shr_ptr;
    zlift_shr_ptr_ = zlift_shr_ptr;

    Chain_::joint_mode_t mode;
    if (hw_interface_mode == "position")
        mode = Chain_::joint_mode_t::POSITION;
    else if (hw_interface_mode == "effort")
        mode = Chain_::joint_mode_t::EFFORT;
    else if (hw_interface_mode == "velocity")
        mode = Chain_::joint_mode_t::VELOCITY;
    else
        mode = Chain_::joint_mode_t::POSITION;

    chain_map_["zlift"] = Chain_("zlift", 1, mode);
    chain_map_["right_arm"] = Chain_(RIGHT_ARM, bot_shr_ptr->GetNdof(RIGHT_ARM),
            mode);
    chain_map_["left_arm"] = Chain_(LEFT_ARM, bot_shr_ptr->GetNdof(LEFT_ARM),
            mode);
    chain_map_["head"] = Chain_(HEAD, bot_shr_ptr->GetNdof(HEAD), mode);
    chain_map_["torso"] = Chain_(TORSO, bot_shr_ptr->GetNdof(TORSO), mode);
    chain_map_["right_hand"] = Chain_(RIGHT_HAND,
            bot_shr_ptr->GetNdof(RIGHT_HAND), mode);
    chain_map_["left_hand"] = Chain_(LEFT_HAND, bot_shr_ptr->GetNdof(LEFT_HAND),
            mode);

    for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++) {
        vector<joint_value_> *vals = &it->second.values;
        for (size_t i = 0; i < vals->size(); i++) {
            registerHandles(vals->at(i).name, &vals->at(i).position,
                    &vals->at(i).velocity, &vals->at(i).effort,
                    &vals->at(i).stiffness,
                    &vals->at(i).pos_cmd, &vals->at(i).vel_cmd,
                    &vals->at(i).eff_cmd, &vals->at(i).stiff_cmd);

        }
    }

    registerInterface(&js_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&ej_interface_);
    registerInterface(&vj_interface_);
    registerInterface(&jk_interface_);

}

void MekaRobotHW::read() {

    for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++) {
        vector<joint_value_> *vals = &it->second.values;
        if (it->first == "zlift") {
            vals->at(0).position = mm2m(zlift_shr_ptr_->GetPos());
            vals->at(0).velocity = mm2m(zlift_shr_ptr_->GetPosDot());
            vals->at(0).effort = mm2m(zlift_shr_ptr_->GetForce()); // mNm -> Nm
            continue;
        }
        bool t_ = false;
        if (it->first == "torso")
            t_ = true;
        for (size_t i = 0; i < vals->size(); i++) {
            if (t_ && i == 2) {
                vals->at(i).position = DEG2RAD(
                        bot_shr_ptr_->GetThetaDeg(it->second.chain_ref, i - 1));
                vals->at(i).velocity = DEG2RAD(
                        bot_shr_ptr_->GetThetaDotDeg(it->second.chain_ref,
                                i - 1));
                vals->at(i).effort = mm2m(
                        bot_shr_ptr_->GetTorque_mNm(it->second.chain_ref,
                                i - 1)); // mNm -> Nm
            } else {
                vals->at(i).position = DEG2RAD(
                        bot_shr_ptr_->GetThetaDeg(it->second.chain_ref, i));
                vals->at(i).velocity = DEG2RAD(
                        bot_shr_ptr_->GetThetaDotDeg(it->second.chain_ref, i));
                vals->at(i).effort = mm2m(
                        bot_shr_ptr_->GetTorque_mNm(it->second.chain_ref, i)); // mNm -> Nm
            }
        }
    }
}

void MekaRobotHW::write() {

    printcount++;
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

    for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++) {

        // if the group is not disabled or got disabled while running
        if (it->second.enabled || it->second.ctrl_state > STATE_STANDBY)
        {
            // did a disable occur when in a running /ready state ?
            if (!it->second.enabled && it->second.ctrl_state > STATE_STANDBY)
            {
                // directly force the state to standby
                it->second.ctrl_state = STATE_STANDBY;
                // and continue to permit to set the low-level meka controller to a safe state
            }
          
            // in ready state, override joint_commands to freeze movements
            if (it->second.ctrl_state == STATE_READY) {
                it->second.joint_mode = Chain_::joint_mode_t::POSITION; //change this!
                checkCtrlConvergence(it->first);
                freezeJoints(it->first);
            }

            double s_ = 1.0;
            if (it->second.ctrl_state < STATE_READY) {
                s_ = 0.0;
                it->second.joint_mode = Chain_::joint_mode_t::NOT_READY;
            }

            vector<joint_value_> *vals = &it->second.values;

            if (it->first == "zlift") {
                zlift_shr_ptr_->SetDesiredStiffness(std::min(1.0, std::max(0.0, vals->at(0).stiff_cmd)) * s_);
                zlift_shr_ptr_->SetSlewRate(
                        s_ * ((M3JointParam*) zlift_shr_ptr_->GetParam())->max_q_slew_rate());
                switch (it->second.joint_mode) {
                case Chain_::joint_mode_t::VELOCITY:
                    zlift_shr_ptr_->SetDesiredControlMode(JOINT_MODE_THETADOT_GC);
                    if (it->second.frozen)
                        zlift_shr_ptr_->SetDesiredPosDot(m2mm(vals->at(0).frz_cmd));
                    else
                        zlift_shr_ptr_->SetDesiredPosDot(m2mm(vals->at(0).pos_cmd));
                    break;
                case Chain_::joint_mode_t::POSITION:
                    zlift_shr_ptr_->SetDesiredControlMode(JOINT_MODE_THETA_GC);
                    zlift_shr_ptr_->SetDesiredPos(m2mm(vals->at(0).pos_cmd));
                    break;
                case Chain_::joint_mode_t::EFFORT: //not yet implemented
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

            for (size_t i = 0; i < vals->size(); i++) {
                if (t_ || h_) {
                    if (i == 2 && t_) //j1 slave is a copy of j1, do not write j1 slave value to the actuator
                        break;
                    bot_shr_ptr_->SetStiffness(it->second.chain_ref, i, std::min(1.0, std::max(0.0, vals->at(i).stiff_cmd)) * s_);
                    bot_shr_ptr_->SetSlewRateProportional(it->second.chain_ref, i,
                            s_);
                    switch (it->second.joint_mode) {
                    case Chain_::joint_mode_t::VELOCITY: //not yet implemented
                        break;
                    case Chain_::joint_mode_t::POSITION:
                        bot_shr_ptr_->SetModeThetaGc(it->second.chain_ref, i);
                        if (it->second.frozen) {
                            //if(printcount%200)
                            //    m3rt::M3_INFO("%s,%s: sending frozen cmd %f, ctrl cmd: %f\n ", it->first.c_str(), vals->at(i).name.c_str(), vals->at(i).frz_cmd, vals->at(i).pos_cmd);
                            bot_shr_ptr_->SetThetaDeg(it->second.chain_ref, i,
                                    RAD2DEG(vals->at(i).frz_cmd));
                        } else {
                            //if(printcount%200)
                            //    m3rt::M3_INFO("%s,%s: sending CTRL cmd %f, frz cmd: %f\n ", it->first.c_str(), vals->at(i).name.c_str(), vals->at(i).pos_cmd, vals->at(i).frz_cmd);
                            bot_shr_ptr_->SetThetaDeg(it->second.chain_ref, i,
                                    RAD2DEG(vals->at(i).pos_cmd));
                        }
                        break;
                    case Chain_::joint_mode_t::EFFORT: //not yet implemented
                        break;
                    default:
                        break;
                    }
                    continue;
                }
                bot_shr_ptr_->SetStiffness(it->second.chain_ref, i, std::min(1.0, std::max(0.0, vals->at(i).stiff_cmd)) * s_);
                bot_shr_ptr_->SetSlewRateProportional(it->second.chain_ref, i, s_);

                if (ha_ && it->second.ctrl_state < STATE_READY)
                    bot_shr_ptr_->SetModeOff(it->second.chain_ref, i);

                switch (it->second.joint_mode) {
                case Chain_::joint_mode_t::VELOCITY:
                    bot_shr_ptr_->SetModeThetaDotGc(it->second.chain_ref, i);
                    bot_shr_ptr_->SetThetaDotDeg(it->second.chain_ref, i,
                            RAD2DEG(vals->at(i).vel_cmd));
                    break;
                case Chain_::joint_mode_t::POSITION:
                    bot_shr_ptr_->SetModeThetaGc(it->second.chain_ref, i);
                    if (it->second.frozen) {
                        //if(printcount%200)
                        //    m3rt::M3_INFO("%s,%s: sending frozen cmd %f, ctrl cmd: %f\n ", it->first.c_str(), vals->at(i).name.c_str(), vals->at(i).frz_cmd, vals->at(i).pos_cmd);
                        bot_shr_ptr_->SetThetaDeg(it->second.chain_ref, i,
                                RAD2DEG(vals->at(i).frz_cmd));
                    } else {
                        //if(printcount%200)
                        //    m3rt::M3_INFO("%s,%s: sending CTRL cmd %f, frz cmd: %f\n ", it->first.c_str(), vals->at(i).name.c_str(), vals->at(i).pos_cmd, vals->at(i).frz_cmd);
                        bot_shr_ptr_->SetThetaDeg(it->second.chain_ref, i,
                                RAD2DEG(vals->at(i).pos_cmd));
                    }
                    break;
                case Chain_::joint_mode_t::EFFORT:
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
}

void MekaRobotHW::setCtrlAcceptableMirrorError(const double accept_ang_pos,
        const double accept_ang_vel, const double accept_torque,
        const double accept_lin_pos, const double accept_lin_vel,
        const double accept_force) {
    ctrl_acc_mirror_error_.linear[Chain_::joint_mode_t::POSITION] =
            accept_lin_pos;
    ctrl_acc_mirror_error_.linear[Chain_::joint_mode_t::VELOCITY] =
            accept_lin_vel;
    ctrl_acc_mirror_error_.linear[Chain_::joint_mode_t::EFFORT] = accept_force;
    ctrl_acc_mirror_error_.angular[Chain_::joint_mode_t::POSITION] =
            accept_ang_pos;
    ctrl_acc_mirror_error_.angular[Chain_::joint_mode_t::VELOCITY] =
            accept_ang_vel;
    ctrl_acc_mirror_error_.angular[Chain_::joint_mode_t::EFFORT] =
            accept_torque;
}

int MekaRobotHW::changeState(const int state_cmd, string group_name) {
    int ret = 0;
    map_it_t it = chain_map_.find(group_name);
    if (it != chain_map_.end()) {
        switch (state_cmd) {
          
        case STATE_CMD_DISABLE:
            it->second.enabled = false;
            if (it->second.ctrl_state == STATE_RUNNING) {
                //switch controller off
                m3rt::M3_INFO("%s: You should switch controllers off !\n",
                        group_name.c_str());
            }
            if (it->second.ctrl_state != STATE_ESTOP) {
                m3rt::M3_INFO("%s: Disabled but in ctrl state Standby \n", group_name.c_str());
                // leave the write loop set to state_standby if not already set
                // it->second.ctrl_state = STATE_STANDBY;
            }
            it->second.frozen = false;
            it->second.allow_running = false;
            break;

        case STATE_CMD_ENABLE:
            if (!it->second.enabled) {
                it->second.enabled = true;
                it->second.frozen = false;
                it->second.allow_running = false;
            }
            break;

        case STATE_CMD_ESTOP:
            if (it->second.enabled)
            {
                if (it->second.ctrl_state == STATE_RUNNING) {
                    //switch controller off
                    m3rt::M3_INFO("%s: You should switch controllers off !\n",
                            group_name.c_str());
                }
            }

            if (it->second.ctrl_state != STATE_ESTOP) {
                m3rt::M3_INFO("%s: ESTOP detected\n", group_name.c_str());
            }

            it->second.ctrl_state = STATE_ESTOP;
            it->second.frozen = false;
            it->second.allow_running = false;
            break;

        case STATE_CMD_STOP:

            if (it->second.ctrl_state == STATE_RUNNING)
                it->second.allow_running = false;
            it->second.ctrl_state = STATE_STANDBY;
            if (it->second.enabled)
                m3rt::M3_INFO("%s: in standby state\n ", group_name.c_str());
            it->second.frozen = false;

            break;

        case STATE_CMD_FREEZE:
            if (it->second.enabled)
            {
                if (it->second.ctrl_state == STATE_RUNNING)
                    it->second.allow_running = false;
                // cannot go to freeze if previously in e-stop
                // go to standby first
                if (it->second.ctrl_state == STATE_ESTOP)
                    ret = -3;
                else {
                    it->second.ctrl_state = STATE_READY;
                    m3rt::M3_INFO("%s: in freeze state\n ", group_name.c_str());
                }
              }
            break;

        case STATE_CMD_START:
            if (it->second.enabled)
            {
                if (it->second.ctrl_state != STATE_RUNNING) {
                    // cannot go to run if previously in e-stop
                    // go to standby first
                    if (it->second.ctrl_state == STATE_ESTOP) {
                        ret = -3;
                        break;
                    }

                    if (it->second.allow_running) {
                        it->second.ctrl_state = STATE_RUNNING;
                        it->second.frozen = false;
                        m3rt::M3_INFO(
                                "%s: allowed to run, putting in running state\n ",
                                group_name.c_str());
                    } else {
                        // set to ready to allow for convergence
                        // to be checked and freeze to be triggered
                        it->second.ctrl_state = STATE_READY;
                        m3rt::M3_ERR(
                                "%s: Controller did not converge to freeze position, \
                                     cannot switch to running...\n",
                                group_name.c_str());
                        ret = -2;
                    }
                }
            }
            break;
        default:
            m3rt::M3_ERR("Unknown state command %d...\n", state_cmd);
            ret = -1;
            break;
        }
    } else {
        m3rt::M3_ERR("Unknown group name %s...\n", group_name.c_str());
        ret = -1;
    }
    return ret;
}

int MekaRobotHW::changeStateAll(const int state_cmd) {
    int ret = 0;
    for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++) {
        int ret_tmp = changeState(state_cmd, it->first);
        if (ret_tmp != 0) {
            ret = ret_tmp;
        }
    }
    return ret;
}

void MekaRobotHW::getPublishableState(m3meka_msgs::M3ControlStates &msg) {
    size_t i = 0;
    for (map_it_t it = chain_map_.begin(); it != chain_map_.end(); it++) {
        msg.group_name[i] = it->first;
        if (it->second.enabled)
            msg.state[i] = it->second.ctrl_state;
        else
            msg.state[i] = STATE_DISABLE;
        ++i;
    }
}

int MekaRobotHW::getCtrlState(const string group_name) {
    map_it_t it = chain_map_.find(group_name);
    if (it != chain_map_.end())
    {
        if (it->second.enabled)
            return it->second.ctrl_state;
        else
            return STATE_DISABLE;
    }
    else
        return 0;
}

int MekaRobotHW::getNbGroup() {
    return chain_map_.size();
}

//---private functions---

void MekaRobotHW::registerHandles(string name, double* pos, double* vel, double* stiffness,
        double* eff, double* poscmd, double* effcmd, double* velcmd, double* stiffcmd) {
    js_interface_.registerHandle(JointStateHandle(name, pos, vel, eff));
    pj_interface_.registerHandle(
            JointHandle(js_interface_.getHandle(name), poscmd));
    ej_interface_.registerHandle(
            JointHandle(js_interface_.getHandle(name), effcmd));
    vj_interface_.registerHandle(
            JointHandle(js_interface_.getHandle(name), velcmd));
    // not an additional joint, just an access to the stiffness with a different joint name
    // (idea by CentroEPiaggio/kuka-lwr/lwr_hw)
    jk_interface_.registerHandle(
            JointHandle(JointStateHandle(name+std::string("_stiffness"), stiffness, stiffness, stiffness),
                        stiffcmd));
}

void MekaRobotHW::freezeJoints(string group_name) {
    vector<joint_value_> *vals = &chain_map_[group_name].values;
    switch (chain_map_[group_name].joint_mode) {
    case Chain_::joint_mode_t::VELOCITY:
        // freeze means zero velocity
        for (size_t i = 0; i < vals->size(); i++) {
            if (!chain_map_[group_name].frozen)
                vals->at(i).frz_cmd = 0.0;
            //vals->at(i).vel_cmd = vals->at(i).frz_cmd;
        }
        chain_map_[group_name].frozen = true;
        break;
    case Chain_::joint_mode_t::POSITION:
        // freeze means maintain current position
        for (size_t i = 0; i < vals->size(); i++) {
            if (!chain_map_[group_name].frozen)
                m3rt::M3_INFO(
                        "%s,%s: Frozen to current position %f, prev cmd: %f\n ",
                        group_name.c_str(), vals->at(i).name.c_str(),
                        vals->at(i).position, vals->at(i).pos_cmd);

            if (!chain_map_[group_name].frozen)
                vals->at(i).frz_cmd = vals->at(i).position;
            //vals->at(i).pos_cmd = vals->at(i).frz_cmd;
        }
        chain_map_[group_name].frozen = true;
        break;
    case Chain_::joint_mode_t::EFFORT:
        // freeze means maintain current effort ?
        for (size_t i = 0; i < vals->size(); i++) {
            if (!chain_map_[group_name].frozen)
                vals->at(i).frz_cmd = vals->at(i).effort;
            //vals->at(i).eff_cmd = vals->at(i).frz_cmd;
        }
        chain_map_[group_name].frozen = true;
        break;
    default:
        break;
    }
}

bool MekaRobotHW::checkCtrlConvergence(string group_name) {
    converged = true;
    vector<joint_value_> *vals = &chain_map_[group_name].values;
    switch (chain_map_[group_name].joint_mode) {
    case Chain_::joint_mode_t::VELOCITY:
        // acceptable error is not the same in rotation and in translation
        if (group_name == "z_lift")
            epsilon =
                    ctrl_acc_mirror_error_.linear[Chain_::joint_mode_t::VELOCITY];
        else
            epsilon =
                    ctrl_acc_mirror_error_.angular[Chain_::joint_mode_t::VELOCITY];

        for (size_t i = 0; i < vals->size(); i++) {
            vals->at(i).err = fabs(vals->at(i).vel_cmd);
            if (vals->at(i).err > epsilon) {
                converged = false;
                break;
            }
        }
        break;
    case Chain_::joint_mode_t::POSITION:
        // acceptable error is not the same in rotation and in translation
        if (group_name == "z_lift")
            epsilon =
                    ctrl_acc_mirror_error_.linear[Chain_::joint_mode_t::POSITION];
        else
            epsilon =
                    ctrl_acc_mirror_error_.angular[Chain_::joint_mode_t::POSITION];

        for (size_t i = 0; i < vals->size(); i++) {
            vals->at(i).err = fabs(vals->at(i).pos_cmd - vals->at(i).position);
            if (vals->at(i).err > epsilon) {
                converged = false;
                if (chain_map_[group_name].allow_running)
                    m3rt::M3_DEBUG(
                            "%s: joint %d, shows a position/angular difference %f > %f \n",
                            group_name.c_str(), i, vals->at(i).err, epsilon);
                break;
            }
        }
        break;
    case Chain_::joint_mode_t::EFFORT:
        // acceptable error is not the same in rotation and in translation
        if (group_name == "z_lift")
            epsilon =
                    ctrl_acc_mirror_error_.linear[Chain_::joint_mode_t::EFFORT];
        else
            epsilon =
                    ctrl_acc_mirror_error_.angular[Chain_::joint_mode_t::EFFORT];

        for (size_t i = 0; i < vals->size(); i++) {
            vals->at(i).err = fabs(vals->at(i).eff_cmd - vals->at(i).effort);
            if (vals->at(i).err > epsilon) {
                converged = false;
                break;
            }
        }
        break;
    default:
        converged = false;
        break;
    }
    chain_map_[group_name].allow_running = converged;
    return converged;
}

}
