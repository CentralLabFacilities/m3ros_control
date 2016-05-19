#!/usr/bin/env python

# author Guillaume WALCK (2015)

import rospy
from actionlib import SimpleActionServer
from m3meka_msgs.msg import M3ControlStates, M3ControlStateErrorCodes
from m3meka_msgs.srv import M3ControlStateChange, M3ControlStateChangeRequest
from m3meka_msgs.msg import M3StateChangeAction, M3StateChangeFeedback, \
    M3StateChangeResult, M3StateChangeGoal
from controller_manager_msgs.srv import SwitchController, \
    SwitchControllerRequest, ListControllersRequest, \
    ListControllers


class MekaStateManager(object):
    """
        Meka StateManager interacting with m3ros_control and controllers
    """
    _feedback = M3StateChangeFeedback()
    _result = M3StateChangeResult()

    def __init__(self, name, m3roscontrol_name):
        """
        Initializes actionlib and access to controller manager
        and m3ros_control
        @param name - name of the action server
        @param m3roscontrol_name - name of the m3roscontrol loop
        """
        self._action_name = name
        self._control_name = m3roscontrol_name
        self._as = SimpleActionServer(self._action_name,
                                      M3StateChangeAction,
                                      execute_cb=self.execute_cb,
                                      auto_start=False)

        #self._state_listener = \
        #    rospy.Subscriber(m3roscontrol_name+"_state_manager/state", M3ControlStates,
        #                     self.state_callback, queue_size=1)

        self._current_state = M3ControlStates()

        self._cm_list_client = None
        self._cm_switch_client = None
        self._state_client = None
        if self.init_service_client():
            self._controller_list = {}
            self.init_controller_list()
            self._as.start()
            rospy.loginfo("Started %s action server", self._action_name)
        else:
            rospy.logerr("Not starting %s action server, \
                         due to missing services", self._action_name)
            rospy.signal_shutdown("missing services")
        
    def execute_cb(self, goal):
        """
        action lib goal execution
        @param goal - goal of the actionlib
        """
        goal_valid = True
        if goal.strategy == 0:
            goal_valid = False

        if goal.strategy == M3StateChangeGoal.RETRY_N_TIMES:
            if goal.retries == 0:
                goal_valid = False

        if goal_valid:
            # init internal vars
            success = True
            commanded_groups = goal.command.group_name
            commanded_states = goal.command.state
            rate = rospy.Rate(2)
            # initialize the feedback
            self._feedback.group_already_changed = 0

            strategy = goal.strategy
            if strategy > 4:
                rospy.logerr("Unknown strategy %d, applying BEST_POSSIBLE",
                             strategy)
                strategy = M3StateChangeGoal.BEST_POSSIBLE

            # start executing the action
            #self._feedback.current_group = group
            self._feedback.change_attempt = 0

            # if strategy is retry N times
            if strategy == M3StateChangeGoal.RETRY_N_TIMES:
                retries = goal.retries
            else:
                retries = 1

            controller_resetted = False

            # while retries
            while retries:
                self._feedback.change_attempt += 1
                # if not keep retrying decrement retries
                if strategy != M3StateChangeGoal.KEEP_TRYING:
                    retries -= 1

                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break

                # try to change state of that group
                req = M3ControlStateChangeRequest()
                req.command.group_name = commanded_groups
                req.command.state = commanded_states

                try:
                    resp = self._state_client(req)
                except rospy.ServiceException:
                    rospy.logerr("Change_state services not available")
                    rospy.signal_shutdown("Change_state \
                                          services not available")

                # if successull break this while loop to continue
                # to next group
                if resp.error_code.val == M3ControlStateErrorCodes.SUCCESS:
                    break

                # cleanup the group command/state for next attempd
                groups_to_reset = []
                groups_to_retry = []
                states_to_retry = []
                for group_name, state, commanded_state in zip(resp.result.group_name, resp.result.state, commanded_states):
                    if state != commanded_state:
                        groups_to_retry.append(group_name)
                        states_to_retry.append(commanded_state)
                        # find the groups that made it to freeze to reset their controllers
                        if state == M3ControlStates.FREEZE:
                            groups_to_reset.append(group_name)
                commanded_groups = groups_to_retry
                commanded_states = states_to_retry

                # if halt on failure, finish the action lib
                if strategy == M3StateChangeGoal.HALT_ON_FAILURE:
                    self._result.result = resp.result
                    rospy.logerr("Changing control state of group %s\
                    failed, halt on failure requested", str(groups_to_retry))
                    self._as.set_aborted(self._result)
                    return
                # if best effort then we are happy with this attempt
                # to next group
                if strategy == M3StateChangeGoal.BEST_POSSIBLE:
                    self._result.result = resp.result
                    self._as.set_succeeded(self._result)
                    return
                # if CONTROLLER_NOT_CONVERGED and group not yet resetted
                if (resp.error_code.val ==
                        M3ControlStateErrorCodes.CONTROLLER_NOT_CONVERGED):
                    if not controller_resetted:
                        # try reset the controllers
                        reset_ret = self.reset_controllers(groups_to_reset)
                        # remember we already switched
                        # (do not reset in next attempt)
                        controller_resetted = True
                        # if success, continue next retry if any
                        if reset_ret:
                            continue
                        # if not, break this while loop to continue
                        # to next group
                        else:
                            self._result.result = resp.result
                            rospy.logerr("Changing control state of group %s\
                            failed, and resetting the controllers for group %s did not work", str(groups_to_retry), str(groups_to_reset))
                            self._as.set_aborted(self._result)
                            return

                # if FAILURE, skip this group
                if (resp.error_code.val ==
                        M3ControlStateErrorCodes.FAILURE):
                    continue

                # give some time for potentially resetted controllers
                # to converge
                rate.sleep()

                # publish the feedback
                self._as.publish_feedback(self._feedback)

            if success:
                self._result.result = self._current_state
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)
        else:
            self._result.result = M3ControlStates()
            if goal.strategy == 0:
                rospy.logerr("goal is invalid, strategy not given")
            else:
                rospy.logerr("goal is invalid, strategy is RETRY_N_TIMES\
                             but retries is null")
            self._as.set_aborted(self._result)

    def init_service_client(self):
        """
        Initialize service clients to communicate with the controller manager
        """
        service_name = self._control_name + \
            '/controller_manager/switch_controller'

        rospy.loginfo("Waiting for %s", service_name)
        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("%s did not show up. Giving up", service_name)
            return False
        self._cm_switch_client = rospy.ServiceProxy(service_name,
                                                    SwitchController)

        service_name = 'state_manager/change_state'
        if self._control_name != "":
            service_name = self._control_name + "_" + service_name

        rospy.loginfo("Waiting for %s", service_name)
        try:
            rospy.wait_for_service(service_name, 5.0)
        except rospy.ROSException:
            rospy.logerr("%s did not show up. Giving up", service_name)
            return False
        self._state_client = rospy.ServiceProxy(service_name,
                                                M3ControlStateChange)

        service_name = self._control_name + \
            '/controller_manager/list_controllers'
        rospy.loginfo("Waiting for %s", service_name)
        try:
            rospy.wait_for_service(service_name, 5.0)
        except rospy.ROSException:
            rospy.logerr("%s did not show up. Giving up", service_name)
            return False
        self._cm_list_client = rospy.ServiceProxy(service_name,
                                                  ListControllers)

        return True

    def init_controller_list(self):
        """
        Get all the controllers running and store their name per group
        """
        # get all the controllers
        try:
            resp = self._cm_list_client(ListControllersRequest())
        except rospy.ServiceException:
            rospy.logerr("Could not call list_controllers")
            return

        # loop on the controllers
        if resp:
            all_controllers = []
            for controller in resp.controller:
                cname_split = controller.name.split("_")
                if len(cname_split) > 1:
                    if cname_split[0] in ["torso", "zlift", "head", "base"]:
                        if cname_split[0] not in self._controller_list:
                            self._controller_list[cname_split[0]]=[]
                        self._controller_list[cname_split[0]].append(controller.name)
                        all_controllers.append(controller.name)
                    else:
                        if len(cname_split) > 2:
                            if cname_split[1] in ["arm", "hand"]:
                                
                                group_name = cname_split[0] + "_" +\
                                    cname_split[1]
                                if group_name not in self._controller_list:
                                    self._controller_list[group_name]=[]
                                self._controller_list[group_name].append(
                                    controller.name)
                                all_controllers.append(controller.name)
            self._controller_list['all'] = all_controllers
        print self._controller_list

    def reset_controllers(self, group_names):
        """
        reset all the controllers in the given group_names
        """
        if self.stop_controllers(group_names):
            rospy.sleep(1)
            if self.start_controllers(group_names):
                return True
        return False

    def start_controllers(self, group_list):
        """
        start all the controllers in the given group_names
        """
        ret = True
        controllers = []
        if isinstance(group_list, str):
            group_names = group_list.split(" ")
        else:
            group_names = group_list
        for group in group_names:
            if group in self._controller_list:
                controllers += self._controller_list[group]
            else:
                rospy.logerr("No controller found for this group")

        req = SwitchControllerRequest()
        req.start_controllers = controllers
        req.strictness = SwitchControllerRequest.BEST_EFFORT

        try:
            resp = self._cm_switch_client(req)
            if resp.ok is False:
                rospy.logerr("Failed to start controllers")
                ret = False
            else:
                rospy.loginfo("Started requested controllers")
        except rospy.ServiceException:
            rospy.logerr("Failed to call cm service")
            ret = False
        return ret

    def stop_controllers(self, group_list):
        """
        stop all the controllers in the given group_names
        """
        ret = True
        controllers = []
        if isinstance(group_list, str):
            group_names = group_list.split(" ")
        else:
            group_names = group_list
        for group in group_names:
            if group in self._controller_list:
                controllers += self._controller_list[group]
            else:
                rospy.logerr("No controller found for group %s", group)

        req = SwitchControllerRequest()
        req.stop_controllers = controllers
        req.strictness = SwitchControllerRequest.BEST_EFFORT

        try:
            resp = self._cm_switch_client(req)
            if resp.ok is False:
                rospy.logerr("Failed to stop controllers")
                ret = False
            else:
                rospy.loginfo("Stopped requested controllers")
        except rospy.ServiceException:
            rospy.logerr("Failed to call cm service")
            ret = False
        return ret

if __name__ == '__main__':
    rospy.init_node('meka_state_manager')
    MekaStateManager(rospy.get_name(), m3roscontrol_name="/meka_roscontrol")
    rospy.spin()
