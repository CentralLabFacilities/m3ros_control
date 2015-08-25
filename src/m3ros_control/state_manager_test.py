#!/usr/bin/env python

import rospy

from m3meka_msgs.msg import M3ControlStates, M3ControlStateErrorCodes
from m3meka_msgs.srv import M3ControlStateChange, M3ControlStateChangeResponse


class MekaStateManagerTest(object):
    """
        Meka StateManager Test
    """
    def __init__(self, name, m3roscontrol_name):
        self._service_name = name
        self.service = rospy.Service(m3roscontrol_name + '/change_state',
                                     M3ControlStateChange, self.callback)

    def callback(self, req):
        rospy.loginfo("received a request")
        print req
        resp = M3ControlStateChangeResponse()
        resp.error_code.val=M3ControlStateErrorCodes.CONTROLLER_NOT_CONVERGED
        return resp

if __name__ == '__main__':
    rospy.init_node('meka_state_manager_test')
    MekaStateManagerTest(rospy.get_name(),m3roscontrol_name="state_manager")
    rospy.spin()
