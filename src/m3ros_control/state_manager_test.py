#!/usr/bin/env python

# author Guillaume WALCK (2015)

import rospy

from m3meka_msgs.msg import M3ControlStates, M3ControlStateErrorCodes
from m3meka_msgs.srv import M3ControlStateChange, M3ControlStateChangeResponse


class MekaStateManagerTest(object):
    """
        Meka StateManager Test
    """
    def __init__(self, name, m3roscontrol_name):
        rospy.init_node('meka_state_manager_test')
        self._service_name = name
        self._cur_state = M3ControlStates.ESTOP
        self.service = rospy.Service(m3roscontrol_name + '/change_state',
                                     M3ControlStateChange, self.callback)
                                     
        self._pub = rospy.Publisher(m3roscontrol_name + '/state',
                                    M3ControlStates, queue_size = 1)
        rate = rospy.Rate(10) # 10hz                           
        while not rospy.is_shutdown():
            self.timeout_cb()
            rate.sleep()
                                    

    def timeout_cb(self):
      
        msg = M3ControlStates()
        msg.group_name.append("all")
        msg.state.append(self._cur_state)
        self._pub.publish(msg) 

    def callback(self, req):
        rospy.loginfo("received a request")
        print req
        resp = M3ControlStateChangeResponse()
        #resp.error_code.val=M3ControlStateErrorCodes.CONTROLLER_NOT_CONVERGED
        resp.error_code.val=M3ControlStateErrorCodes.SUCCESS
        
        if len(req.command.state) > 0:
            self._cur_state = req.command.state[0]
        
        return resp

if __name__ == '__main__':
    
    MekaStateManagerTest(rospy.get_name(),m3roscontrol_name="meka_roscontrol_state_manager")
