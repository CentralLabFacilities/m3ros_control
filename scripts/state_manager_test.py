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
        self._cur_state = {}
        self._cur_state["left_arm"] = M3ControlStates.ESTOP
        self._cur_state["head"] = M3ControlStates.ESTOP
        
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
        msg.group_name.append("left_arm")
        msg.group_name.append("head")
        msg.state.append(self._cur_state["left_arm"])
        msg.state.append(self._cur_state["head"])
        self._pub.publish(msg) 

    def callback(self, req):
        rospy.loginfo("received a request")
        print req
        resp = M3ControlStateChangeResponse()
        #resp.error_code.val=M3ControlStateErrorCodes.CONTROLLER_NOT_CONVERGED
        resp.error_code.val=M3ControlStateErrorCodes.SUCCESS
        #resp.error_code.val=M3ControlStateErrorCodes.FAILURE
        
        for group_name, state in zip(req.command.group_name, req.command.state):
            if group_name in self._cur_state:
                
                resp.result.group_name.append(group_name)
                if "head" in group_name:
                    self._cur_state[group_name] = 0
                    resp.result.state.append(0)
                    resp.error_code.val=M3ControlStateErrorCodes.CONTROLLER_NOT_CONVERGED
                else:
                    self._cur_state[group_name] = state
                    resp.result.state.append(state)
            else:
                rospy.logerr("group %s does not exist", group_name)            
        print resp
        return resp

if __name__ == '__main__':
    
    MekaStateManagerTest(rospy.get_name(),m3roscontrol_name="meka_roscontrol_state_manager")
