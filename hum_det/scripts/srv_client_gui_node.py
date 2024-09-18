#!/usr/bin/env python3

import rospy
from hum_det.srv import *

DET_MODE_SWITCH_SRV = "det_mode_switch"


def call_det_mode_switch(is_on):
    rospy.wait_for_service(DET_MODE_SWITCH_SRV)
    try:
        det_mode_switch = rospy.ServiceProxy(DET_MODE_SWITCH_SRV, DetModeSwitch)
        res = det_mode_switch(is_on)
        return res.code
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
	
    print("code = %s" % (call_det_mode_switch(True)))
    # print("code = %s" % (call_det_mode_switch(False)))
