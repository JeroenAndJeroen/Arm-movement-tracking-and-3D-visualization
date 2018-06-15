#!/usr/bin/env python

import rospy

def handle_zCorrection(req):
    print "Returning %s"%(req.zCorrection)
    return req.zCorrection

def zCorrection_server():
    rospy.init_node('zCorrection_server')
    s = rospy.Service('zCorrection', zCorrection, handle_zCorrection)
    print "Ready to get z orientation."
    rospy.spin()

if __name__ == "__main__":
    zCorrection_server()
