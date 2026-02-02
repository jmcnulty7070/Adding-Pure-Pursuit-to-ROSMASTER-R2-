#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
twist_mux_locks_pub.py

ROS Melodic compatible.
Publishes /e_stop_lock and /safety_lock continuously so twist_mux never
"failsafe-stops" due to missing lock publishers.

Default behavior: publish FALSE (unlocked).

You can still override at any time:
  rostopic pub -r 10 /e_stop_lock std_msgs/Bool "data: true"
  rostopic pub -r 10 /safety_lock std_msgs/Bool "data: true"
"""

import rospy
from std_msgs.msg import Bool


def main():
    rospy.init_node("twist_mux_locks_pub", anonymous=False)

    e_stop_default = rospy.get_param("~e_stop_default", False)
    safety_default = rospy.get_param("~safety_default", False)
    rate_hz = float(rospy.get_param("~rate_hz", 10.0))

    pub_e = rospy.Publisher("/e_stop_lock", Bool, queue_size=1, latch=True)
    pub_s = rospy.Publisher("/safety_lock", Bool, queue_size=1, latch=True)

    msg_e = Bool(data=bool(e_stop_default))
    msg_s = Bool(data=bool(safety_default))

    # Latch once immediately, then keep publishing.
    pub_e.publish(msg_e)
    pub_s.publish(msg_s)

    rate = rospy.Rate(rate_hz)
    rospy.loginfo("twist_mux_locks_pub: publishing /e_stop_lock=%s /safety_lock=%s at %.1f Hz",
                  msg_e.data, msg_s.data, rate_hz)

    while not rospy.is_shutdown():
        pub_e.publish(msg_e)
        pub_s.publish(msg_s)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
