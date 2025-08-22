#!/usr/bin/env python3
"""
servo_controller_node.py
Author: Treggon Owens for W8less.io
Description: Python ROS node to send and receive ServoArray messages
from the Arduino. Includes error handling if Arduino is disconnected.
"""

import rospy
from servo_controller_msgs.msg import ServoArray

class ServoController:
    def __init__(self):
        rospy.init_node('servo_controller_node', anonymous=True)
        self.servo_angles = [90]*16
        self.servo_enabled = [True]*16
        self.pwm_input_1 = 0
        self.pwm_input_2 = 0
        try:
            self.pub = rospy.Publisher('/servo_cmd', ServoArray, queue_size=10)
            self.sub = rospy.Subscriber('/servo_state', ServoArray, self.state_callback)
        except rospy.ROSException as e:
            rospy.logwarn(f"ROS serial not connected: {e}")
            self.pub = None
            self.sub = None

    def state_callback(self,msg):
        try:
            self.servo_angles = list(msg.angles)
            self.servo_enabled = list(msg.enabled)
            self.pwm_input_1 = msg.pwm_input_1
            self.pwm_input_2 = msg.pwm_input_2
            rospy.loginfo(f"PWM1: {self.pwm_input_1} us, PWM2: {self.pwm_input_2} us")
        except Exception as e:
            rospy.logwarn(f"Error reading servo state: {e}")

    def send_command(self,angles=None,enabled=None):
        if not self.pub:
            rospy.logwarn("Publisher not initialized. Cannot send command.")
            return
        try:
            msg = ServoArray()
            msg.angles = angles if angles else self.servo_angles
            msg.enabled = enabled if enabled else self.servo_enabled
            msg.pwm_input_1 = 0
            msg.pwm_input_2 = 0
            self.pub.publish(msg)
        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to publish command: {e}")

    def run_example(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                for angle in range(0,181,10):
                    self.servo_angles[0]=angle
                    self.send_command()
                    rate.sleep()
                for angle in range(180,-1,-10):
                    self.servo_angles[0]=angle
                    self.send_command()
                    rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("ROS Interrupt, shutting down.")
                break
            except Exception as e:
                rospy.logwarn(f"Error in loop: {e}")
                rate.sleep()

if __name__=="__main__":
    try:
        controller = ServoController()
        controller.run_example()
    except rospy.ROSInterruptException:
        pass
