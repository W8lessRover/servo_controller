#!/usr/bin/env python3
"""
servo_controller_gui.py
Author: Treggon Owens
Description: Tkinter GUI to control 16 PCA9685 servos, toggle enable/disable,
display PWM inputs, includes error handling if Arduino is disconnected.
"""

import rospy
from servo_controller_msgs.msg import ServoArray
import tkinter as tk
from tkinter import ttk

class ServoControllerGUI:
    def __init__(self):
        rospy.init_node('servo_controller_gui_node', anonymous=True)
        self.servo_angles = [90]*16
        self.servo_enabled = [True]*16
        self.pwm_input_1 = 0
        self.pwm_input_2 = 0
        try:
            self.pub = rospy.Publisher('/servo_cmd', ServoArray, queue_size=1)
            self.sub = rospy.Subscriber('/servo_state', ServoArray, self.state_callback)
        except rospy.ROSException as e:
            rospy.logwarn(f"ROS serial not connected: {e}")
            self.pub = None
            self.sub = None

        self.root = tk.Tk()
        self.root.title("Servo Controller GUI")
        self.sliders = []
        self.vars = []

        for i in range(16):
            frame = ttk.Frame(self.root)
            frame.grid(row=i//4,column=i%4,padx=5,pady=5,sticky="n")
            label = ttk.Label(frame,text=f"Servo {i}")
            label.pack()
            var = tk.IntVar(value=1)
            chk = ttk.Checkbutton(frame,text="Enable",variable=var,command=self.update_servo_enabled)
            chk.pack()
            self.vars.append(var)
            slider = tk.Scale(frame,from_=0,to=180,orient=tk.HORIZONTAL,
                              command=lambda val,ch=i:self.update_servo_angle(ch,int(val)))
            slider.set(90)
            slider.pack()
            self.sliders.append(slider)

        self.pwm1_label = ttk.Label(self.root,text="PWM1: 0 us")
        self.pwm1_label.grid(row=4,column=0,columnspan=2)
        self.pwm2_label = ttk.Label(self.root,text="PWM2: 0 us")
        self.pwm2_label.grid(row=4,column=2,columnspan=2)

        self.update_gui()

    def state_callback(self,msg):
        try:
            self.servo_angles = list(msg.angles)
            self.servo_enabled = list(msg.enabled)
            self.pwm_input_1 = msg.pwm_input_1
            self.pwm_input_2 = msg.pwm_input_2
            for i in range(16):
                self.sliders[i].set(self.servo_angles[i])
                self.vars[i].set(1 if self.servo_enabled[i] else 0)
        except Exception as e:
            rospy.logwarn(f"Error reading servo state: {e}")

    def update_servo_angle(self,channel,angle):
        self.servo_angles[channel]=angle
        self.send_command()

    def update_servo_enabled(self):
        for i in range(16):
            self.servo_enabled[i] = bool(self.vars[i].get())
        self.send_command()

    def send_command(self):
        if not self.pub:
            rospy.logwarn("Publisher not initialized. Cannot send command.")
            return
        try:
            msg = ServoArray()
            msg.angles = self.servo_angles
            msg.enabled = self.servo_enabled
            msg.pwm_input_1 = 0
            msg.pwm_input_2 = 0
            self.pub.publish(msg)
        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to publish command: {e}")

    def update_gui(self):
        self.pwm1_label.config(text=f"PWM1: {self.pwm_input_1} us")
        self.pwm2_label.config(text=f"PWM2: {self.pwm_input_2} us")
        self.root.after(100,self.update_gui)

    def run(self):
        self.root.mainloop()

if __name__=="__main__":
    try:
        gui = ServoControllerGUI()
        gui.run()
    except rospy.ROSInterruptException:
        pass
