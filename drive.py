#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

class Avoid:
    def __init__(self):
        self.b_ultrasonic = False
        
        self.ultrasonic_subscriber = rospy.Subscriber("ultrasonic", Int32MultiArray, self.ultrasonic_CB)
        self.motor_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.data = xycar_motor()

        self.speed = 0 
        self.angle = 0


    def ultrasonic_CB(self, msg):
        if not self.b_ultrasonic:
            rospy.loginfo("Ultrasonic data received")
            self.b_ultrasonic = True

        self.ultra_data = self.swap_list_to_dic(msg.data)
        self.data_anlyze() 

    
    def data_anlyze(self):
        dif = self.ultra_data["DR"] - self.ultra_data["DL"]
        self.data.angle = dif//6
        self.data.speed = 1000
        
        if self.data.angle < 0 :
            rospy.loginfo(f"left : {self.data.angle}")
        elif int(self.data.angle) == 0 :
            rospy.loginfo(f"stright : {self.data.angle}")
        else :
            rospy.loginfo(f"right : {self.data.angle}")        

        self.motor_publisher.publish(self.data)
    
    def swap_list_to_dic(self, data):
        # "DL과 DR의 차이 존재 -> DL에 offset 1 추가"
        return {"L": data[0], "DL" : data[1]+1, "C":data[2]
        ,"DR": data[3], "R" : data[4]}

  
if __name__=="__main__":
    AV = Avoid()
    rospy.init_node('driver')
    rospy.spin()

