#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState , ModelStates

rospy.init_node("running")

msg2=ModelStates()
ss =1
def callback(sub_msg : ModelStates):
    global msg2
    msg2 = sub_msg

def publishh(vel):
    msg = ModelState()
    for i in range(0,5):
        msg.model_name = msg2.name[i+1]
        msg.pose=msg2.pose[i+1]
        msg.twist.linear.x = vel
        mod_st.publish(msg)
        print("msg is published")

rospy.Subscriber("/gazebo/model_states",ModelStates,callback) 
mod_st = rospy.Publisher("/gazebo/set_model_state", ModelState,queue_size=10)

while not rospy.is_shutdown():
        vel = float(input())
        publishh(vel)
        
        
        