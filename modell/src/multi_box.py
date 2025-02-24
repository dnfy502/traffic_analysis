#!/usr/bin/env python3
import rospy
import subprocess
from gazebo_msgs.msg import ModelState, ModelStates
import numpy as np
from access_db import filtered_data

# Global Variables
msg2 = ModelStates()  # Ensure msg2 is defined globally
position_data = {}    # Store position values dynamically
carss=[]
def spawn_urdf(name, x, y, z):
    """ Spawns a URDF model in Gazebo """
    command = f"rosrun gazebo_ros spawn_model -urdf -model {name} -x {x} -y {y} -z {z} -file ~/krackhack/src/modell/src/robot.urdf"
    subprocess.call(command, shell=True)

def publishh(x,y,z, track_id):
    """ Publishes velocity updates for a model """
    global msg2
    msg = ModelState()
    msg.model_name = f"car_{track_id}"

    # Ensure we have the latest position data
    # if len(msg2.pose) > track_id:
    #     msg.pose = msg2.pose[track_id]

    msg.pose.position.x = z
    msg.pose.position.y = -(x+(np.e**(0.001*x)))
    msg.pose.position.z= 0

    mod_st.publish(msg)
    print(f"Published velocity: car_{track_id}, x={z}, y={ -(x+(np.e**(0.001*x)))}")

def callback(sub_msg: ModelStates):
    """ Callback function for Gazebo model state updates """
    global msg2
    msg2 = sub_msg

if __name__ == "__main__":
    rospy.init_node("spawn_boxes")

    # ROS Subscriber and Publisher
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    mod_st = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)

    # Process filtered data
    for doc in filtered_data:
        track_id = doc["track id"]
        
        if track_id not in carss:
            carss.append(track_id)
            # Store initial positions
            # position_data[f"x_{track_id}"] = doc["x"] / 2
            # position_data[f"y_{track_id}"] = doc["y"] / 2
            # position_data[f"z_{track_id}"] = doc["z"] / 2

            spawn_urdf(f"car_{track_id}", doc["z"] / 2, -(doc["x"] / 2 + (np.e**(0.001 * doc["x"] / 2))),0)

        else:
            # Store next positions
            position_data[f"nx_{track_id}"] = doc["x"] / 2
            position_data[f"ny_{track_id}"] = doc["y"] / 2
            position_data[f"nz_{track_id}"] = doc["z"] / 2

            # # Compute velocity
            # fr = 1
            # vel_x = (position_data[f"nz_{track_id}"] - position_data[f"z_{track_id}"]) / fr
            # vel_y = (position_data[f"nx_{track_id}"] - position_data[f"x_{track_id}"]) / fr

            publishh(position_data[f"nx_{track_id}"] , position_data[f"ny_{track_id}"],position_data[f"nz_{track_id}"], track_id)
            position_data[f"x_{track_id}"] = position_data[f"nx_{track_id}"]
            position_data[f"y_{track_id}"] =position_data[f"ny_{track_id}"]
            position_data[f"z_{track_id}"] = position_data[f"nz_{track_id}"]
            # rospy.sleep(1)
    # for id in carss:
    #     publishh(0,0,id)

    rospy.loginfo("All boxes spawned!")
    rospy.spin()  # Keeps the node running
