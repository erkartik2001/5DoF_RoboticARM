#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import pybullet as p
import math
import time
import serial
import tf


#home and drop position
home = [10,10,100,90,90]
drop = [90,90,60,90,180]


#initializing pybullet essentials
p.connect(p.DIRECT)
urdf_path = "/home/yujiro/catkin_ws/src/urdf_tut/urdf/5dofARM.urdf"
robot = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=1)
end_effector_index = 4

max_iterations = 1000
tolerance = 1e-10
print("\n----------Program Starts-------------")


#initializing serial essentials
port = "/dev/ttyACM0"
baud_rate = 9600

ser = serial.Serial(port=port,baudrate=baud_rate)
time.sleep(4)

def sendSerial(angles):
    payload = " ".join(map(str,angles))
    ser.write(payload.encode())


#initializing jointstate publisher variables
jointPub = rospy.Publisher("/joint_states",JointState,queue_size=1)


def publishJoint(angles):
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = ["link5_joint","link4_joint","link3_joint","link2_joint","link1_joint"]
    joint_state_msg.position = angles
    joint_state_msg.velocity = [0.0,0.0,0.0,0.0,0.0]

    jointPub.publish(joint_state_msg)



def calculate_ik(target_position, target_orientation):
    joint_positions = p.calculateInverseKinematics(
    robot,
    end_effector_index,
    target_position,
    targetOrientation=p.getQuaternionFromEuler(target_orientation),
    maxNumIterations=max_iterations,
    residualThreshold=tolerance,
    )

    joint_degrees = [math.degrees(angle) for angle in joint_positions]
    
    valid_angles = [-angle if angle<0 else angle for angle in joint_degrees]

    return valid_angles


def main():
    rospy.init_node("HardwareController")
    pub = rospy.Publisher("/generateTransform",String,queue_size=1)

    pub.publish("dont")
    sendSerial(home)

    time.sleep(5)

    publishJoint(angles=home)

    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        pub.publish("generate")

        try:
            trans,rot = listener.lookupTransform("/base_link","/obj_frame",rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
            
        pub.publish("stop")
        angles = calculate_ik(target_position=trans, target_orientation=[0,0,0])

        #Reach to object position
        sendSerial(angles=angles)
        time.sleep(5)
        publishJoint(angles=angles)


        #Reach to drop position
        sendSerial(angles=drop)
        time.sleep(5)
        publishJoint(angles=drop)


        #Reach to home position
        sendSerial(angles=home)
        time.sleep(5)
        publishJoint(angles=home)

        pub.publish("generate")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(str(e))

    finally:
        try:
            ser.close()
        except Exception as e:
            print("Error closing the serial port:", str(e))
        try:
            rospy.signal_shutdown("Closing the ROS node")
        except Exception as e:
            print("Error shutting down the ROS node:", str(e))

        
    
            


