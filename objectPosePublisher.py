#!/usr/bin/python3

import rospy
import time
import cv2
import tf
import numpy as np
from std_msgs.msg import String
import torch
import urllib.request
import matplotlib.pyplot as plt



#initializing midas variables
model_type = "DPT_Hybrid"
midas = torch.hub.load("/home/yujiro/Downloads/isl-org-MiDaS-v3_1-2-gbdc4ed6/isl-org-MiDaS-bdc4ed6/",source="local",model=model_type,weights="/home/yujiro/Documents/major_project/major2/modelMidas/dpt_hybrid_384/archive/data.pkl")

device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval()

midas_transforms = torch.hub.load("intel-isl/MiDaS","transforms")

if model_type=="DPT_Large" or model_type=="DPT_Hybrid":
  transform = midas_transforms.dpt_transform

else:
  transform = midas_transforms.small_transform


#urdf path to load into rosparam
urdf_path = "/home/yujiro/catkin_ws/src/urdf_tut/urdf/5dofARM.urdf"


def getObjPose():
    cap = cv2.VideoCapture(2)
    ret, image = cap.read()
    cap.release()

    cv2.imwrite("captured.jpg",image)
    # cv2.imshow("img",image)


    #Depth Estimation part
    input_batch = transform(image).to(device)
    with torch.no_grad():
        prediction = midas(input_batch)

        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size = image.shape[:2],
            mode="bicubic",
            align_corners = False,
        ).squeeze()

        output = prediction.cpu().numpy()

    
    #QR code detection part
    detector = cv2.QRCodeDetector()
    success, qr_codes, points, _ = detector.detectAndDecodeMulti(image)

    if success and len(qr_codes) > 0:
        num_qr_codes = len(qr_codes)
        print(f"Number of QR codes detected: {num_qr_codes}")

        for i in range(num_qr_codes):
            # Extract relevant information from points[i]
            x1 = int(points[i][0][0])
            y1 = int(points[i][0][1])
            x2 = int(points[i][2][0])
            y2 = int(points[i][2][1])

            x_center = (x2 - x1) // 2 + x1
            y_center = (y2 - y1) // 2 + y1

            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 4)
            cv2.circle(image, (x_center, y_center), 4, (0, 0, 255), 10)

        # Display count on the image
        cv2.putText(image, f"Package Detected: {num_qr_codes}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        # Display if no QR codes are detected
        cv2.putText(image, "No Package Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("image", image)

    z = output[y_center,x_center]
    
    #converting camera coordinates to real world coordinates using intrinsic parameters
    X = z *((x_center-337.16370289)/701.51605235)
    Y = z *((y_center-199.22182115)/938.52525887)
    Z = z

    return(X,Y,Z)
    

def publishTransform(obj_pose, fname):
    br = tf.TransformBroadcaster()
    br.sendTransform(translation=obj_pose,
                    rotation=tf.transformations.quaternion_from_euler(0,0,0),time=rospy.Time.now(),child=fname,parent="link_5")
    

def getTransformCallback(message):
    data = message.data
    pose = getObjPose()
    if data == "generate":
        if pose:
            publishTransform(obj_pose=pose,fname="/obj_frame")

    rospy.loginfo("getting data")


def main():
    rospy.init_node("objectPosePublisher")
    with open(urdf_path,"r") as urdf_file:
        urdf_string = urdf_file.read()

    rospy.set_param("robot_description",urdf_string)    

    rospy.Subscriber("/generateTransform",String,getTransformCallback)

    rospy.loginfo("Working fine")

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    
    except Exception as e:
        print(str(e))
