import csv

CHAIR_POSE = 3
WARRIOR_1 = 1
WARRIOR_2 = 3

with open('pose_descriptions.csv', 'w', newline='') as file:
     writer = csv.writer(file)
     
     writer.writerow(["Pose", "right_ankle_y", "right_ankle_x" ,"right_knee", 
                      "right_hip_x", "right_hip_y", "right_hip_z", 
                      "left_hip_z", "left_hip_y", "left_hip_x", 
                      "left_knee", "left_ankle_x", "left_ankle_y", "left_ankle_z", 
                      "waist_y","waist_z", "waist_x"
                      "right_shoulder_x", "right_shoulder_y", "right_shoulder_z", 
                      "right_elbow", 
                      "right_wrist_x", "right_wrist_y", "right_wrist_z", 
                      "left_shoulder_x", "left_shoulder_y", "left_shoulder_x", 
                      "left_elbow", 
                      "left_wrist_x", "left_wrist_y", "left_wrist_z",
                      "neck_1", "neck_2", "neck_3"])
     
     writer.writerow([CHAIR_POSE, 0.454621,-1.599603,         #"Pose", "right_ankle", "right_knee"
                      1.404628,-0.210719,-0.327871,           #"right_hip_1", "right_hip_2", "right_hip_3
                      0.217871,0.210719,-1.404628,            #"left_hip_1", "left_hip_2", "left_hip_3", 
                      1.599603,-0.508144,-0.008615,-0.098502, #"left_knee", "left_ankle_1", "left_ankle_2", "left_ankle_3",   
                      0.0,                                    #"waist"
                      -3.321765,-0.447239,0.059882,           #"right_shoulder_1", "right_shoulder_2", "right_shoulder_3", 
                      -0.036452,                              #"right_elbow", 
                      0.390400,-0.107298,-0.506217,             #"right_wrist_1", "right_wrist_2", "right_wrist_3", 
                      -3.299092,-0.266798,-0.112313,          #"left_shoulder_1", "left_shoulder_2", "left_shoulder_3", 
                      -0.106552,                              #"left_elbow", 
                      0.390400,-0.107298,-0.506217,           # "left_wrist_1", "left_wrist_2", "left_wrist_3",
                      -0.787140,0.192233,0.269256])           #"neck_1", "neck_2", "neck_3"
    #  writer.writerow([2, "Gary Oak", "Mathematics"])
    #  writer.writerow([3, "Brock Lesner", "Physics"])