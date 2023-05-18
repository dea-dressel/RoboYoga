import csv

CHAIR_POSE = 3
WARRIOR_1 = 1
WARRIOR_2 = 3

with open('pose_descriptions.csv', 'w', newline='') as file:
     writer = csv.writer(file)
     
     writer.writerow(["Pose", "right_ankle", "right_knee", 
                      "right_hip_1", "right_hip_2", "right_hip_3", 
                      "left_hip_1", "left_hip_2", "left_hip_3", 
                      "left_knee", "left_ankle_1", "left_ankle_2", "left_ankle_3", 
                      "waist",
                      "right_shoulder_1", "right_shoulder_2", "right_shoulder_3", 
                      "right_elbow", 
                      "right_wrist_1", "right_wrist_2", "right_wrist_3", 
                      "left_shoulder_1", "left_shoulder_2", "left_shoulder_3", 
                      "left_elbow", 
                      "left_wrist_1", "left_wrist_2", "left_wrist_3",
                      "neck_1", "neck_2", "neck_3"])
     
     writer.writerow([CHAIR_POSE, 0.454621,-1.599603,
                      1.473881,-0.171505,-0.235412,
                      0.327871,0.210719,-1.404628,
                      1.601027,-0.508144,-0.008615,-0.098502,
                      0.336433,
                      -3.321765,-0.447239,0.059882,
                      -0.036452,
                      0.422319,0.770947,1.522959,
                      -3.321765,-0.447239,0.059882,
                    #   -3.299092,-0.266798,-0.112313,
                      -0.106552,
                      0.390400,-0.107298,-0.506217,
                      -0.787140,0.192233,0.269256])
    #  writer.writerow([2, "Gary Oak", "Mathematics"])
    #  writer.writerow([3, "Brock Lesner", "Physics"])