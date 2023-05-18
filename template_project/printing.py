import csv

 with open('students.csv', 'w', newline='') as file:
     writer = csv.writer(file)
     
     writer.writerow(["Pose", "right_ankle", "right_knee", "right_hip_1", "right_hip_2", "right_hip_3", "left_hip_1", "left_hip_2", "left_hip_3", "left_knee", "left_ankle_1", "left_ankle_2", "left_ankle_3", "waist", "right_shoulder_1", "right_shoulder_2", "right_shoulder_3", "right_elbow", "right_wrist_1", "right_wrist_2", "right_wrist_3", "left_shoulder_1", "left_shoulder_2", "left_shoulder_3", "left_elbow", "left_wrist_1", "left_wrist_2", "left_wrist_3",])
     writer.writerow([1, "Ash Ketchum", "English"])
     writer.writerow([2, "Gary Oak", "Mathematics"])
     writer.writerow([3, "Brock Lesner", "Physics"])