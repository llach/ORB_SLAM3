import cv2
import json
import pickle
import pathlib

IMU_TOPIC = "/camera/imu"
RGB_TOPIC = "/camera/color/image_raw/compressed"

path = str(pathlib.Path(__file__).parent.parent) + "/data"
pklfile = f"{path}/light_stack_09_18_10.pkl"

with open(pklfile, "rb") as f:
    data = pickle.load(f)

with open(f"{path}/imu.json", "w") as f:
    f.write(json.dumps({"samples": data[IMU_TOPIC]}, indent=2))

vts = []
height,width,layers = data[RGB_TOPIC][0][1].shape
video = cv2.VideoWriter(f"{path}/video.avi", cv2.VideoWriter_fourcc(*'DIVX'), 30, (width,height))
print(height, width)

for d in data[RGB_TOPIC]:
    vts.append(d[0])
    video.write(cv2.cvtColor(d[1], cv2.COLOR_RGB2BGR))

with open(f"{path}/video.json", "w") as f:
    f.write(json.dumps({"samples": vts}, indent=2))