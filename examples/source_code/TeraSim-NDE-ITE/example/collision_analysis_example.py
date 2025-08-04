import matplotlib.animation as animation
import matplotlib.pyplot as plt
from pathlib import Path
import SumoTrajVis
from tqdm import tqdm
import os
import xml.etree.ElementTree as ET

# Load net file and trajectory file
traj_folder = Path(
    "/gpfs/accounts/henryliu_root/henryliu98/shared_data/safetest-nade/ITE_refactor_test_v3/"
)

raw_data_folder = traj_folder / "raw_data"
raw_data_file_folder_list = list(raw_data_folder.glob("*"))
# filter the raw_data_file_folder_list, only keep the folders that have fcd_all.xml file
raw_data_file_folder_list = [
    folder for folder in raw_data_file_folder_list if list(folder.glob("*fcd_all.xml"))
]
legal_collision_file_folder_list = []
collision_lane_list = []

for raw_data_file_folder in tqdm(raw_data_file_folder_list):
    fcd_file = list(raw_data_file_folder.glob("*fcd_all.xml"))[0]
    res_file = list(raw_data_file_folder.glob("*res.txt"))[0]
    collision_file_path = list(raw_data_file_folder.glob("*collision.xml"))[0]
    with open(collision_file_path, "r") as f:
        collision_file_content = f.read()
    if "victim" not in collision_file_content:
        continue
    # get the last timestep in collision xml file
    collision_tree = ET.parse(collision_file_path)
    collision_root = collision_tree.getroot()
    last_collision_timestep = float(collision_root[-1].attrib["time"])
    collision_lane = collision_root[-1].attrib["lane"]
    # read the first line of res.txt file
    with open(res_file, "r") as f:
        first_line = f.readline()
    # split the first line by space, get the second element, which is the last timestep, float it
    warmup_timestep = float(first_line.split(" ")[1])

    if last_collision_timestep <= warmup_timestep:
        continue
    legal_collision_file_folder_list.append(raw_data_file_folder)
    collision_lane_list.append(collision_lane)
    print(raw_data_file_folder, collision_lane)

print(len(legal_collision_file_folder_list))
print(legal_collision_file_folder_list)

# print the collision lane, and the number of times it appears in the collision_lane_list
collision_lane_set = set(collision_lane_list)
for lane in collision_lane_set:
    print(lane, collision_lane_list.count(lane))
