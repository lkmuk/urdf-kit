from urdf_kit.graph.simplify import fix_revolute_joint
from urdf_kit.misc import format_then_write #, print_urdf
from math import pi

import xml.etree.ElementTree as ET
from pathlib import Path
data_dir = (Path(__file__).resolve().parent/".."/".."/"data"/"kuka_iiwa").resolve()

src_urdf_path = data_dir/"model.urdf"
new_urdf_path = data_dir/"fixed_joint_4.urdf" # should be the same dir

urdf_root = ET.parse(src_urdf_path).getroot()
# print_urdf(urdf_root)

joint_elem = None
for candidate in  urdf_root.findall("joint"):
    if candidate.get("name") == "lbr_iiwa_joint_4":
        joint_elem = candidate
        break
assert joint_elem is not None
fix_revolute_joint(
    joint_elem, 
    #joint_angle=-pi # assertion caught
    joint_angle=-pi/6
)

format_then_write(urdf_root, new_urdf_path)

import pybullet as p
p.connect(p.GUI)
_ = p.loadURDF(str(new_urdf_path))
if __name__ == "__main__":
    # just to block the control flow.
    prompt = "Are you happy? Type yes to exit: "
    ans = ""
    while ans not in ('y','Y'):
        ans = input(prompt)
else:
    p.disconnect()


