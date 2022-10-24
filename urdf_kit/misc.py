from xml.etree import ElementTree as ET
from .edit_joints import grab_all_joints

def format_then_write(urdf_root: ET.ElementTree, fpath: str):
    # self.tree.write(fpath)  # no autoformat
    ET.indent(urdf_root) # beautify
    with open(fpath, "wb") as f: # make no mistake it's 'wb' not 'w'
        f.write(ET.tostring(urdf_root) ) # requires Python 3.9+




def print_urdf(urdf_root: ET.ElementTree):
    print("-"*15, " link(s) ", "-"*15)
    for link_entry in urdf_root.findall("link"):
        print(link_entry.get("name"))

    print("-"*15, " joint(s) ", "-"*15)
    for joint_entry in grab_all_joints(urdf_root):
        print(joint_entry)