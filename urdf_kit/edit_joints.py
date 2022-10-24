from __future__ import annotations
from xml.etree import ElementTree as ET

__all__ = ("joint_entry_T", "grab_all_joints")

# the purposes of this container class
#   1. facilitate printing of joint entries
#   2. to facilitate programmatic access
import dataclasses as dc 
# named tuple does not allow mutability so a container class instead
@dc.dataclass(eq=True)
class joint_entry_T:
    joint_ptr: xml.etree.ElementTree.ElementTree
    child: str
    parent: str


def grab_all_joints(urdf_root: ET.ElementTree) -> list[joint_entry_T]:
    out = []
    for joint_entry in urdf_root.findall("joint"):
        out.append(joint_entry_T(
            joint_ptr=joint_entry, 
            child=joint_entry.find("child").get("link"),
            parent=joint_entry.find("parent").get("link")
        ))
    return out


# TODO zero-out: 
# set a threshold, e.g. 1e-13, and replace those values with 0,0,0
# concern: the pose xyz rpy

def grab_expected_joints_handle(urdf_root: ET.ElementTree, joint_names: list[str]) -> list[ET.ElementTree]:
    """ grab all the joint xml handle corresponding to the joint_names list (sorted in the same order)
    You might then want to iterate through the resultant list, for your own needs.

    I assume joint_names are all unique!!!

    raise 
    -----------
    ValueError if not all expected joints are found!
    """
    joint_names_set = set(joint_names)
    assert len(joint_names_set) == len(joint_names), "You have duplicated joints in joint_names, which should not happen (would result in incorrect behavior)!"

    out = [None]*len(joint_names)
    # iterating through the "supply" is more efficient
    # for this one-to-one matching.
    # worst-case: iterate through all joints in the URDF
    for joint_entry in urdf_root.findall("joint"):
        for ind, wanted_name in enumerate(joint_names):
            if joint_entry.attrib["name"] == wanted_name:
                out[ind] = joint_entry
                break # go search the next "supply"
    
    # not_found_indices = [] # it doesn't matter in Python
    not_found_name_list = [] 
    for ind in range(len(joint_names)):
        if out[ind] is None:
            not_found_name_list.append(joint_names[ind])
    if len(not_found_name_list) > 0:
        raise ValueError("These joints are not found: " + ' ,'.join(not_found_name_list))
    else:
        return out

# if __name__=="__main__":
#     from pathlib import Path
#     this_dir = Path(__file__).resolve().parent
#     test_file = this_dir/".."/"tests"/"dummy.urdf"
#     tree = ET.parse(test_file)
#     root = tree.getroot()
#     res = grab_all_joints(root)
#     for entry in res:
#         print(entry)