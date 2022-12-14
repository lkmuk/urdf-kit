from xml.etree import ElementTree as ET

from . import get_X_JointChild, get_X_ParentJoint, write_origin
from . import remove_subelement_by_tag

def fix_revolute_joint(joint_elem: ET.Element, joint_angle: float, verbose=False) -> None:
    """freeze the given revolute joint
    
    This can be the first step for reducing your URDF.
    After fixing a joint, you might also want 
    to merge the link upstream. 
    See `merge_fixed_joints`.
    """
    assert isinstance(joint_elem, ET.Element), "maybe the desired joint_elem is not found?"
    assert isinstance(joint_angle, float)
    name = joint_elem.get("name")
    if verbose:
        print("  fixing joint", name, f"to {joint_angle:.3f} radian")
    assert joint_elem.get("type") in ("revolute","continuous"), f"joint '{name}' is of type {joint_elem.get('type')}"
    
    if joint_elem.get("type") == "revolute":
        lb = float(joint_elem.find("limit").get("lower")) 
        ub = float(joint_elem.find("limit").get("upper"))
        assert lb <= joint_angle <= ub, f"This joint [{joint_elem.get('name')}]'s angle should be within {lb} and {ub} but you gave {joint_angle}!!!"

    X_ParentChild = get_X_ParentJoint(joint_elem) * get_X_JointChild(joint_elem, joint_angle)

    # time to write the necessary changes ...
    # 1. update robot/joint/origin
    write_origin(joint_elem.find("origin"), X_ParentChild)
    # 2. update robot/joint/@type
    joint_elem.attrib["type"] = 'fixed'
    # 3. remove irrelevant subelements
    for tag in ("axis", "mimic", "limit", "dynamics", "joint_properties"):
        remove_subelement_by_tag(joint_elem, tag)
    

"""merge_fixed_joints
such operation is 
implemented in the `urdf_kit.graph.tree.kinematic_tree` instead.
"""