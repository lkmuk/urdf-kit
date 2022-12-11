import numpy as np
from spatialmath import SE3

from . import floatList_from_vec3String, vec3String_from_floatList

def get_X_JointChild(joint_elem, joint_angle: float) -> SE3:
    joint_ax_elem = joint_elem.find("axis")
    joint_ax_xyz = floatList_from_vec3String(joint_ax_elem.get("xyz"))
    return SE3.AngleAxis(theta=joint_angle, v=joint_ax_xyz, unit='rad')

def get_X_ParentJoint(joint_elem) -> SE3:
    joint_origin_elem = joint_elem.find("origin")
    joint_origin_elem_xyz = floatList_from_vec3String(joint_origin_elem.get("xyz"))
    joint_origin_elem_rpy = floatList_from_vec3String(joint_origin_elem.get("rpy"))
    return SE3.Trans(joint_origin_elem_xyz)@SE3.RPY(joint_origin_elem_rpy, unit='rad', order='zyx')

def write_origin(origin_elem, X: SE3) -> None:
    """ Write SE3 into the given <origin> XML element 
    
    An <origin> element can be found in so many places in a URDF:
    1. joint/origin
    2. link/collision, 
    3. link/visual and 
    4. link/inertial/origin
    so this function does not "belong" to `edit_joints` or `edit_links`
    """
    rpy = X.rpy(order='zyx',unit='rad') # TODO how will it handle singularity???
    origin_elem.attrib['rpy'] = vec3String_from_floatList(rpy)
    origin_elem.attrib['xyz'] = vec3String_from_floatList(X.t)
