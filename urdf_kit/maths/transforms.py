import numpy as np
from spatialmath import SE3
from xml.etree.ElementTree import Element # just for typehint

from . import floatList_from_vec3String, vec3String_from_floatList
from . import color_code

"""
This submodule deals with rigid-body transform.
In URDF, this is referred as <origin>.
"""

##################################
# getter/ passive operations
##################################
def get_origin(origin_elem: Element) -> SE3:
    """Read in a URDF <origin> element and return a SE3 object
    
    <origin> means "transforms in URDF".
    It can belong to ...
    * <joint>
    * <inertial>
    * <visual>
    * <collision>

    See also `write_origin`
    """
    if origin_elem is None:
        return SE3.Tx(0)
    else:
        assert origin_elem.tag == "origin"
        origin_elem_xyz = floatList_from_vec3String(origin_elem.get("xyz"))
        origin_elem_rpy = floatList_from_vec3String(origin_elem.get("rpy"))
        return SE3.Trans(origin_elem_xyz)@SE3.RPY(origin_elem_rpy, unit='rad', order='zyx')

def _get_axis_xyz(joint_elem: Element) -> np.ndarray:
    """
    return the normalized values [x,y,z] of the rotation axis/ axis of translation

    Do NOT call this function if this joint is fixed!

    This function will also handle the edge case where
    <axis> is not specified in the URDF.
    In which case, the default is (1,0,0).
    See https://wiki.ros.org/urdf/XML/joint
    """
    assert joint_elem.get("type") != "fixed"
    axisElem = joint_elem.find("axis")
    if axisElem is None:
        return np.array((1,0,0))
    else:
        axis = floatList_from_vec3String(axisElem.get("xyz"))
        axis = np.array(axis).reshape(3)
        return axis/np.linalg.norm(axis)
def get_X_JointChild(joint_elem: Element , joint_angle: float) -> SE3:
    """compute the SE3 of the child link w.r.t. its parent joint
    """
    if joint_elem.get("type") == "fixed":
        return SE3.Tx(0)
    elif joint_elem.get("type") in ("continuous","revolute"):
        return SE3.AngleAxis(theta=joint_angle, v=_get_axis_xyz(joint_elem), unit='rad')
    else: 
        raise NotImplementedError("Prismatic joint not supported yet.")

def get_X_ParentJoint(joint_elem: Element ) -> SE3:
    """compute the SE3 of the joint link w.r.t. the parent link

    Note: In the context of a fixed joint, we can also use this function
    to compute the SE3 of the child link w.r.t. the parent link, 
    think it like `get_X_ParentChild(joint_elem)`.
    
    """
    joint_origin_elem = joint_elem.find("origin")
    return get_origin(joint_origin_elem)

def get_X_CparentCchild(joint_elem: Element, parent_elem: Element, child_elem: Element) -> SE3:
    """ calculate the pose of the Center of mass-aligned frame of the child link w.r.t that of the parent link
    
    use cases:
    1. merging fixed joint
    2. dynamics algorithm: the home "position" constant (joint angle := 0)
       Sometimes you might want to invert the returned SE3.

    See also `edit_links.grab_elems_dict_by_joint_name`, 
    you might find it useful before calling this function.
    
    No checks on whether the given parent and child elements
    are consistent with the joint_elem.

    This API is a bit redundant but 
    it can avoid searching for the XML elements again.
    """
    X_ParentChild = get_X_ParentJoint(joint_elem)
    X_ParentCparent = get_origin(parent_elem.find("inertial/origin"))
    X_ChildCchild = get_origin(child_elem.find("inertial/origin"))
    X_CparentCchild = X_ParentCparent.inv()@X_ParentChild@X_ChildCchild
    return X_CparentCchild

####################################
# in-place modification(s)
####################################
def write_origin(origin_elem: Element , X: SE3) -> None:
    """ Write SE3 into the given <origin> XML element 
    
    See also `get_origin`    
    """
    # throw an error because otherwise user won't know he/she did sth wrong.
    if origin_elem.tag != "origin":
        raise ValueError(color_code['r']+"The xml element you pass is invalid! Expect an <origin> element!"+color_code['w'])
    rpy = X.rpy(order='zyx',unit='rad') # TODO how will it handle singularity???
    origin_elem.attrib['rpy'] = vec3String_from_floatList(rpy)
    origin_elem.attrib['xyz'] = vec3String_from_floatList(X.t)
