import pytest
from urdf_kit.maths import get_X_JointChild, get_X_ParentJoint
from urdf_kit.maths import write_origin
from urdf_kit.misc import floatList_from_vec3String

import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element

import numpy as np

def standalone_joint() -> Element:
    # TODO some parametrization... 
    cfg = dict(
        origin_xyz="-0.2 0.123 0.200", 
        origin_rpy="0 0.01 0.2", 
        axis_xyz="0 0 1.0"
    )
    joint_elem = Element("joint",
        name="mybot/arm_j1",
        type="revolute"
    )
    
    child_elem_list = [
        Element("parent", name="mybot/base_link"),
        Element("child", name="mybot/arm_l1"),
        Element("origin", xyz=cfg['origin_xyz'], rpy=cfg['origin_rpy']),
        Element("axis", xyz=cfg['axis_xyz']),
        Element("limit", lower = "-1.2", upper = "0.6") # incomplete
    ]
        
    for child_elem in child_elem_list:
        joint_elem.append(child_elem)

    return joint_elem, cfg# TODO expected transforms, ...

def standalone_origin() -> Element:
    joint_elem = standalone_joint()[0]
    return joint_elem.find("origin")


@pytest.fixture(scope="function")
def standalone_joint_fixture():
    return standalone_joint()
@pytest.fixture(scope="function")
def standalone_origin_fixture():
    return standalone_origin()


def test_get_X_ParentJoint_NoCrash(standalone_joint_fixture):
    # For now, just want to see that at least it runs without crashing
    joint_elem = standalone_joint_fixture[0]
    # expected_X_ParentJoint = standalone_joint_fixture[1]
    res = get_X_ParentJoint(joint_elem)

def test_get_X_JointChild_NoCrash(standalone_joint_fixture):
    joint_elem = standalone_joint_fixture[0]
    res = get_X_JointChild(joint_elem, 0.2)

def test_write_origin(standalone_joint_fixture):
    joint_elem = standalone_joint_fixture[0]
    cfg = standalone_joint_fixture[1]

    origin_elem = joint_elem.find("origin")
    # just made it up, cyclic check
    # shouldn't change the origin elem too much
    X = get_X_ParentJoint(joint_elem)
    write_origin(origin_elem, X) # shouldn't crash
    
    expected_rpy = np.array(floatList_from_vec3String(cfg["origin_rpy"]))
    new_rpy = np.array(floatList_from_vec3String(origin_elem.get("rpy")))
    np.testing.assert_allclose(new_rpy, expected_rpy, atol=1e-11)

    expected_xyz = np.array(floatList_from_vec3String(cfg["origin_xyz"]))
    new_xyz = np.array(floatList_from_vec3String(origin_elem.get("xyz")))
    np.testing.assert_allclose(new_xyz, expected_xyz)



if __name__ == "__main__":
    test_this = standalone_joint()[0]
    ET.indent(test_this)
    print(ET.tostring(test_this).decode('utf-8'))

    X1 = get_X_ParentJoint(test_this)
    X2 = get_X_JointChild(test_this, 0.2)
    print(X1)
    print(X2)
    print(X1@X2)

    origin_elem = test_this.find("origin")
    write_origin(origin_elem, X1) # cyclic test... which should be similar to how it was
    print(ET.tostring(test_this).decode('utf-8'))