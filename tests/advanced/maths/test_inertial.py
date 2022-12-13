import pytest

from urdf_kit.maths.inertial import body_inertial_urdf
from urdf_kit.edit_links import grab_link_elem_by_name
from urdf_kit.edit_joints import grab_expected_joints_handle

from spatialmath import SE3
import numpy as np

def test_inertia_fuse():
    from pathlib import Path
    urdf_dir = (Path(__file__).resolve().parent/".."/".."/"data"/"kuka_iiwa").resolve()

    import xml.etree.ElementTree as ET
    urdf_root = ET.parse(urdf_dir/"joint4_fixed_at_0.urdf").getroot()

    print("="*40)
    print("before editing")
    for link_elem in urdf_root.findall("link"):
        print(body_inertial_urdf(link_elem))


    link3_elem = grab_link_elem_by_name(urdf_root, "lbr_iiwa_link_3")
    link4_elem = grab_link_elem_by_name(urdf_root, "lbr_iiwa_link_4")
    joint4_elem = grab_expected_joints_handle(urdf_root, ["lbr_iiwa_joint_4"])[0]
    link3_inertia = body_inertial_urdf(link3_elem)
    link4_inertia = body_inertial_urdf(link4_elem)
    print("="*40)
    print("Fusing the inertia of link4 to link3 (fixed joint: joint3)")
    # link3_inertia.fuse_child_link(joint4_elem, link4_elem) # exception raised, as expected
    link3_inertia.fuse_child_link(joint4_elem, link4_inertia)


    print("="*40)
    print("after editing")
    print(link3_inertia)
    print(link4_inertia)

    # --------------------------------
    # some Just-in-case test
    assert link3_inertia.I.shape == (3,3)
    np.testing.assert_allclose(link3_inertia.I.T, link3_inertia.I)

    # --------------------------------------
    # stuff computed correctly? (require some ground-truth)
    expected_new_X_LinkCom = SE3.Trans(x=0, y= -0.00031579, z=0.202237)
    print(expected_new_X_LinkCom)
    #assert link3_inertia.X_LinkCom == expected_new_X_LinkCom #probably the tolerance is too tight
    np.testing.assert_allclose(link3_inertia.X_LinkCom.data, expected_new_X_LinkCom.data, atol=1e-5)

    assert np.allclose([link3_inertia.m] , [5.7], atol=1e-6)

    expected_new_inertia = np.zeros((3,3))
    np.fill_diagonal(expected_new_inertia, [0.165439, 0.151137, 0.028302])
    expected_new_inertia[1,2] = expected_new_inertia[2,1] = 0.019782
    np.testing.assert_allclose(link3_inertia.I, expected_new_inertia, rtol=1e-4)

    # look at the modified xml (also to check the serialization)
    print(ET.tostring(link3_elem).decode('utf-8'))

    # -----------------------------------------------
    # book-keeping done properly?
    assert link4_inertia.is_merged
    assert link4_elem.find("inertial") is None
    print(ET.tostring(link4_elem).decode('utf-8'))
    # rerunning (by mistake) should cause an error
    with pytest.raises(ValueError) as e:
        link3_inertia.fuse_child_link(joint4_elem, link4_inertia)

if __name__ == "__main__":
    test_inertia_fuse()