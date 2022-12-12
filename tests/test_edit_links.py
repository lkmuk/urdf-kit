import pytest
from pathlib import Path
from urdf_kit.misc import print_urdf
from urdf_kit.edit_links import rename_link, grab_link_elem_by_name, grab_elems_dict_by_joint_name
from xml.etree import ElementTree as ET

@pytest.fixture(scope="function")
def dummy_urdf_root():
    this_dir = Path(__file__).resolve().parent
    test_file = this_dir/"dummy.urdf"
    tree = ET.parse(test_file)
    root = tree.getroot()
    return root

def test_grab_link_elem_by_name(dummy_urdf_root):
    urdf_root = dummy_urdf_root
    test_link_name = 'l2'
    res_elem = grab_link_elem_by_name(urdf_root, test_link_name)
    assert res_elem.tag == "link"
    assert res_elem.get("name") == test_link_name

def test_grab_link_elem_by_name_NotFound(dummy_urdf_root):
    urdf_root = dummy_urdf_root
    test_link_name = 'funny'
    with pytest.raises(ValueError) as e:
        res_elem = grab_link_elem_by_name(urdf_root, test_link_name)
        # assert res_elem.get("name") == test_link_name 


def test_grab_elems_dict_by_joint_name(dummy_urdf_root):
    urdf_root = dummy_urdf_root
    # some "attributes" of the expected results
    test_joint_name = 'j13'
    test_parent_name = 'l1'
    test_child_name = 'l3'

    out_dict = grab_elems_dict_by_joint_name(urdf_root, test_joint_name)
    for expected_key in ("joint_elem", "child_elem", "parent_elem"):
        assert expected_key in out_dict.keys()
    
    assert out_dict['joint_elem'].tag == "joint"
    assert out_dict['joint_elem'].get('name') == test_joint_name
    assert out_dict['joint_elem'].find("parent").get('link') == test_parent_name
    assert out_dict['joint_elem'].find("child").get('link') == test_child_name

    assert out_dict['parent_elem'].tag == "link"
    assert out_dict['parent_elem'].get("name") == test_parent_name

    assert out_dict['child_elem'].tag == "link"
    assert out_dict['child_elem'].get("name") == test_child_name


def test_rename(dummy_urdf_root):
    root = dummy_urdf_root
    print("\n========= before ============== ")
    print_urdf(root)
    rename_link(root, link_name_old="l1", link_name_new="l1b")
    rename_link(root, link_name_old="l2", link_name_new="link_two")

    print("\n\n========= after ============== ")
    print_urdf(root)

