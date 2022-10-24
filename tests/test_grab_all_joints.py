import pytest
from urdf_kit.edit_joints import grab_all_joints, grab_expected_joints_handle
import xml.etree.ElementTree as ET

@pytest.fixture
def dummy_testcase():
    from pathlib import Path
    this_dir = Path(__file__).resolve().parent
    test_file = this_dir/"dummy.urdf"
    tree = ET.parse(test_file)
    root = tree.getroot()
    return root

def test_grab_all_joints(dummy_testcase):
    root = dummy_testcase
    res = grab_all_joints(root)
    for entry in res:
        print(entry)
    # TODO replace this with comparison with expected results


@pytest.mark.parametrize("wishlist",[(("j12", "j13")), (["j13"])])
def test_grab_match_joints_nominal(dummy_testcase, wishlist):
    urdf_root = dummy_testcase
    handle_list = grab_expected_joints_handle(urdf_root, wishlist)
    
    # verify the name is indeed correct
    assert len(handle_list) == len(wishlist)
    for i in range(len(wishlist)):
        assert wishlist[i] == handle_list[i].attrib["name"]
        


@pytest.mark.parametrize("wishlist",[(("j4", "l0", "j2", "l1")), (("my_stunning_joint","j12","j21"))])
def test_grab_match_joints_not_found(dummy_testcase, wishlist):
    urdf_root = dummy_testcase
    with pytest.raises(ValueError):
        handle_list = grab_expected_joints_handle(urdf_root, wishlist)
        print(handle_list) # for --verbose
    
@pytest.mark.parametrize("wishlist",[(("j0", "j0", "j1v", "j2")), (("j42","j012","j42"))])
def test_grab_match_joints_invalid_input_guard(dummy_testcase, wishlist):
    urdf_root = dummy_testcase
    with pytest.raises(AssertionError):
        # the wishlist can contain names that do not exist 
        # but this is irrelevant in this test --- the input validation should throw
        # the assertion error first
        handle_list = grab_expected_joints_handle(urdf_root, wishlist)
        print(handle_list) # for --verbose