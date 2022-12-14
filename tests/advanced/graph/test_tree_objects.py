from urdf_kit.graph.tree import body_entry, kinematic_tree

import pytest
import xml.etree.ElementTree as ET
from math import pi
from numpy.testing import assert_allclose

def prepare_data_biped() -> dict:
    from pathlib import Path
    out = dict()

    # data_dir = (Path(__file__).resolve().parent/".."/".."/"data"/"kuka_iiwa").resolve()
    # src_urdf_path = data_dir/"model.urdf"
    data_dir = (Path(__file__).resolve().parent/".."/".."/"data").resolve()
    src_urdf_path = data_dir/"biped2d_pybullet.urdf"

    out['urdf_filepath'] = src_urdf_path
    out['urdf_root'] = ET.parse(src_urdf_path).getroot()
    out['expected_num_joints'] = 9
    out['expected_total_mass'] = 90.02
    return out
@pytest.fixture()
def biped_fixture():
    return prepare_data_biped()

@pytest.mark.parametrize("method", 
    (
        "depth_first",
        "breadth_first"
    )
)
def test_tree_iterator_topdown(biped_fixture, method):
    out = biped_fixture
    tree = kinematic_tree(out['urdf_root'])
    assert len(list(tree.gen_sorted_list_topdown(method))) == out['expected_num_joints']


@pytest.mark.parametrize("link_sorting,show_SE3", 
    (
        ("depth_first", False),
        ("breadth_first", False),
        ("depth_first", True),
    )
)
def test_print_graph(biped_fixture, link_sorting: str, show_SE3: bool):
    test_data = biped_fixture
    my_tree = kinematic_tree(test_data['urdf_root'])
    my_tree.print_graph(link_sorting='depth_first',show_SE3=show_SE3)

@pytest.mark.parametrize("whitelist",
    (
        ["l_foot"], # this leaf link will be made fixed wrt its parent soon so this is about "omitting" this edge case
        [], # fuse all fixed joint whenever found
    )
)
def test_merge_fixed_joints(biped_fixture, whitelist):
    test_data = biped_fixture
    my_tree = kinematic_tree(test_data['urdf_root'])
    # more specific test preparation (roughly about freezing the left leg)

    my_tree.links['torso'].fix_revolute_joint(0.0)
    my_tree.links['l_lowerleg'].fix_revolute_joint(-pi/2.05)
    my_tree.links['l_foot'].fix_revolute_joint(pi/4)
    expected_new_num_links = test_data['expected_num_joints'] - 3 + len(whitelist) + 1
    # Notes: 
    #   +1 to account for the root link
    #   there are no other fixed joints originally
    #   the whitelist must be specific, i.e. it must be valid link name, which is also fixed wrt the parent.

    # TODO maybe dump it to somewhere else instead
    my_tree.merge_fixed_joints(link_whitelist=whitelist) # shouldn't crash
    my_tree.print_graph(link_sorting='depth_first',show_SE3=False) # shouldn't crash
    assert len(my_tree.links.keys()) == expected_new_num_links

    # TODO automate more criteria & split them up
    # mass conservation
    new_total_mass = 0
    for mass_elem in my_tree.urdf_root.findall("link/inertial/mass"):
        link_mass = float(mass_elem.get("value"))
        assert link_mass >= 0 # just in case
        new_total_mass += link_mass
    assert_allclose([new_total_mass], [test_data['expected_total_mass']] )


if __name__ == "__main__":
    from urdf_kit.misc import format_then_write

    test_data = prepare_data_biped()
    my_tree = kinematic_tree(test_data['urdf_root'])
    my_tree.print_graph(link_sorting='depth_first',show_SE3=False)

    # --------------------------------------
    # used for prototyping `merge_fixed_joints`
    my_tree.links['l_lowerleg'].fix_revolute_joint(-pi/2.05)
    my_tree.links['l_foot'].fix_revolute_joint(pi/4)
    my_tree.links['torso'].fix_revolute_joint(0.0)
    outfile_path = (test_data['urdf_filepath']/'..'/'reduced.urdf').resolve()
    # format_then_write(my_tree.urdf_root, outfile_path)
    # my_tree.merge_fixed_joints(link_whitelist=['happy']) # invalid link name exception
    # my_tree.merge_fixed_joints(link_whitelist=['l_foot','torso'])
    my_tree.merge_fixed_joints() # which include merging a fixed leaf link upstream
    my_tree.print_graph(link_sorting='depth_first',show_SE3=False)
    format_then_write(my_tree.urdf_root, outfile_path) # can it serialize properly? 
    #
    # visualize & inspect the final / the original URDF using Pybullet's GUI
    import pybullet as p
    p.connect(p.GUI)
    # robot_original_ID = p.loadURDF(str(test_data['urdf_filepath']))
    robot_reduced_ID = p.loadURDF(str(outfile_path))
    # just to block the control flow.
    prompt = "Are you happy? Type yes to exit: "
    ans = ""
    while ans not in ('yes', 'y','Y'):
        ans = input(prompt)
    p.disconnect()