from urdf_kit.graph.tree import body_entry, kinematic_tree

import pytest
import xml.etree.ElementTree as ET

def prepare_data_biped() -> dict:
    from pathlib import Path
    out = dict()

    # data_dir = (Path(__file__).resolve().parent/".."/".."/"data"/"kuka_iiwa").resolve()
    # src_urdf_path = data_dir/"model.urdf"
    data_dir = (Path(__file__).resolve().parent/".."/".."/"data").resolve()
    src_urdf_path = data_dir/"biped2d_pybullet.urdf"

    out['urdf_root'] = ET.parse(src_urdf_path).getroot()
    out['expected_num_joints'] = 9
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

if __name__ == "__main__":
    urdf_root = prepare_data_biped()['urdf_root']
    my_tree = kinematic_tree(urdf_root)
    my_tree.print_graph(link_sorting='depth_first',show_SE3=False)