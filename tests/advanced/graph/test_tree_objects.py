from urdf_kit.graph.tree import body_entry, kinematic_tree
import pytest

# TODO test the constructors in isolation!
# TODO compare the results against ground truth
def _test_print_graph(link_sorting: str, show_SE3: bool):
    import xml.etree.ElementTree as ET
    from pathlib import Path

    case = 'biped'
    if case == 'kuka':
        data_dir = (Path(__file__).resolve().parent/".."/".."/"data"/"kuka_iiwa").resolve()
        src_urdf_path = data_dir/"model.urdf"
    elif case == 'biped':
        data_dir = (Path(__file__).resolve().parent/".."/".."/"data").resolve()
        src_urdf_path = data_dir/"biped2d_pybullet.urdf"
    else:
        raise ValueError("undefined test case name")

    urdf_root = ET.parse(src_urdf_path).getroot()
    my_tree = kinematic_tree(urdf_root)
    my_tree.print_graph(link_sorting='depth_first',show_SE3=True)
    # my_tree.print_graph(link_sorting='breadth_first')

@pytest.mark.parametrize("link_sorting,show_SE3", 
    (
        ("depth_first", False),
        ("breadth_first", False),
        ("depth_first", True),
    )
)
def test_print_graph(link_sorting: str, show_SE3: bool):
    _test_print_graph(link_sorting, show_SE3)

if __name__ == "__main__":
    _test_print_graph('depth_first', show_SE3=False)