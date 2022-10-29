import pytest
from urdf_kit.composition import make_component_def_macro
from xml.etree import ElementTree as ET

@pytest.fixture
def dummy_xacro_testcase():
    from pathlib import Path
    this_dir = Path(__file__).resolve().parent
    test_file = this_dir/"test_input"/"dummy.urdf.intermediate"
    tree = ET.parse(test_file)
    urdf_root = tree.getroot()

    with open(this_dir/"test_output_expected"/"dummy.urdf.xacro","r") as f:
        expected_res_str_list = f.readlines()
    expected_res_str = ""
    for line in expected_res_str_list:
        expected_res_str += line
    return urdf_root, expected_res_str


@pytest.fixture
def dummy_xacro_messed():
    from pathlib import Path
    this_dir = Path(__file__).resolve().parent
    test_file = this_dir/"test_input"/"dummy.urdf.intermediate"
    tree = ET.parse(test_file)
    urdf_root = tree.getroot()

    nasty_elem = ET.Element("xacro:include", filename=r"($ find my_rospkg)/sth_fishy.xacro")
    urdf_root.append(nasty_elem)

    return urdf_root


def test_mk_reusable_macro_nominal(dummy_xacro_testcase):
    urdf_root, expected_output_str = dummy_xacro_testcase
    xacro_etree = make_component_def_macro("mk_dummy", urdf_root)
    ET.indent(xacro_etree) # <-- so did I do to the "expected output"
    out_text = ET.tostring(xacro_etree).decode()
    assert out_text == expected_output_str # <-- this can be brittle

def test_mk_reusable_macro_error_on_xacro_include(dummy_xacro_messed):
    with pytest.raises(ValueError):
        xacro_etree = make_component_def_macro("mk_dummy", dummy_xacro_messed)