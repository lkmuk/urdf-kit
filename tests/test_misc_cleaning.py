import pytest
from urdf_kit.misc import clean_vec3_string, clean_inertia_tensor
import xml.etree.ElementTree as ET

@pytest.mark.parametrize("src,correct_output",(
    ("0 0 0.1", "0 0 0.1"),
    (" 0 0.0  1.2e-3", "0 0 1.2e-3"), # some nasty whitespace
    ("-1 3.2e-10 2e+03  ","-1 3.2e-10 2e+03")
))
def test_clean_vec_string_nominal(src, correct_output):
    output = clean_vec3_string(src, threshold=1e-11)
    assert output == correct_output


@pytest.mark.parametrize("src_dict,expected_dict",(
    (
        dict(ixx="2e-1",iyy="2e-1",izz=" 3.2e-2 ", ixy="-1.2345e-15", iyz="-2.36231e-14", ixz="-4.32e-7"),
        dict(ixx="2e-1",iyy="2e-1",izz=" 3.2e-2 ", ixy="0", iyz="0", ixz="-4.32e-7")
    ),
    (
        dict(ixx="2e-1",iyy="2e-1",izz=" 3.2e-2 ", ixy="-0", iyz="-0.00", ixz="-4.32e-7"),
        dict(ixx="2e-1",iyy="2e-1",izz=" 3.2e-2 ", ixy="0", iyz="0", ixz="-4.32e-7")
    ),
    
))
def test_clean_inertia_tensor_nominal(src_dict, expected_dict):
    elem = ET.Element("inertia", attrib=src_dict)
    clean_inertia_tensor(elem, threshold=1e-12)
    for attrib_name in expected_dict.keys():
        assert elem.get(attrib_name)==expected_dict[attrib_name]