import pytest
from xml.etree import ElementTree as ET
from urdf_kit.edit_transmission import make_simple_transmission_elem

@pytest.mark.parametrize("joint_name,mode",(("dummy1","position"),("dummy2","velocity"),("my_joint","effort")))
def test_mk_simple_transmission_elem_should_pass(joint_name, mode):
    tx_elem = make_simple_transmission_elem(joint_name, mode)
    ET.indent(tx_elem)
    print(ET.tostring(tx_elem).decode())


@pytest.mark.parametrize("joint_name,mode",(("dummy1","Position"),("dummy2","torque")))
def test_mk_simple_transmission_elem_should_fail(joint_name, mode):
    with pytest.raises(ValueError) as e:
        tx_elem = make_simple_transmission_elem(joint_name, mode)
        ET.indent(tx_elem)
        print(ET.tostring(tx_elem).decode())
    # assert e.value.code == -404

if __name__ == "__main__":
    _ = make_simple_transmission_elem("abc", mode="funny") # watch it fail 