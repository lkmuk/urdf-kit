import xml.etree.ElementTree as ET
from sys import exit

# https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/classhardware__interface_1_1JointCommandInterface.html
# contrary to what https://wiki.ros.org/urdf/XML/Transmission says, 
# these work also for gazebo_ros_control
JOINT_CMD_INTERFACE_NAME = {
    "position": "hardware_interface/PositionJointInterface",
    "velocity": "hardware_interface/VelocityJointInterface",
    "effort"  : "hardware_interface/EffortJointInterface"
}
SUPPORTED_JOINT_CMD_MODE = JOINT_CMD_INTERFACE_NAME.keys()

def make_simple_transmission_elem(
    joint_name: str, 
    mode: str, 
    tx_name: str = None, 
    actuator_name: str = None
) -> ET.Element:
    # really helpful to have the defense line here.
    # Otherwise user might have no idea why the build target(s) fail.
    assert isinstance(joint_name, str), " got "+str(type(joint_name))
    if tx_name is None:
        tx_name = joint_name+"_transmission"
    else:
        assert isinstance(tx_name, str), " got "+str(type(tx_name))
    if actuator_name is None:
        actuator_name = joint_name+"_actuator"
    else:
        assert isinstance(actuator_name, str), " got "+str(type(actuator_name))

    assert isinstance(mode, str), " got "+str(type(mode))
    try:
        assert mode in SUPPORTED_JOINT_CMD_MODE
    except AssertionError:
        print("Error! Supported mode(s) are: ")
        for mode_name in SUPPORTED_JOINT_CMD_MODE:
            print("   ", mode_name)
        print("You gave: "+mode)
        exit(-404)

    elem = ET.Element("transmission", name=tx_name)

    type_elem = ET.SubElement(elem, "type")
    type_elem.text = "transmission_interface/SimpleTransmission"

    # TODO https://wiki.ros.org/urdf/XML/Transmission is just too outdated (last mod 2017-02-02 14:58:56)
    # I can confirm (from my experience) hardware_interface/EffortJointInterface also works 
    # for gazebo_ros_control's DefaultRobotHW.
    joint_elem = ET.SubElement(
        elem, "joint",
        name=joint_name
    ) # don't get confused with the usual joint element
    hardwareInterface_elem = ET.SubElement(joint_elem, "hardwareInterface")
    hardwareInterface_elem.text = JOINT_CMD_INTERFACE_NAME[mode]

    actuator_elem = ET.SubElement(elem, "actuator", name=actuator_name)
    mechRed_elem = ET.SubElement(actuator_elem, "mechanicalReduction")
    mechRed_elem.text="1" # let's make it very clear, it has to be a string!

    return elem

def test_mk_simple_transmission_elem(mode):
    tx_elem = make_simple_transmission_elem(r"${ns}/dummy", mode)
    ET.indent(tx_elem)
    print(ET.tostring(tx_elem).decode())
if __name__ == "__main__":
    test_mk_simple_transmission_elem(mode="position")