import pytest
from pathlib import Path
import numpy as np
from spatialmath import SE3

@pytest.fixture()
def biped_tree() -> dict:
    test = dict()
    
    data_dir = (Path(__file__).resolve().parent/".."/"data").resolve()

    import xml.etree.ElementTree as ET
    urdf_root = ET.parse(data_dir/"biped2d_pybullet.urdf").getroot()
    test["urdf_root"] = urdf_root
    
    test["expected_res"] = dict(
        world = ['y_prismatic'],
        y_prismatic = ['z_prismatic'],
        z_prismatic = ['torso'],
        torso = ['r_upperleg', 'l_upperleg'],
        r_upperleg = ['r_lowerleg'],
        l_upperleg = ['l_lowerleg'],
        l_lowerleg = ['l_foot'],
        r_lowerleg = ['r_foot'],
    )
    test['root_link_name'] = 'world'
    return test

@pytest.fixture(scope="function")
def kuka_iiwa_joint4() -> dict:
    """
    return the urdf root <robot> element and 
    some prepared ground-truth results.

    To allow extensions and minimize the chance of misuse,
    the output is chosen to be a dictionary.
    """
    import yaml
    output = dict()
    from pathlib import Path
    urdf_dir = (Path(__file__).resolve().parent/".."/"data"/"kuka_iiwa").resolve()

    import xml.etree.ElementTree as ET
    urdf_root = ET.parse(urdf_dir/"model.urdf").getroot()
    output["urdf_root"] = urdf_root


    # ground truth rigid body motion data
    gnd_truth_datafile = urdf_dir/'joint4_gnd_truth.yaml'
    with open(gnd_truth_datafile,'r') as f:
        d = yaml.safe_load(f)
    assert isinstance(d['joint_name'], str)
    output['joint_name'] = d['joint_name']
    for key in d.keys():
        if key[:2] == "X_":
            tmp_array = np.array(d[key])
            assert tmp_array.shape == (4,4), f"key [{key}] corresponds to invalid ground-truth value!\nDouble-check {str(gnd_truth_datafile)}."
            output[key] = SE3(tmp_array)
    
    return output