from urdf_kit.graph import params as robot_params
import numpy as np
import yaml
from pathlib import Path

def test_compositional_asdict(generate_oracle = False):
    """
    TODO split up this test
    """
    expected_res_fpath = Path(__file__).parent/".."/".."/"test_output_expected"/"extracted_params_for_dummyBot.yaml"
    if not generate_oracle:
        with open(expected_res_fpath,"r") as f:
            expected_dict = yaml.safe_load(f)

    # actually also SUTs....
    joint_mdls = [
        robot_params.joint_body_kinematics_param(
            "eta1", home_pose=np.eye(4), screw_axis=[1, 0, 0, 0, 0, 1], parent_link_name="base", child_link_name="link1"
        ),
        robot_params.joint_body_kinematics_param(
            "eta2", home_pose=np.eye(4), screw_axis=[1, 0, 0, 0, 0, 1], parent_link_name="link1", child_link_name="link2"
        )
    ]

    # the real SUT
    robot_mdl = robot_params.robot_kinematics(robot_name="dummy", joints=joint_mdls)

    if not generate_oracle:
        assert robot_mdl.as_dict() == expected_dict
    # print(robot_mdl.as_dict())

    if generate_oracle:
        print("dumping the extracted parameters to "+str(expected_res_fpath.resolve()))
        with open(expected_res_fpath,"w") as f:
            yaml.safe_dump(robot_mdl.as_dict(), f, sort_keys=False, default_flow_style=None)
            # https://stackoverflow.com/questions/23716531/python-yaml-dump-format-list-in-other-yaml-format


if __name__ == "__main__":
    test_compositional_asdict(generate_oracle=True)