from urdf_kit.graph import params as robot_params
import numpy as np
import yaml

EXPECTED_YAML = """
robot_name: dummy
joints:
- joint_name: eta1
  home_pose:
  - - 1.0
    - 0.0
    - 0.0
    - 0.0
  - - 0.0
    - 1.0
    - 0.0
    - 0.0
  - - 0.0
    - 0.0
    - 1.0
    - 0.0
  - - 0.0
    - 0.0
    - 0.0
    - 1.0
  screw_axis:
  - 1
  - 0
  - 0
  - 0
  - 0
  - 1
  parent_link_name: base
  child_link_name: link1
- joint_name: eta2
  home_pose:
  - - 1.0
    - 0.0
    - 0.0
    - 0.0
  - - 0.0
    - 1.0
    - 0.0
    - 0.0
  - - 0.0
    - 0.0
    - 1.0
    - 0.0
  - - 0.0
    - 0.0
    - 0.0
    - 1.0
  screw_axis:
  - 1
  - 0
  - 0
  - 0
  - 0
  - 1
  parent_link_name: link1
  child_link_name: link2
    """

def test_compositional_asdict():
    joint_mdls = [
        robot_params.joint_body_kinematics_param(
            "eta1", home_pose=np.eye(4), screw_axis=[1, 0, 0, 0, 0, 1], parent_link_name="base", child_link_name="link1"
        ),
        robot_params.joint_body_kinematics_param(
            "eta2", home_pose=np.eye(4), screw_axis=[1, 0, 0, 0, 0, 1], parent_link_name="link1", child_link_name="link2"
        )
    ]

    robot_mdl = robot_params.robot_kinematics(
        robot_name="dummy", joints=joint_mdls)

    assert robot_mdl.as_dict() == yaml.safe_load(EXPECTED_YAML)
    # print(robot_mdl.as_dict())
    # with open("test_robot_param_export.yaml","w") as f:
    #     yaml.safe_dump(robot_mdl.as_dict(), f, sort_keys=False)


if __name__ == "__main__":
    test_compositional_asdict()