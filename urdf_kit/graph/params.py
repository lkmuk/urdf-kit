from __future__ import annotations
from . import body_inertial_urdf
import numpy as np
import dataclasses

"""
This module defines some data formats that are not really URDF-specific

Extraction of these parameters from URDF, especially for dynamics computation
is very attractive.
The extraction functionality will be implemented in another module.

Screw theory/ spatial math is chosen as the underlying representation because...
* it is intuitive
* there is no need to distinguish prismatic from revolute joints
* the spatial notation chosen here: linear part first, then rotation part
Reference: The "Modern Robotics" textbook

At the moment, this format assumes each joint has only 1-DoF.
"""

def _handle_polymorphic_vector(data, expected_sz: int) -> list[float]:
    if isinstance(data, np.ndarray):
        assert data.shape == (expected_sz,)
        return data.tolist()
    else:
        assert np.array(data).shape == (expected_sz,)
        return list(data) # just in case, it's a tuple, which will cause a pytest to fail
def _handle_polymorphic_matrix(data, expected_num_rows: int, expected_num_cols: int) -> list[list[float]]:
    if isinstance(data, np.ndarray):
        assert data.ndim == 2
        assert data.shape == (expected_num_rows, expected_num_cols)
        return data.tolist()
    else:
        assert np.array(data).shape == (expected_num_rows, expected_num_cols)
        return data


@dataclasses.dataclass
class joint_body_kinematics_param:
    """
    It is intended for data serialization/ export
    so you probably have already finished other operations before constructing such objects
    
    As of now, I assume the convention that the pose/ screw pertains to the one 
        * from the parent 
        * to the child, 
        * expressed in parent frame
    (so no extra data members)

    It is intended for data serialization/ export
    so you probably have already finished other operations before constructing such objects

    TODO: check if the screw axis is really normalized
    """
    joint_name: str
    home_pose: union[np.ndarray, list[list[float]]]
    screw_axis: union[np.ndarray, list[float]]
    parent_link_name: str
    child_link_name: str
    def __post_init__(self):
        self.home_pose = _handle_polymorphic_matrix(self.home_pose, 4,4)
        self.screw_axis = _handle_polymorphic_vector(self.screw_axis, 6)
    def as_dict(self) -> dict:
        return dataclasses.asdict(self)

@dataclasses.dataclass
class joint_body_dynamics_param(joint_body_kinematics_param):
    """ 
    extend `joint_body_kinematics_param` with inertial data
    """
    mass: float
    inertia: union[list[list[float]], np.ndarray] # not the most efficient but simplify the implementation
    def __post_init__(self):
        assert self.m > 1e-4
        self.inertia = _handle_polymorphic_matrix(self.inertia, 3, 3)
        # TODO check symmetry, positive definiteness?

@dataclasses.dataclass
class robot_kinematics:
    robot_name: str
    joints: list[joint_body_kinematics_param]
    def as_dict(self) -> dict:
        return dataclasses.asdict(self)

@dataclasses.dataclass
class robot_dynamics():
    robot_name: str
    base_mass: float # if you a fixed base, leave both `base_mass` and `base_inertia` as None 
    base_inertia: union[list[list[float]], np.ndarray]
    joints: list[joint_body_dynamics_param]
    def __post_init__(self):
        if self.base_mass is None:
            self.base_inertia = None
        else:
            assert self.base_mass > 1e-4
            self.base_inertia = _handle_polymorphic_matrix(self.base_inertia, 3, 3)
    def as_dict(self) -> dict:
        return dataclasses.asdict(self)