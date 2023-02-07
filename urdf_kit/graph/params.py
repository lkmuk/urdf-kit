from __future__ import annotations
from . import body_inertial_urdf
import numpy as np
import dataclasses
import yaml # for stream output/ dumping


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

def _handle_polymorphic_array(data, expected_shape: tuple[int], keep_threshold: float =1e-5) -> list:
    assert keep_threshold >= 0
    assert isinstance(expected_shape, tuple)
    assert len(expected_shape) in (1,2), "Invalid 'expected shape"
    data = np.array(data)
    if len(expected_shape) == 1:
        data = data.reshape(-1) # just in case
    assert data.shape == expected_shape
    mask = np.abs(data) < keep_threshold
    data[mask] = 0
    return data.tolist()
def _nice_printout(data: dict) -> str:
    return yaml.safe_dump(data, sort_keys=False, default_flow_style=None) # tested on Python 3.9 and Pyyaml 6.0


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
        self.home_pose = _handle_polymorphic_array(self.home_pose, (4,4), keep_threshold=1e-6)
        self.screw_axis = _handle_polymorphic_array(self.screw_axis, (6,), keep_threshold=1e-6)
    def as_dict(self) -> dict:
        return dataclasses.asdict(self)
    def __repr__(self) -> dict:
        """
        the default one can be hard to inspect, hence this override function
        """
        return _nice_printout(self.as_dict())

@dataclasses.dataclass
class joint_body_dynamics_param(joint_body_kinematics_param):
    """ 
    extend `joint_body_kinematics_param` with inertial data
    """
    mass: float
    inertia: union[list[list[float]], np.ndarray] # not the most efficient but simplify the implementation
    def __post_init__(self):
        super().__post_init__() # <------ don't forget this bit
        assert self.mass > 1e-6, f"got {self.mass}"
        self.inertia = _handle_polymorphic_array(self.inertia, (3, 3),1e-9)
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
            assert self.base_mass > 1e-4, f"got {self.base_mass}"
            self.base_inertia = _handle_polymorphic_array(self.base_inertia, (3, 3))
    @property
    def base_is_mobile(self) -> bool:
        return self.base_mass is not None
    def as_dict(self) -> dict:
        return dataclasses.asdict(self)
    def __repr__(self) -> str:
        """
        the default one can be hard to inspect, hence this override function
        """
        return _nice_printout(self.as_dict())