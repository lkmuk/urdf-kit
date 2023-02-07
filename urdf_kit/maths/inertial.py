from __future__ import annotations

from . import floatList_from_vec3String, vec3String_from_floatList
from . import get_origin, write_origin, get_X_CparentCchild
from . import color_code

from xml.etree.ElementTree import Element
import numpy as np
from spatialmath import SE3
# from spatialmath import SpatialInertia

__all__ = ['body_inertial_urdf']

def _make_dummy_inertial_elem() -> Element:
    inertial_elem = Element("inertial")
    subelems = (
        Element("mass", value="0"),
        Element("origin", xyz="0 0 0", rpy="0 0 0"),
        Element("inertia", ixx="0", iyy="0", izz="0", ixy="0", ixz="0", iyz="0")    
    )
    for subelem in subelems:
        inertial_elem.append(subelem)
    return inertial_elem

class body_inertial_urdf:
    def __init__(self, link_elem: Element, is_dummy: bool = False):
        """inertial properties of a rigid body, with focus on URDF processing
        
        1. fusing a fixed joint
        2. diagonization
        3. read and write routines (taking care of conversion from/to string)

        Concerning the first use case, it consists of
        simple operations like rotating the CoM-aligned frame, and/or
        introducing CoM-offseting (cf. Huygens-Steiner's theorem). 
        Here, we don't the last one as a separate function because
        it will lead to an inconsistent state (In URDF, inertial data
        must be expressed in a CoM-centered frame).

        Dummy link should not have <inertial> element.
        """
        assert link_elem.tag == "link"
        assert isinstance(link_elem, Element)
        # used in fuse_child_link and handling dummy case...
        self._link_elem = link_elem

        assert isinstance(is_dummy,bool)
        if is_dummy:
            # also fine to have an empty inertia element according to URDF specification
            # let's create one to faciliate coding.
            assert link_elem.find("inertial") is None, "A dummy link probably shouldn't have a inertial element?"
            self._inertial_elem = _make_dummy_inertial_elem()
            self._link_elem.insert(0, self._inertial_elem)
            self.make_inertial_data_dummy()
        else:
            self._inertial_elem = link_elem.find("inertial")
            if self._inertial_elem is None:
                raise ValueError("Inertial subelement not found in the given link element!")
            self.X_LinkCom = get_origin(self._inertial_elem.find("origin"))
            self.m = float( self._inertial_elem.find("mass").get("value") )
            assert self.m > 1e-5, f"got {self.m} kg"
            self.I = body_inertial_urdf.inertia_urdf_to_np_array(self._inertial_elem.find("inertia"))
            self.validate_inertia()
    
    @property
    def is_merged(self):
        """
        useful for mangaing the lifecycle
        """
        return self._inertial_elem is None
    @property
    def link_name(self):
        return self._link_elem.get("name")
    def __str__(self):
        txt = "-"*40+"\n"
        if self.is_merged:
            txt += f"Link [{self.link_name}] is already merged\n"
        else:
            txt += f"Link name: {self.link_name}\n"
            txt += "mass (kg): "+str(self.m)+"\n"
            txt += "center of mass frame offset (m):"+str(self.X_LinkCom.t)+"\n"
            txt += "center of mass frame rpy (deg):"+str(self.X_LinkCom.rpy(unit='deg'))+"\n"
            txt += "Inertia tensor (kg-m^2):\n"
            txt +=  str(self.I)
            txt += "\n"
        return txt
        
    # ==================================
    def make_inertial_data_dummy(self):
        self.X_LinkCom = SE3.Tx(0) # identity
        self.m = 0
        self.I = np.zeros((3,3),dtype=float) 
        # self._link_elem.remove(self._inertial_elem)

    @classmethod
    def inertia_urdf_to_np_array(cls, inertia_elem: Element) -> np.array:
        ixx = float(inertia_elem.get("ixx"))
        iyy = float(inertia_elem.get("iyy"))
        izz = float(inertia_elem.get("izz"))
        ixy = float(inertia_elem.get("ixy"))
        ixz = float(inertia_elem.get("ixz"))
        iyz = float(inertia_elem.get("iyz"))
        # TODO: correct convention for product of inertia?
        return np.array([
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz]
        ])
    # ==========================================
    def validate_inertia(self):
        assert self.I.shape == (3,3)
        assert np.allclose(self.I.T, self.I)
        lambdas, _ = np.linalg.eig(self.I)
        min_lambda = np.min(lambdas[0])
        assert min_lambda > 1e-7, f"The smallest eigenvalue should be +ve, got {min_lambda}"

    def diagonalize(self):
        pass

    def get_serializable(self) -> dict[float, list[list[float]]]:
        """
        a numpy array cannot be directly serialized by something like pyyaml.
        so this helper generate the two data members in the standard formats.
        """
        return {'mass': float(self.m), 'inertia': self.I.tolist()}

    def writeback(self, as_child: bool):
        assert not self.is_merged, "shouldn't call this function!"
        assert isinstance(as_child, bool), f"got {type(as_child)}"
        print("modifying the inertial data of link [",self.link_name,"]")
        if as_child:
            print(" removing the inertial tag from the xml tree")
            self._link_elem.remove(self._inertial_elem)

            # house-keeping
            # notice that the xml element lifecycle of that <inertial> is not necessarily over yet.
            # we are just ensuring that we save the state that
            # this link is already merged
            self._inertial_elem = None 
            # probably not needed
            self.make_inertial_data_dummy()

        else:
            print(" rewriting <mass>")
            subelem = self._inertial_elem.find("mass")
            subelem.attrib["value"] = str(self.m)

            print(" rewriting <origin>")
            subelem = self._inertial_elem.find("origin")
            write_origin(subelem, self.X_LinkCom)

            print(" rewriting <inertia>")
            subelem = self._inertial_elem.find("inertia")
            subelem.attrib["ixx"] = str(self.I[0][0])
            subelem.attrib["iyy"] = str(self.I[1][1])
            subelem.attrib["izz"] = str(self.I[2][2])
            subelem.attrib["ixy"] = str(self.I[0][1])
            subelem.attrib["ixz"] = str(self.I[0][2])
            subelem.attrib["iyz"] = str(self.I[1][2])

    def _steiner_(self, offset: np.array):
        """in-place
        """
        # print(offset)
        assert offset.shape == (3,)
        # offset_so3_T_offset_so3 = np.linalg.norm(offset)* np.eye(3) - np.outer(offset,offset)  # oops
        offset_so3_T_offset_so3 = np.dot(offset,offset)* np.eye(3) - np.outer(offset,offset)  # oops
        self.I += self.m * offset_so3_T_offset_so3

    def fuse_child_link(self, joint_elem: Element, child_link_inertial: body_inertial_urdf) -> None:
        """Fuse the inertia of the child link

        What this function does
        ------------------------
        * update the <link/inertia> contains for BOTH links
          In one single function avoiding
          potential inconsistency

          In fact, the child link's <inertia> element will be
          removed, which is fine according to the URDF specification,
          cf. https://wiki.ros.org/urdf/XML/link .

        what this function does NOT do
        -----------------------
        * remove the child link from the urdf tree 
          (which actually also needs to take care of the grandchildren links, if any)
        * remove the joint link
        (See also TODO)

        Built-in robustness against input error, like
        --------------------------------
        * the joint is not of fixed type
        * the joint has nothing to do 
          with the parent (i.e. this) link and the given child link.
        
        """
        assert isinstance(joint_elem, Element)
        assert joint_elem.tag == "joint"
        assert joint_elem.get("type") == "fixed"
        if not isinstance(child_link_inertial, body_inertial_urdf):
            raise ValueError(f"got {type(child_link_inertial)}, which should be body_inertial_urdf instead")
        if child_link_inertial.is_merged:
            raise ValueError(f"The Child link [{child_link_inertial.link_name}] has already been merged!")
        _err_msg_ = "The given joint element's child link is something else!!!"
        assert joint_elem.find("child").get("link") == child_link_inertial._link_elem.get("name"), _err_msg_
        assert joint_elem.find("parent").get("link") == self._link_elem.get("name"), _err_msg_
        child_link_name = joint_elem.find("child").get("link")
        #parent_link_name = joint_elem.find("parent").get("link")
        if child_link_inertial.is_merged:
            print(color_code['o'])
            print("Child link [", child_link_name, "] already merged! Nothing to compute/ update!")
            print(color_code['w'])
            return
        # ---------------------
        #   At this stage,
        #   only update the numerical data
        #   also don't modify any XML elements.
        # -----------------------
        # some preparation 
        X_ParentCparent = get_origin(self._inertial_elem.find("origin"))
        X_CparentCchild = get_X_CparentCchild(joint_elem, self._link_elem, child_link_inertial._link_elem)
        total_mass = child_link_inertial.m + self.m

        # compute the new CoM
        vec_CparentCchild_CParent = X_CparentCchild.t
        vec_CparentCnew_CParent = vec_CparentCchild_CParent * (child_link_inertial.m / total_mass)
        # self.X_LinkCom += X_ParentCparent.R @ vec_CparentCnew_CParent # oops...
        self.X_LinkCom.t += X_ParentCparent.R @ vec_CparentCnew_CParent
        
        # update the mass
        self.m = total_mass

        # update the inertia
        # (we will keep the parent CoM frame's orientation)
        # everything frame used here is wrt such frame's orientation
        R = X_CparentCchild.R
        child_link_inertial.I = R.T@child_link_inertial.I@R
        # print(child_link_inertial.I)
        vec_CchildCnew_CParent = - vec_CparentCchild_CParent + vec_CparentCnew_CParent
        
        child_link_inertial._steiner_(vec_CchildCnew_CParent)
        self._steiner_(vec_CparentCnew_CParent)

        self.I += child_link_inertial.I

        # ---------------------------------
        # final stage:
        #   writeback &
        #   maintaining consistent representation
        # -------------------------------
        self.writeback(as_child=False)
        child_link_inertial.writeback(as_child=True)
