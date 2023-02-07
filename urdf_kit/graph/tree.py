from __future__ import annotations
from xml.etree.ElementTree import Element
import numpy as np
from spatialmath import SE3
from collections import deque
from dataclasses import dataclass

from . import get_X_ParentJoint, get_X_CparentCchild, get_origin, write_origin
from . import grab_all_joints, grab_link_elem_by_name, floatList_from_vec3String, _get_axis_xyz
from .simplify import fix_revolute_joint
from . import body_inertial_urdf
from .params import joint_body_kinematics_param, robot_kinematics
from .params import joint_body_dynamics_param, robot_dynamics
from .. import color_code

"""
Unlike the `edit_xxxx` modules,
this submodule explicitly consider the kinematic tree.
The classes and functions in this submodule analyze/ modify the kinematic tree.
Under the hood, they perform appropriate actions on the underlying XML data, 
e.g. modification, deletion, ...


Such additioan layer of abstraction is helpful for 
* multibody kinematics/ dynamics computation and
* modifying the topology of the graph (e.g. when fusing joints).

"""

def calc_descendent_adjacency(urdf_root: Element) -> dict[list[str]]:
    """recompute the adjacency from scratch
    argument
    ------------
    urdf_root (Element): a handle to <robot> xml element

    return
    ----------
    (a dictionary):
        Key: the name of this link
        Value: list of its descending links
    """
    out = dict()
    for joint_entry in grab_all_joints(urdf_root):
        if joint_entry.parent in out.keys():
            out[joint_entry.parent].append(joint_entry.child)
        else:
            out[joint_entry.parent] = [joint_entry.child]
    return out

def get_root_link_name(urdf_root: Element) -> str:
    """search for the name of the root link

    identify the root link (the one that doesn't show up as a child link)
    """
    root_link_name_candidates = set([elem.get("name") for elem in urdf_root.findall("link")])
    for child_elem in urdf_root.findall("joint/child"):
        child_link_name = child_elem.get("link")
        if child_link_name in root_link_name_candidates:
            root_link_name_candidates.remove(child_link_name)
        else:
            raise ValueError(f"link [{child_link_name}] is either already claimed by a joint or doesn't even exist!")
    assert len(root_link_name_candidates) == 1, f"but got {len(root_link_name_candidates)} 'root' link(s)"
    return list(root_link_name_candidates)[0]

@dataclass
class body_entry:
    """building block of a kinematic tree (for non-root links)
    
    Notes
    -------------
    Due to the kinematic tree structure of URDF,
    each link (except the root link) has exactly 
    one unique joint that "owns" it.

    """
    joint_elem: Element # the joint that "owns" this body
    this_link_elem: Element # the child link
    parent_link_elem: Element
    def __post_init__(self):
        assert self.joint_elem.tag == "joint"
        assert self.this_link_elem.tag == "link"
        assert self.parent_link_elem.tag == "link"
        assert self.joint_elem.find("child").get("link") == self.this_link_elem.get("name")
        assert self.joint_elem.find("parent").get("link") == self.parent_link_elem.get("name")
    def __str__(self):
        return self.childLink_name
    @property
    def joint_name(self):
        return self.joint_elem.get("name")
    @property
    def parentLink_name(self):
        return self.joint_elem.find("parent").get("link")
    @property
    def childLink_name(self):
        return self.this_link_elem.get('name')
    def describe_connection(self) -> str:
        """ a more sophisticated printout
        parent --[joint]--> child
        """
        out = color_code['g']
        out += self.parentLink_name
        out += color_code['w']
        out += " --["
        out += color_code['o']
        out += self.joint_name
        out += color_code['w']
        out += "]--> "
        out += color_code['g']
        out += self.childLink_name
        out += color_code['w']
        return out
    @classmethod
    def describe_connection_format(cls) -> str:
        return "parent ---> [ joint ] ---> child link"

    @property
    def joint_type(self) -> str:
        return self.joint_elem.get("type")
    def is_fixed_wrt_parent(self) -> bool:
        return self.joint_type == "fixed"
           
    @property
    def X_ParentJoint(self) -> SE3:
        return get_X_ParentJoint(self.joint_elem)
    def get_X_ParentChild(self, *args) -> SE3:
        if self.is_fixed_wrt_parent():
            return get_X_ParentJoint(self.joint_elem)
        else:
            raise NotImplementedError("At the moment, only fixed-type joint. Yours is "+self.joint_type+".")

    @property
    def screwAx_ParentChild_Parent(self) -> np.ndarray:
        """
        specifically, this function returns the screw axis 
        * of the child (URDF) link's motion 
        * relative to the parent (URDF) link's motion 
        * expressed in the parent (URDF) link's frame

        Do NOT call this if this joint is fixed!

        (linear part-first notation)
        """
        X_ParentJoint = get_X_ParentJoint(self.joint_elem)
        R_ParentJoint = X_ParentJoint.R
        ax_Joint = _get_axis_xyz(self.joint_elem)
        ax_Parent = R_ParentJoint@ax_Joint
        if self.joint_type == "prisimatic":
            return np.array([*ax_Parent, 0,0,0])
        elif self.joint_type == "revolute":
            pos_JointParent_Parent = - X_ParentJoint.t
            return np.array([*(np.cross(ax_Parent, pos_JointParent_Parent)), *ax_Parent])
        else:
            raise ValueError("Shouldn't have called this function! Expect either prismatic/ revolute-typed joint!")
    @property
    def screwAx_ParentChild_CParent(self) -> np.ndarray:
        X_ParentCParent = get_origin(self.parent_link_elem.find("inertial/origin"))
        X_CParentParent = X_ParentCParent.inv()
        screwAx_ParentChild_CParent = X_CParentParent.Ad()@ self.screwAx_ParentChild_Parent
        return screwAx_ParentChild_CParent

    def fix_revolute_joint(self, joint_angle: float):
        fix_revolute_joint(self.joint_elem, joint_angle, verbose=True)
    def extract_names(self) -> dict[str]:
        return dict(
            joint_name=self.joint_name, 
            parent_link_name = self.parentLink_name,
            child_link_name = self.childLink_name, 
        )
    def extract_params_joint_body_kinematics(self) -> joint_body_kinematics_param:
        raise NotImplementedError()
    def extract_params_joint_body_dynamics(self) -> joint_body_dynamics_param:
        inertial_param = body_inertial_urdf(self.this_link_elem)
        return joint_body_dynamics_param(
                home_pose = get_X_CparentCchild(self.joint_elem, self.parent_link_elem, self.this_link_elem).data[0], 
                screw_axis = self.screwAx_ParentChild_CParent, 
                **self.extract_names(),
                **inertial_param.get_serializable() # 'mass' and 'inertia'
            )

# class link_entry(joint_entry_T):
class kinematic_tree:
    def __init__(self, urdf_root: Element):
        """singly-linked list of a rigid body

        Arguments
        --------------------
        urdf_root: 
            the <robot> element

        Remarks some the cached data
        ---------------------
        at the moment, the transform only consider fixed joints
        specifically we shall use them to facilitate fusing bodies.
        Otherwise, we will set the transform as None
        """
        # ---------- really basic input validation -------------------
        assert isinstance(urdf_root, Element)
        assert urdf_root.tag == "robot"
        self.urdf_root = urdf_root

        #------more simple input validation ----------------------
        num_links = len(urdf_root.findall("link"))
        num_joints = len(urdf_root.findall("joint"))
        assert num_links == num_joints + 1, f"You have {num_links} links but {num_joints} joints."
        
        # ---------------------------
        self.root_name = get_root_link_name(urdf_root)

        # -----------------------------
        # build the tree by parsing all the joints
        self.links = dict() # dict[body_entry] 
        self.links[self.root_name] = None # special treatment for the root (TODO: a better way?)
        # this list should not contain the root link
        list_link_elems = list(urdf_root.findall("link"))
        for link_elem in self.urdf_root.findall("link"):
            if link_elem.get('name') == self.root_name:
                list_link_elems.remove(link_elem)
        for joint_elem in urdf_root.findall("joint"):
            link_name = joint_elem.find("child").get("link")
            assert link_name not in self.links.keys(), "Link ["+link_name+"] should only have one parent joint but found 2."
            parent_link_elem = grab_link_elem_by_name(self.urdf_root, joint_elem.find("parent").get("link"))
            child_link_elem_found = False
            for link_elem in list_link_elems:
                if link_elem.get("name") == link_name:
                    self.links[link_name] = body_entry(
                        joint_elem = joint_elem,
                        this_link_elem = link_elem,
                        parent_link_elem = parent_link_elem
                    )
                    list_link_elems.remove(link_elem) # to speed up
                    child_link_elem_found = True
                    break # goto processing the next joint
            if not child_link_elem_found:
                raise ValueError(f"Joint [{joint_elem.get('name')}] says its child link is '{link_name}', which is not found/ has already been claimed by any child link")
    
    def get_children_entries(self, link_name: str) -> list[body_entry]:
        """ convenience function
        """
        assert isinstance(link_name,str)
        assert link_name in self.links.keys()
        table = calc_descendent_adjacency(self.urdf_root)
        return [self.links[descendent_link_name] for descendent_link_name in table[link_name] ]

    def _is_leaf_link(self, link_name: str, return_table: bool = False) -> tuple [bool, dict[list[str]] ]:
        assert isinstance(link_name,str)
        assert link_name in self.links.keys(), "got "+link_name+", recognized links:"+str(self.links.keys())
        adjacency_table = calc_descendent_adjacency(self.urdf_root)
        ans = not link_name in adjacency_table.keys()
        if return_table:
            return ans, adjacency_table
        else:
            return ans
    def is_leaf_link(self, link_name: str)-> bool:
        return self._is_leaf_link(link_name, return_table=False)

    def __len__(self):
        return len(self.links.keys())

    def get_root_link_name(self):
        return self.root_name
    _sorting_method = (
        "depth_first",
        "breadth_first"
    )
    def gen_sorted_list_topdown(self, method='depth_first') -> iter[str]:
        """
        Notice that the root element will never be returned.

        You should avoid refrain from calling this function 
        while altering the kinematic tree topology.
        TODO implement handling of such edge case?
        """
        assert isinstance(method,str)
        assert method in kinematic_tree._sorting_method

        table = calc_descendent_adjacency(self.urdf_root)
        if len(table) == 0:
            print("  This robot [", self.urdf_root.get("name"),"] has just 1 link so nothing to traverse")
            assert len(self.links) == 1, "Should be exact one link, maybe some bug leading inconsistence between this manager object and the underlying XML data?"
            return
        # initialize the buffer with those/ that directly under the root link
        buffer = deque(table[self.root_name]) # list[str]
        while len(buffer) > 0:
            # visit this link
            this_link_name =  buffer.pop()
            
            # expand this link's descendence
            if this_link_name in table.keys(): # i.e.  not leaf node
                for child_link_name in table[this_link_name]:
                    if method == 'depth_first':
                        buffer.append(child_link_name)
                    elif method == 'breadth_first':
                        buffer.appendleft(child_link_name)
                    else:
                        raise NotImplementedError()
            
            yield this_link_name
    
    def print_graph(self, link_sorting='depth_first', show_SE3=False):
        """
        supported link_sorting method: breadth_first and depth_first
        """
        table = calc_descendent_adjacency(self.urdf_root)
        print("-"*30)
        print("robot name: ", self.urdf_root.get("name"))
        print("root link name: ", self.root_name)
        print(" connections: (sorting by ", link_sorting, ")")
        print(" (format: ", body_entry.describe_connection_format(), ")")
        for link_name in self.gen_sorted_list_topdown(method=link_sorting):
            this_link_entry = self.links[link_name]
            print("  ", this_link_entry.describe_connection())
            if show_SE3:
                print("   X_ParentJoint =")
                print(this_link_entry.X_ParentJoint)
    def merge_fixed_joints(self, link_whitelist: list[str] =[]):
        """ in-place modification of the XML data as well as this object. 
        
        By merging fixed joints, we get a simpler graph,
        which is advantageous to kinematics/ dynamics calculation.

        You might want to retain some links, e.g. end-effector, camera optical frame etc.
        You can then specify them in the whitelist.
        Note that you will get an warning 
        if any of those on the whitelist are NOT found. 
        This should help identify bugs early on.

        Before that, you might want to first perform 
        `urdf_kit.graph.simplify.fix_revolute_joint` on some joints (e.g. those on a gripper)?

        Description
        -------------------
        scan for all fixed joints that shall be removed
        for each fixed joint (and the link) to be removed
            1. update joints spawning out of this link
            2. move its visual and collision elements one link up.
            3. "move" its <inertial> data one link up.
            
            4. remove xml element of the joint and element
            BY removing the associated link entry (self.links)
            TODO warn the user if other types of elements, e.g. <transmission>, 
            make references to the deleted joint/ element, or when the <link> element still contains stuff
        """
        print("="*50)
        print("Attempting to remove fixed joints as much as possible")
        print("="*50)
        for link_to_keep in link_whitelist:
            assert isinstance(link_to_keep, str), "got "+type(link_to_keep)
            assert link_to_keep in self.links.keys(), link_to_keep+" is not a link name for this robot!"
            assert self.links[link_to_keep].is_fixed_wrt_parent(), link_to_keep+" is NOT fixed wrt its parent link! Nothing to whitelist! Or is it a mistake?"
        
        pending_list = []
        for candidate_link_name in self.links.keys():
            if candidate_link_name in link_whitelist:
                print("ignoring this fixed joint: ", self.links[candidate_link_name].describe_connection())
                continue
            if candidate_link_name == self.root_name:
                continue # to avoid key-error below
            if self.links[candidate_link_name].is_fixed_wrt_parent():
                pending_list.append(candidate_link_name)    
        if len(pending_list) == 0:
            print("no fixed joints to be removed!")
            return
        else:
            print("Preview: ")
            print(" We will remove these joints (and the associated link) in a careful manner!")
            print(" (format: ", body_entry.describe_connection_format(), ")")
            for link_name in pending_list:
                print(self.links[link_name].describe_connection())
        
        for linkName_Removee in pending_list:
            jointElem_Removee = self.links[linkName_Removee].joint_elem
            linkName_Newparent = jointElem_Removee.find("parent").get("link")
            # fixed joint means removee's joint frame == removee's link frame
            X_NewparentRemovee = self.links[linkName_Removee].X_ParentJoint
            # X_NewparentRemovee = self.links[linkName_Newparent].X_ParentJoint  # oops...
            
            # reroute the joints that spawn joints spawning out of this link
            # (adjacency table would become inconsistent so regenerate it on demand)
            # TODO: scheduling of the order to remove the links, 
            #       thereby eliminating the need to recalculate adjacency
            removee_is_leaf, adjTab = self._is_leaf_link(linkName_Removee, return_table=True) # oops...
            if not removee_is_leaf:
                for linkName_Grandchild in adjTab[linkName_Removee]:
                    # under the hood, modifying the XML contents
                    # nothing needs to be changed in `self.links`
                    jointElem_Granchild = self.links[linkName_Grandchild].joint_elem
                    # 1. update <joint/parent/@link>
                    jointElem_Granchild.find("parent").attrib["link"] = linkName_Newparent
                    # 2  update <joint/origin>
                    X_RemoveeGrandchildjoint = self.links[linkName_Grandchild].X_ParentJoint
                    X_NewparentGrandchild = X_NewparentRemovee @ X_RemoveeGrandchildjoint
                    write_origin(jointElem_Granchild.find("origin"), X_NewparentGrandchild)
                    # other contents of this <joint> remains untouched
            del adjTab # to avoid accidentally reusing such inconsistent data.
            
            # move its visual and collision elements one link up.
            # which also doesn't harm the consistency of `self.links`
            linkElem_Removee = self.links[linkName_Removee].this_link_elem
            linkElem_Newparent = self.urdf_root.find(f"link/[@name='{linkName_Newparent}']")
            #
            # properly should have chosen lxml ???
            # for subElem_Removee in linkElem_Removee.findall("."): # wrong!
                # if subElem_Removee.tag not in ("visual", "collision"):
                #     continue
                # do the stuff
            # some patchy solution
            def move_geom_elem_up_one_level(geom_elem: Element):
                # 1. update the pose accordingly
                originElem = geomElem_Removee.find("origin")
                if originElem is None:
                    # URDF spec: <origin> is optional for <visual> and <collision> 
                    X_RemoveeGeom = SE3.Tx(0) # identity 
                else:
                    X_RemoveeGeom = get_origin(originElem)
                X_NewparentGeom = X_NewparentRemovee @ X_RemoveeGeom
                write_origin(originElem, X_NewparentGeom)
                # 2. detach the element from that link
                linkElem_Removee.remove(geomElem_Removee)
                # 3. attach the modified elem to the new parent link
                linkElem_Newparent.append(geomElem_Removee)
            for geomElem_Removee in linkElem_Removee.findall("visual"):
                move_geom_elem_up_one_level(geomElem_Removee)
            for geomElem_Removee in linkElem_Removee.findall("collision"):
                move_geom_elem_up_one_level(geomElem_Removee)

            # move its inertial data upstream
            isdummy_Removee = linkElem_Removee.find("inertial") is None
            isdummy_Newparent = linkElem_Newparent.find("inertial") is None
            inertial_Removee = body_inertial_urdf(linkElem_Removee, isdummy_Removee)
            inertial_Newparent = body_inertial_urdf(linkElem_Newparent, isdummy_Newparent)
            inertial_Newparent.fuse_child_link(jointElem_Removee, inertial_Removee)

            # TODO warn the user if other types of elements, e.g. <transmission>, 
            # make references to the deleted joint/ element, or when the <link> element still contains stuff
            #
            # remove xml element of the joint and element
            # BY removing the associated link entry (self.links)
            print(color_code['r'])
            print(f"removing link [{linkName_Removee}] and its associated joint [{jointElem_Removee.get('name')}]")
            print(color_code['w'])
            self.urdf_root.remove(linkElem_Removee)
            self.urdf_root.remove(jointElem_Removee)
            del self.links[linkName_Removee] # to maintain consistence
    def extract_kinematics(self) -> robot_kinematics:
        raise NotImplementedError()
    def extract_dynamics(self, base_is_mobile = True) -> robot_dynamics:
        """
        Assumptions on the graph (by the time of invoking this function)
        
        if `base_is_mobile`, the root link is the mobile base,
        otherwise, the root link is understood as the first!!! link
        """
        out = dict()
        out['robot_name'] = self.urdf_root.get("name")
        if base_is_mobile:
            baselink_elem = grab_link_elem_by_name(self.urdf_root, self.root_name)
            out['base_mass'], out['base_inertia'] = body_inertial_urdf(baselink_elem).get_serializable()
        else:
            out['base_mass'], out['base_inertia'] = None, None
        out['joints'] = []
        for keys, vals in self.links.items():
            if keys == self.root_name:
                continue
            out['joints'].append(vals.extract_params_joint_body_dynamics())
        
        return robot_dynamics(**out)
