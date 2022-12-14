from __future__ import annotations
from xml.etree.ElementTree import Element
from spatialmath import SE3
from collections import deque

from . import get_X_ParentJoint
from . import grab_all_joints, grab_link_elem_by_name
from .. import color_code

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

class body_entry:
    def __init__(self, joint_elem: Element, this_link_elem: Element):
        """building block of a kinematic tree (for non-root links)

        Tips
        -------------
        Add a world link element for mobile robots
        
        Parameters
        -----------------
        * joint_elem: Element
          the joint that "owns" this body, leave it as None for the root link
        * this_link_elem: Element
          The child link of the joint. This is the subject of this entry.

        Notes
        -------------
        Due to the kinematic tree structure of URDF,
        each link (except the root link) has exactly 
        one unique joint that "owns" it.

        A network view is needed for kinematics/ dynamics
        also for modifying the topology of the graph (e.g. when fusing joints).

        This class associates the bipartite graph 
        to the parsed xml elements, 
        which contain further data.
        """
        assert joint_elem.tag == "joint"
        assert this_link_elem.tag == "link"
        assert joint_elem.find("child").get("link") == this_link_elem.get("name")
        self.joint_elem = joint_elem
        self.this_link_elem = this_link_elem

        # more cache
        self.X_ParentJoint = get_X_ParentJoint(self.joint_elem)
    
    def __str__(self):
        return self.this_link_elem.get("name")
    def describe_connection(self) -> str:
        """ a more sophisticated printout
        parent --[joint]--> child
        """
        out = color_code['g']
        out += self.joint_elem.find("parent").get("link")
        out += color_code['w']
        out += " --["
        out += color_code['o']
        out += self.joint_elem.get("name")
        out += color_code['w']
        out += "]--> "
        out += color_code['g']
        out += self.this_link_elem.get('name')
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
    def get_X_ParentChild(self, *args) -> SE3:
        if self.is_fixed_wrt_parent():
            return self.X_ParentJoint
        else:
            raise NotImplementedError("At the moment, only fixed-type joint. Yours is "+self.joint_type+".")


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
            for link_elem in list_link_elems:
                if link_elem.get("name") == link_name:
                    self.links[link_name] = body_entry(
                        joint_elem = joint_elem,
                        this_link_elem = link_elem
                    )
                    list_link_elems.remove(link_elem) # to speed up
                    break # goto processing the next joint
                raise ValueError(f"Joint [{joint_elem.get('name')}] says its child link is '{link_name}', which is not found/ has already been claimed by any child link")
    
    def get_children_entries(self, link_name: str) -> list[body_entry]:
        """ convenience function
        """
        assert isinstance(link_name,str)
        assert link_name in self.links.keys()
        table = calc_descendent_adjacency(self.urdf_root)
        return [self.links[descendent_link_name] for descendent_link_name in table[link_name] ]

    def is_leaf_link(self, link_name: str) -> bool:
        assert isinstance(link_name,str)
        assert link_name in self.links.keys(), "got "+link_name+", recognized links:"+str(self.links.keys())
        table = calc_descendent_adjacency(self.urdf_root)
        return not link_name in table.keys()

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
        """
        assert isinstance(method,str)
        assert method in kinematic_tree._sorting_method

        table = calc_descendent_adjacency(self.urdf_root)
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