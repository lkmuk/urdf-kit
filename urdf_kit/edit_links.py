from . edit_joints import grab_all_joints, joint_entry_T
from xml.etree import ElementTree as ET

def rename_link(urdf_root_ptr: ET.ElementTree, link_name_old: str, link_name_new: str) -> None:
    """update the entries in both <link> and <joint>(s) that reference that link

    The "old" name corresponds to exactly one link in the existing URDF!

    I am not going to consider transmission, gazebo stuff! 
    You should inject those information after you rename the links.
    """
    link_ptr = None
    for entry in urdf_root_ptr.findall("link"):
        if entry.get("name") == link_name_old:
            link_ptr = entry
            break
    assert link_ptr is not None, "Link ["+link_name_old+"] not found!"

    # the updates
    link_ptr.attrib["name"] = link_name_new

    joint_list = grab_all_joints(urdf_root_ptr) 
    for joint_entry in joint_list:
        if joint_entry.child == link_name_old:
            joint_entry.joint_ptr.find("child").attrib["link"] = link_name_new
            joint_entry.child = link_name_new # JIC
            continue # there shouldn't be self-referencing, let's skip
        if joint_entry.parent == link_name_old:
            joint_entry.joint_ptr.find("parent").attrib["link"] = link_name_new
            joint_entry.parent = link_name_new # JIC

def purge_nonprimitive_collision_geom(urdf_root: ET.ElementTree):
    """remove all collision meshes that are not explicitly defined using SCAD
    TODO allow exception(s)

    intended to be used as post-processing of Onshape-to-Robot
    which nicely converts SCAD-files (that only uses simple primitives)
    to URDF counterparts.
    """
    print(" purging implicitly defined collision mesh...")
    for elem_link in urdf_root.iter('link'): # also for base_link!
        for elem_collision in elem_link.iter('collision'):
            elem_geom = elem_collision.find("geometry")
            if elem_geom.find("mesh") is not None:
                print("  * Link: "+elem_link.get("name")+" --- mesh:", elem_geom.find("mesh").get("filename"))
                elem_link.remove(elem_collision)