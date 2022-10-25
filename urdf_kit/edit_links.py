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

def purge_nonprimitive_collision_geom(urdf_root: ET.ElementTree, whitelist: list[str]):
    """remove all collision meshes^ that are not explicitly defined using SCAD
    while not being in the whitelist.

    whitelist is the list of "part/ stl-mesh" file name (without file extension)

    ^ please do NOT confuse it with the link name! 
    intended to be used as post-processing of Onshape-to-Robot
    which nicely converts SCAD-files (that only uses simple primitives)
    to URDF counterparts.
    """
    print("---------------------------------------------")
    print(" purging implicitly defined collision mesh...")
    print(" note that ")
    print(" 1. we will retain those in the whitelist, if found.")
    print(" 2. the file extension should NOT be used in the list of whitelist!")
    for part_name in whitelist:
        print("  - whitelisted: ", part_name)
    for elem_link in urdf_root.iter('link'): # also for base_link!
        for elem_collision in elem_link.iter('collision'):
            elem_geom = elem_collision.find("geometry")
            if elem_geom.find("mesh") is not None:
                mesh_rospath = elem_geom.find("mesh").get("filename")
                mesh_filename = mesh_rospath.split("/")[-1] # You are not supposed to use Windows anyways
                # mesh_filename_no_ext = mesh_filename.split(".")[0] # not very robust
                assert mesh_filename[-4:] == ".stl"
                if mesh_filename[:-4] not in whitelist:
                    print("  - discarded: ", mesh_filename, " of Link [", elem_link.get("name"), "]")
                    elem_link.remove(elem_collision)