from xml.etree import ElementTree as ET
from .edit_joints import grab_all_joints

def format_then_write(urdf_root: ET.ElementTree, fpath: str):
    # self.tree.write(fpath)  # no autoformat
    ET.indent(urdf_root) # beautify
    with open(fpath, "wb") as f: # make no mistake it's 'wb' not 'w'
        f.write(ET.tostring(urdf_root) ) # requires Python 3.9+




def print_urdf(urdf_root: ET.ElementTree):
    print("-"*15, " link(s) ", "-"*15)
    for link_entry in urdf_root.findall("link"):
        print(link_entry.get("name"))

    print("-"*15, " joint(s) ", "-"*15)
    for joint_entry in grab_all_joints(urdf_root):
        print(joint_entry)

def clean_vec3_string(input_str: str, threshold: float = 1e-9) -> str:
    """reinstate structural zeros + beautify a string of 3 fp numbers
    First the string is parsed into a list of 3 floating-point numbers,
    then any element whose absolute value is smaller than the threshold 
    will be made a structural zero.
    The structural zeros **allow** more optimized code 
    for kinematics/ dynamics calculations.

    Another contribution of this function is to remove
    redundant whitespaces.

    Potential uses of this function in the context of URDF
    ------------------
    * joint/origin/@xyz
    * joint/origin/@rpy
    * joint/axis/@rpy (for non-fixed joints)
    * link/inertial/origin/@xyz
    * link/inertial/origin/@rpy
    * link/visual/origin/@xyz
    * link/visual/origin/@rpy
    * link/collision/origin/@rpy
    * link/collision/origin/@xyz

    Out of scope for this function 
    ---------------------
    * link/inertial/inertia/@i(xx|yy|zz|xy|yz|xz) 
      cf. `clean_inertia_tensor`
    """
    assert isinstance(input_str,str)
    assert 1e-7 >= threshold >= 1e-19, "The threshold has to be slightly larger than zero but neither too large!"

    val_list = input_str.split(" ")
    # just in case, there are some non-sense cases going on
    assert len(val_list)>=3, "There should be exactly 3 floating point numbers in the input string!"

    # just in case there are some contiguous whitespace and/or trailing spaces
    assert len(val_list)<=20, "Probably too many whitespaces in the input string? The input string is "+input_str
    while '' in val_list:
        val_list.remove('')
    assert len(val_list)==3, "Expect exactly three floating point numbers in the input string!"

    for i in  range(3):
        val = float(val_list[i]) # will throw an ValueError if val_list is sth crazy e.g. '1,0', '-', '3.3.3' ,...
        if abs(val) <= threshold:
            val_list[i] = "0"
        # # just keep how it is
        # else:
        #    val_list[i] = f"{val_list[i]:.3e}"
    return " ".join(val_list)


def clean_inertia_tensor(inertia_elem_ptr: ET.Element, threshold=1e-12) -> None:
    """reinstate structural zeros of an <inertia> element in a URDF
    For the main motivation, see the docstring for clean_vec3_string
    """
    assert isinstance(inertia_elem_ptr, ET.Element)
    assert inertia_elem_ptr.tag=="inertia"
    assert 1e-8 >= threshold >= 1e-30, "The threshold has to be slightly larger than zero but neither too large!"

    # moment of inertia
    for suffix in ("xx","yy","zz"):
        attrib_name = "i"+suffix
        val = inertia_elem_ptr.attrib[attrib_name] # may throw KeyError
        val = float(val) # may throw ValueError
        assert val >= 0, f"moment of inertia shall never be -ve!, got {val:.4e} for {attrib_name}"
        if val <= threshold:
            inertia_elem_ptr.attrib[attrib_name]="0" 
            # make sure to write as string, not fp!!!

    # product of inertia
    for suffix in ("xy","yz","xz"):
        attrib_name = "i"+suffix
        val = inertia_elem_ptr.attrib[attrib_name]
        val = float(val)
        assert val <= 0, f"moment of inertia shall never be +ve!, got {val:.4e} for {attrib_name}"
        if val >= -threshold: # note the sign !!!
            inertia_elem_ptr.attrib[attrib_name]="0" 
            # make sure to write as string, not fp!!!