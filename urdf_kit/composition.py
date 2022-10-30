from xml.etree import ElementTree as ET


def make_component_def_macro(macro_name: str, intermediate_urdf_root: ET.Element) -> ET.Element:
    """ A factory for wrapping a robot component into a xacro:macro
    
    This is a helper function for realising my "framework" for
    composable robots. In a nutshell, this framework consists of 
    the following xacro files:
    * reusable subassemblies/ components
      (e.g. arm, gripper, pan-tilt unit).

      Each xacro contains (exactly?) one macro definition.
      Inside that macro, we expect ...
      1. links and joints are prefixed by 
         the xacro macro argument, e.g. a link will look like 
         "${ns}/my_link".
      2. To avoid issues, you should NOT include any xacro file 
         inside that macro.
         (TODO is it possible at the first place?)

    * top-level assembly(ies) 
      representing the COMPLETE assembly of your robot:
      Here, you instantiate the components by invoking the macros.
      This xacro file shall be authored by yourself,
      probably not much to automate.

    So what is this factory about?
    It's a helper for converting some urdf (with namespace prefix) 
    
    See the documentation for more details on the framework.

    Args:
    ---------
        macro_name (str): 
            pls make sure it's a valid (and unique) xacro:macro name!
        intermediate_urdf_root (ET.Element):
            intermediate means namespace-aware,
            this concerns not just the link and joint elements,
            but also anything that references them, e.g. 
            `transmission`.

    Returns:
    ---------
        xacro_root (ET.Element)

    Explanation
    -------------
    Assuming you give the following inputs:
    * `macro_name` = "mk_firefly" and
    * `urdf_root` looks like ...
        <robot name="xyz">
            <!-- omitted for brevity -->
        </robot>

    the resultant xml tree will look like
    <robot name="xyz" xmlns:xacro="http://ros.org/wiki/xacro">
        <xacro:macro name="mk_firefly" params="ns">
           <!-- stuff copied here! -->
        </xacro:macro>
    </robot>
    """
    assert isinstance(macro_name, str)
    assert isinstance(intermediate_urdf_root.attrib["name"], str), "got "+str(intermediate_urdf_root.attrib["name"]) # <-- str() may cause another exception
    xacro_root = ET.Element("robot", name=intermediate_urdf_root.attrib["name"])
    xacro_root.attrib["xmlns:xacro"] = "http://www.ros.org/wiki/xacro"
    elem_macro = ET.SubElement(xacro_root, "xacro:macro", name=macro_name, params="ns")
    
    possible_xmlns_rep = (
        "xacro", 
        r"{http://www.ros.org/wiki/xacro}",
    )
    for elem in intermediate_urdf_root:
        # TODO test cases for detect xacro:include
        if elem.tag in (xmlns_rep+":include" for xmlns_rep in possible_xmlns_rep):
            raise ValueError(
                "xacro:include should not exist inside the intermediate_urdf_root!!!" + 
                "This has the attribute(s) of\n"+str(elem.attrib)+
                " Consider putting this include in the top model, if appropriate."
            )
        elem_macro.append(elem)

    return xacro_root