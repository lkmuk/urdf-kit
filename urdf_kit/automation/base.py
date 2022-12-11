import xml.etree.ElementTree as ET
from pathlib import Path

from .. misc import format_then_write, clean_vec3_string, clean_inertia_tensor
from .. edit_joints import grab_all_joints
from .. edit_links import rename_link, purge_nonprimitive_collision_geom
from .. composition import make_component_def_macro

from typing import Union
from sys import exit

# https://stackoverflow.com/questions/27265322/how-to-print-to-console-in-color
color_code = {
    'w': '\033[0m',  # white (normal)
    'r': '\033[31m', # red
    'g': '\033[32m', # green
}

#
# TODO documentation about this automation framework
# what about some physics-related simulation data? 
#   If it shall be generated from the augmentation config 
#    (e.g. to abstract away problematic plugin design issues)
#    I tend to embedded these elements also into the "main" xacro
#   Otherwise, it's recommended to separate as a .gazebo file
# For sensor plugins, I tend to add them at the top model hierarhy
# 
# examples : see ...

class export_target_spec:
    def __init__(
        self, 
        target_full_path: Union[str, Path], 
        reusable_macro_name: str # <-- specify as None to make it non-reusable, i.e. "inlined"
    ):
        if reusable_macro_name is None:
            self.make_reusable = False
        else:
            assert isinstance(reusable_macro_name, str)
            self.make_reusable = True
            self.macro_name = reusable_macro_name # no checks on its validity
        print("Export target info:")
        if self.make_reusable: 
            print(f"  - reusable xacro macro (name: {self.macro_name})")
        else:
            print("  - namespace-aware xacro (which is not recommended!)")
        
        self.path = target_full_path.resolve()
        output_dir = self.path.parent
        print("  - output file path: ", self.path)
        try:
            assert output_dir.is_dir() and output_dir.exists()
        except AssertionError:
            print(color_code['r'])
            print( "Error: Please first create this directory: "+str(output_dir))
            print(color_code['w'])
            exit()


    def make_target(self, xml_root: ET.Element):
        """wrap into reusable macro (if needed) --> format --> write file

        Args:
            xml_root (ET.Element) --- won't be modified
        """
        if self.make_reusable:
            # Be assured: this operation won't mutate `xml_root`
            # and the macro will has the param of "ns"
            print("   wrapping into a xacro:macro", f"(name: {self.macro_name})")
            product = make_component_def_macro(
                macro_name = self.macro_name, 
                intermediate_urdf_root=xml_root
            )
        else:
            product = xml_root
        print("   beautifying --> writing the file into:")
        # TODO write a warning that the file is generated???                    
        format_then_write(product, self.path)
        print("   ", self.path)

class generic_xacro_export:
    def __init__(
        self, 
        xacro_output_list: list[export_target_spec],
        # xacro_output_path: Union[str, Path], 
        OnshapeToRobotProjectDir: Union[str, Path], # <--- should be a full path
    ):
        print("project directory: ", str(OnshapeToRobotProjectDir))
        assert Path(OnshapeToRobotProjectDir).is_dir()

        
        self.src_dir = Path(OnshapeToRobotProjectDir).resolve()
        try:
            # expect a file called "robot.urdf" inside the project dir
            self.urdf_root = ET.parse(self.src_dir/"robot.urdf").getroot()  
        except FileNotFoundError:
            # nicer user error printouts for non-developers
            print(color_code["r"])
            print("Error !!! "+str(self.src_dir/"robot.urdf")+" not found...")
            print("Please at least run the command 'onshape-to-robot .' once!")
            print("Or is the project directory specified correctly?")
            print(color_code['w'])
            exit(-1)
        # we will modify `self.urdf_root` on the go.
        # # "define" this in your subclass IF you want to 
        # # separate simulation params from the "native" URDF stuff 
        # # into a separate target, say `xxx.gazebo.xacro`,
        # self.gazebo_xacro_root = None  
        # # see also `self._make_targets` 
        
        assert hasattr(xacro_output_list, "__iter__"), "Expect a list/ tuple of export_target_spec"
        for output_spec in xacro_output_list:
            assert issubclass(type(output_spec), export_target_spec)
        self.xacro_output_list = xacro_output_list

        # URDF-native data shall go to the "config.json" used by Onshape-to-Robot (OTR)
        # (even if it is not officially supported) 
        # below is an example of extra stuff that I want but not supported by OTR
        print("purge all collision STL files unless whitelisted?", end="  ")
        import json 
        with open(self.src_dir/"config.json", "r") as f:
            OTR_config_json_data = json.load(f)
        if "purgeCollisionStl_enabled" in OTR_config_json_data.keys():        
            assert isinstance(OTR_config_json_data["purgeCollisionStl_enabled"], bool) # OTR stands for Onshape-To-Robot
            if OTR_config_json_data["purgeCollisionStl_enabled"]:
                self.purge_collision_stl_enabled = True 
                print("yes (you will see what gets whitelisted/ purged soon...)")
                assert hasattr(OTR_config_json_data["purgeCollisionStl_whitelist"], "__iter__"), "It should be a list!"
                self.purge_collision_stl_whitelist = OTR_config_json_data["purgeCollisionStl_whitelist"]
            else:
                self.purge_collision_stl_enabled = False
                print("no")
        else:
            self.purge_collision_stl_enabled = False
            print("no")

        # Other data shall be in another config data.
        # Let's call these use case-specific data the "augmentation data"
        # load those augmentation config in your subclass! (cf. `_validate_augmentation_config`)
    

    # ---------------------
    # Override the methods below, whether needed
    # ----------------
    def _get_expected_joints_names(self) -> list[str]:
        """
        To be implemented by the subclass 
        whenever you have a naming convention in place.

        This is not called directly in the postprocessing_chain
        but called inside _validate_consistency_with_src_URDF
        """
        return []

    def _process_augmentation_config(self):
        """
        first validate the config file!
        then you will probably set some member variables
        required by _get_expected_joints_names.
        """
        
        pass

    
    def _validate_consistency_with_src_URDF(self):
        print("validating the joints in the source URDF...")
        print("   expecting these joints:")
        expected_joint_names = self._get_expected_joints_names()
        if len(expected_joint_names) == 0:
            print(" None ")
            return
        else:
            for expected_name in expected_joint_names:
                print("      ", expected_name)
            # counter of existence of each joint (I want more than just validating existence)
            counters = [0]*len(expected_joint_names) 
            for joint_elem in self.urdf_root.findall("joint"):
                try:
                    expected_joint_index = expected_joint_names.index(joint_elem.get("name"))
                    counters[expected_joint_index] += 1
                except ValueError: # i.e. not existing
                    continue
            for expected_name, counter in zip(expected_joint_names,counters):
                try:
                    assert counter == 1
                except AssertionError:
                    print(color_code['r'])
                    print("  Error !!! ", expected_name, " has", counter, " occurence(s); expect exactly 1!")
                    print(color_code['w'])
                    exit()  


    def _enforce_naming_convention(self):
        """
        e.g. supposedly continuous joint (e.g. rotors, wheels) 
        are exported as "revolute joints" by Onshape-to-Robot.
        (do also remove the "lower" and "upper" limits!)

        Another example: rename links according to your use-case specific convention!
        """
        pass
    def _inject_ns_aware_elems(self):
        """
        ideally URDF native concepts (e.g. transmission), 

        possible to have e.g. gazebo elems (but depreciated by some people).

        You've got to choose where to inject the elements, self.urdf_root ?
        """
        pass
    def _make_targets(self):
        """ for each target: wrap into reusable macro (if needed) --> format --> write file
        
        If you have multiple targets, you probably need to overload this method,
        specifically you may need to individualize the `xml_root` argument
        """
        for i, target in enumerate(self.xacro_output_list):
            print(f"outputting target {i+1}...")
            target.make_target(xml_root = self.urdf_root)


    # ---------------------
    # you probably don't need to override the members below 
    # ----------------
    def _cleanup_small_values(self):
        """rounding close-to-zero values as structural zeros
        
        motivations
        1. **help** improve efficiency in kinematics & dynamics
          calculation
        2. avoid the diff being cluttered by rounding differences
        """
        print("Attempting to enforce structural zeros")
        for elem in self.urdf_root.findall("joint/origin"):
            elem.attrib['xyz'] = clean_vec3_string(elem.attrib['xyz'], threshold=1e-9)
            elem.attrib['rpy'] = clean_vec3_string(elem.attrib['rpy'], threshold=1e-8)
        for elem in self.urdf_root.findall("joint/axis"):
            elem.attrib['xyz'] = clean_vec3_string(elem.attrib['xyz'], threshold=1e-6)

        for elem in self.urdf_root.findall("link/inertial/origin"):
            elem.attrib['xyz'] = clean_vec3_string(elem.attrib['xyz'], threshold=1e-9)
            elem.attrib['rpy'] = clean_vec3_string(elem.attrib['rpy'], threshold=1e-8)    
        for elem in self.urdf_root.findall("link/visual/origin"):
            elem.attrib['xyz'] = clean_vec3_string(elem.attrib['xyz'], threshold=1e-9)
            elem.attrib['rpy'] = clean_vec3_string(elem.attrib['rpy'], threshold=1e-8)
        for elem in self.urdf_root.findall("link/collision/origin"):
            elem.attrib['xyz'] = clean_vec3_string(elem.attrib['xyz'], threshold=1e-9)
            elem.attrib['rpy'] = clean_vec3_string(elem.attrib['rpy'], threshold=1e-8)
        
        for elem in self.urdf_root.findall("link/inertial/inertia"):
            clean_inertia_tensor(elem, threshold=1e-12)

    def _reroute_to_base_link(self):
        """Flatten the default tf tree
        
        assumptions
        -----------------
        1. the source urdf is a valid URDF
        2. The link graph look likes 
           
           base_link --> link_A +--> link_B --->  ...
                                |--> link_C
                                |--> link_B1
        3. top non-dummy link coincides with the dummy base_link

        The corresponding resultant URDF graph looks like
        ---------------------
           base_link +--> link_A 
                     |--> link_B --->  ... (whatever it was)
                     |--> link_C
                     |--> link_B1
        """
        print("flattening the tree")
        list_joint_T = grab_all_joints(self.urdf_root)
        # 1. identify the immediate child of the base_link
        #    (i.e. link_A in the example above)
        base_link_name =  "base_link"
        base_link_found = False
        for joint_T in list_joint_T:
            if joint_T.parent == base_link_name:
                linkA_true_name = joint_T.child
                # TODO assert assumption 3 holds
                base_link_found = True
                break
        assert base_link_found
        # 2. modify joints with linkA as parent
        #   (we don't really need to identify linkB, linkC, etc)
        for joint_T in list_joint_T:
            if joint_T.parent == linkA_true_name:
                # what really matters
                joint_T.joint_ptr.find("parent").attrib["link"] = base_link_name
                # joint_T.joint_ptr.attrib["parent"] = base_link_name  # ooops again

                # not really necessary but JIC
                joint_T.parent = base_link_name 
        

    def _inject_ns_to_all_link_joint_elems(self):
        """
        Important: This renaming operation
        will only be done to <link> and <joint> elements.

        Other elements, e.g. transmission shall already have ${ns},
        or gazebo plugin instances
        when injected.

        Not super efficient but good enough!
        """
        print("injecting the macro parameter 'ns' to all <link> and <joint> elements")
        list_joint_T = grab_all_joints(self.urdf_root)
        # we won't update the cache variables...
        for joint_T in list_joint_T:
            joint_T.joint_ptr.attrib["name"] = r"${ns}/"+joint_T.joint_ptr.attrib["name"]
        
        for elem_link in self.urdf_root.findall("link"):
            # ONLY modify the contents of the elem_link, 
            # not adding/ removing elem from urdf_root.
            rename_link(
                self.urdf_root, 
                link_name_old = elem_link.get("name"), 
                link_name_new = r"${ns}/"+elem_link.get("name")
            )

 

        

    def postprocessing_chain(self):
        """
        A reference skeleton
        
        Remarks: when should namespace be injected?
        -------------------------
        supposedly, the source URDF file is 
        * namespace-UNaware AND
        * contains only <link> and <joint> elements

        We might want to inject new elements as well,
        e.g. <transmission>, <gazebo> etc, which will typically
        references links and joints.
        I guess it's more convenient to inject namespace 
        prefix when we inject these elements.

        To ensure the xml tree is consistent throughout the
        post-processing (which may be helpful for debugging), 
        I think it's better to 
        * ...
        * first rename links according to convention, 
        * inject namespace to all link and joint elements,
        * inject namespace-aware elements
        


        2. self._make_reusable_macro() if applicable
        3. self._write_file()
        """
        print("="*45)
        print("  begins post-processing the URDF output")
        print("  from the Onshape-To-Robot project")
        print("="*45)
        # -----------------------
        #  use-case specific stuff (please override them)
        self._process_augmentation_config()
        self._validate_consistency_with_src_URDF()
        self._enforce_naming_convention()
        self._inject_ns_aware_elems() 
        # --------------------------------------------
        
        # now it's time for more generic stuff/ making up for the limitations of Onshape-To-Robot...
        self._reroute_to_base_link()
        if self.purge_collision_stl_enabled:
            purge_nonprimitive_collision_geom(self.urdf_root, self.purge_collision_stl_whitelist)
        self._cleanup_small_values()
        self._inject_ns_to_all_link_joint_elems()
        self._make_targets()

        print(color_code['g'])
        print("  done !  ")
        print(color_code['w'])