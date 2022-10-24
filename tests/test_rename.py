from pathlib import Path
from urdf_kit.misc import print_urdf
from urdf_kit.edit_links import rename_link
from xml.etree import ElementTree as ET

def test_rename():
    this_dir = Path(__file__).resolve().parent
    test_file = this_dir/"dummy.urdf"
    tree = ET.parse(test_file)
    root = tree.getroot()

    print("\n========= before ============== ")
    print_urdf(root)
    rename_link(root, link_name_old="l1", link_name_new="l1b")
    rename_link(root, link_name_old="l2", link_name_new="link_two")

    print("\n\n========= after ============== ")
    print_urdf(root)

test_rename()