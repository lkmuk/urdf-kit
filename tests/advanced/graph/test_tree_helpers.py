from urdf_kit.graph.tree import calc_descendent_adjacency, get_root_link_name

def test_adjacency_biped(biped_tree):
    test_data = biped_tree
    urdf_root = test_data["urdf_root"]
    expected_res = test_data["expected_res"]
    res = calc_descendent_adjacency(urdf_root) # which returns a dict of list of str
    assert isinstance(res, dict)
    assert res == expected_res # TODO: allow passing the test, despite permutation in the list. 

def test_find_root(biped_tree):
    import xml
    test_data = biped_tree
    urdf_root = test_data["urdf_root"]
    true_root_link_name = test_data["root_link_name"]
    res = get_root_link_name(urdf_root)
    assert isinstance(res, str)
    assert res == true_root_link_name

if __name__ == "__main__":
    import xml.etree.ElementTree as ET
    from pathlib import Path

    case = 'biped'
    if case == 'kuka':
        data_dir = (Path(__file__).resolve().parent/".."/".."/"data"/"kuka_iiwa").resolve()
        src_urdf_path = data_dir/"model.urdf"
    elif case == 'biped':
        data_dir = (Path(__file__).resolve().parent/".."/".."/"data").resolve()
        src_urdf_path = data_dir/"biped2d_pybullet.urdf"
    else:
        raise ValueError("undefined test case name")

    urdf_root = ET.parse(src_urdf_path).getroot()
    adj_list = calc_descendent_adjacency(urdf_root)
    print("-"*45)
    print("parent  -> child(ren)")
    for key, val in adj_list.items():
        print(key, val)