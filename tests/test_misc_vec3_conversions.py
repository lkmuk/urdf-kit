import pytest

from urdf_kit.misc import stringList_from_vec3String
@pytest.mark.parametrize("test_input,correct_output",(
        ( "-0.1 0.2 0",   ["-0.1","0.2","0"]),
        ( "+0.1  0.01 0.0", ["+0.1","0.01","0.0"]),
    )
)
def test_stringList_from_vec3String(test_input, correct_output):
    res = stringList_from_vec3String(test_input)
    assert res==correct_output


from urdf_kit.misc import vec3String_from_floatList, floatList_from_vec3String
import numpy as np
@pytest.mark.parametrize("test_input,correct_output",(
        ( (-0.1,0.2,0),   "-0.1 0.2 0"),
        ( (+0.1,0.01,0.0),"0.1 0.01 0.0"),
    )
)
def test_vec3String_from_floatList(test_input, correct_output):
    res = vec3String_from_floatList(test_input)
    assert isinstance(res, str)
    assert res==correct_output # you might need to modify the test case. The expected value above were obtained based on experiment...

@pytest.mark.parametrize("test_preinput",(
        (-0.2, 0.1, 0.3),
        (0.002, 10.2, -34.1)
    )
)
def test_floatList_from_vec3String(test_preinput):
    test_input = vec3String_from_floatList(test_preinput)
    expected_res = np.array(test_preinput)
    res = np.array(floatList_from_vec3String(test_input))
    np.testing.assert_allclose(expected_res, res)
