from spatialmath import SE3
import numpy as np
np.set_printoptions(precision=8, suppress=True)
# test case: joint 4 of kuka iiwa

# omg... spent a whole hour to figure out why the result computed here is not correct.
# this is why we need automation...
# X_ParentJoint = SE3.Tz(0.02155)@SE3.Rx(1.57079632679,unit='rad')
X_ParentJoint = SE3.Tz(0.2155)@SE3.Rx(1.57079632679,unit='rad')
print("-"*30, "Parent link -> Joint", X_ParentJoint, sep='\n')
X_ParentCparent = SE3.Trans([0,0.03,0.13])
print("-"*30, "Parent link -> Com parent", X_ParentCparent, sep='\n')
X_ChildCchild = SE3.Trans([0,0.067,0.034])
print("-"*30, "Child link -> Com child", X_ChildCchild, sep='\n')

X_CparentCchild = X_ParentCparent.inv()@X_ParentJoint@X_ChildCchild
print("-"*30, "Com parent -> Com child ", X_CparentCchild, sep='\n')

################################
print("="*30)
print("calculating the ground-truth for fusing link4's inertia into link3")
vec_CparentCnew_Cparent = X_CparentCchild.t *(2.7/5.7)
print("offset of the combined CoM wrt the old CoM frame of link 3")
print(vec_CparentCnew_Cparent)
print("offset of the combined CoM wrt the link 3") # which has the same orietnation as its CoM frame
print(vec_CparentCnew_Cparent + X_ParentCparent.t)

X_CparentCnew = SE3.Trans(vec_CparentCnew_Cparent)
print("-"*30 , "parent Com (old) -> new Com frame ", X_CparentCnew, sep='\n')

X_ParentCnew = X_ParentCparent@X_CparentCnew
print("-"*30 , "parent -> new Com ", X_ParentCnew.data, sep='\n')