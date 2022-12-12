from spatialmath import SE3

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

