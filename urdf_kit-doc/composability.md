# A modular framework to URDF

## Background

A robot component can be 
* mobile base
* manipulator arm
* gripper
* pan-tilt unit 
* ...

A robot typically has either 0 (i.e. robot being fixed) or exactly one mobile base,
and many other fancy components, e.g. dual-arms ... All of these components might be reused for/ from other projects.

Xacro by itself provides a valuable infrastructure for composability & reusability. Unfortunately, there isn't much guideline (AFAIK) on how to use it effectively. Some people love to "instantiate" a component **by** **just** including a xacro file --- feels convenient but it does not allow multiple instances within each robot. While this limitation is irrelevant for a mobile base, it is for grippers, arms etc... This framework is an attempt to formalize my approach :sunglasses:. It has the following principles :

1. Global variables" are completely **avoided** 
   (so everything *shall* be done using the `xacro:macro` mechanism). 
2. Namespacing of robot instance/ robot components are also **explicitly** considered --- it should be transparent at the top assembly level, as well as flexible enough 
   (e.g. allowing multiple instances of the same component (type) without name clash)

> Side note: At the moment, I prefer **not** to use xacro for anything other than dealing with namespacing. I prefer to make all parameters of the robot inside the CAD, and generate the URDF.
>
> Nonetheless, nothing stops you from reintroducing global variables, hence allowing you to "parameterize" your robot.

## So what is the framework then?

This framework requires us to have the following kinds of xacro files:

1. a xacro representing top-level assembly

   It represents the COMPLETE assembly of your robot (type).
   Here, you instantiate the components by invoking the macros.
   This xacro file shall be authored by yourself,
   probably not much to automate.
   More importantly, it should be quite transparent what is going on.

2. some xacros representing the components
   Each xacro *shall* contains (exactly?) one macro definition.
   The macro shall have an unique *name*, and only one argument which is `ns`, standing for namespace.
   Inside that macro, we expect ...
   
   1. links and joints are prefixed by the xacro macro argument, e.g. a link will look like 
       `${ns}/my_link`.
       Naturally, this might also concern plugin instances, and other stuffs that references links and/or joints, e.g. `<transmission>`.
   2. To avoid potential bugs, you should NOT include any xacro file inside that macro. 
      (TODO: is it possible at the first place?)
   

Let's look at a minimal example

### A top-level assembly

```xml
<robot name="my_mobile_manipulator" xmlns:xacro="https://ros.org/wiki/xacro">
    <xacro:arg name="namespace"/>

    <xacro:include filename="($find my_mobile_base_description)/urdf/car.urdf.xacro"/>
    <xacro:include filename="($find my_mobile_base_gazebo)/urdf/car.gazebo.xacro"/>
    <!-- 
   The .gazebo file contains simulation-specific parameters, e.g. 
   * enabling self collision, 
            * plugin instances (e.g. sensors, wrench generation etc.)
            * extra joints (e.g. for physics-based simulation of four-bar linkage, gearbox)
            * parameters concerning contacts & friction cone.
   * ... 

   It might also be reasonable to define these at this top-level assembly.
            It's up to you.

            For more details, see SDFormat.org and 
            https://classic.gazebosim.org/tutorials?tut=ros_urdf
  -->
    <xacro:mk_my_mobile_base           ns="${arg namespace}"/>
    <xacro:mk_my_mobile_base_sim_elems ns="${arg namespace}"/>

    <xacro:include filename="($find my_arm_description)/urdf/arm.urdf.xacro">
        <xacro:mk_my_awesome_arm ns="${arg namespace}/arm1"/>
        <xacro:mk_my_awesome_arm ns="${arg namespace}/arm2"/>

        <joint name="${arg namespace}/base_to_arm1" type="fixed">
            <parent link="${arg namespace}/arm1_connector"/>
            <child link="${arg namespace}/arm1/base_link"/>
            <!--
    Let's suppose I've already defined the "connector" 
                inside the parent subassembly so
            I don't have to define the "details" here.

           An alternative approach is shown for attaching arm2 to the mobile base,
           (possibly there are more alternatives!)
   -->
        </joint>

        <joint name="${arg namespace}/base_to_arm2" type="fixed">
            <parent link="${arg namespace}/base_link"/>
            <child link="${arg namespace}/arm2/base_link"/>
            <origin xyz="0.3 -0.2 0.08" rpy="0 0 -0.2"/>
        </joint>
</robot>
```

### A reusable subassembly/ component

An example of such for the preceding example:

```xml
<robot name="arm_prototype123" xmlns:xacro="https://ros.org/wiki/xacro">
    <xacro:macro name="mk_my_awesome_arm" params="ns">
        <link name="${ns}/base_link"/>
        <link name="${ns}/base_link_inertia">
             ...
        </link>         
        <link name="${ns}/upper_arm">
        	...
    	</link>
        <joint name="${ns}/base_joint" type="fixed">
            <parent link="${ns}/base_link"/>
            <child link="${ns}/base_link_inertia"/>
        </joint>
        <joint name="${ns}/shoulder" type="revolute">
            <parent link="${ns}/base_link"/> 
            <!--sidenote: probably better than using base_link_inertia -->
            <origin xyz="0  0.1"/>
            <axis xyz="0 0 1"/>
            <child link="${ns}/upper_arm"/>
            <limit lower="-0.2" upper="0.8"/>
        </joint>
        ...
    </xacro:macro>
</robot>
```

## Tool support for the framework

###  Transforming (texturally) a namespace-unaware URDF to a reusable xacro tree.
If you already have a manually written URDF, all you need to do is to 
1. ensure it has no `xacro:include`(probably it should be done somewhere else instead, e.g. on the top assembly) and 
2. inject the `${ns}` prefix in the right places
3. copy the boiler plate)
All these should be doable within 3~30 minutes.
I don't offer a tool for this case.
But what if the mechanical designs of your components are modified **constantly**
and you need to develop the SW stack simultaneously?
You probably want to have some extent of **automation** for "exporting" CAD into URDF.
See ??? for more details on such idea.
`urdf_kit.composition` is primarily intended for such case.
In this case, namespace are injected programmatically (such program is component-specific and not governed by this framework)

### Graph simplification (TODO)

The "parent" subassembly (e.g. the mobile base or fixture, or the end effector of an arm) 
will define a "connector" frame (typically a dummy link).
There will possibly some chain of fixed joints that could have been chained together.
If you need to get rid of them (e.g. for inverse dynamics calculation). 
take a look at the `urdf_kit.simplify_graph` submodule.
