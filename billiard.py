import mujoco

XML = r"""
<?xml version="1.0" encoding="utf-8"?>
<mujoco>
    <compiler coordinate="local" angle="radian" inertiafromgeom="true"/>
    <option timestep="0.002" integrator="Euler" gravity="0 0 0"/>
    <default>
        <joint armature="0" damping="0" limited="true"/>
        <geom contype="1" conaffinity="1" rgba="1 1 1 1" condim="1" friction="0 0 0" density="1000" solmix="1" solref="-100000 -0" margin="0" gap="0"/>
    </default>
    <asset>
        <texture builtin="gradient" height="100" rgb1="1 1 1" rgb2="0 0 0" type="skybox" width="100"/>
        <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1" name="texgeom" random="0.01" rgb1="0.8 0.6 0.4"
        rgb2="0.8 0.6 0.4" type="cube" width="127"/>
        <texture builtin="checker" height="100" width="100" rgb1="1 0.5 0.5" rgb2="0.5 1 0.5" type="2d" name="texplane"/>
        <material name="MatPlane" reflectance="0" shininess="0" specular="0" texrepeat="10 10" texture="texplane"/>
        <material name="geom" texture="texgeom" texuniform="true"/>
    </asset>

    <worldbody>
        <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1" castshadow="true"/>
        <camera name="fixed" pos="0 0 4.9" quat="1 0 0 0" fovy="45"/>
        <camera name="external" pos="0 -4.0 4.5" quat="0.9396 0.342 0 0" fovy="45"/>

        <geom name="floor" type="plane" material="MatPlane" pos=" 0  0  0" quat="1 0 0 0" size="2.0 2.0 0.1" contype="1" conaffinity="1"/>

        <body name="object_0" pos="0 0 0.2">
            <geom name="object_0_geom" type="sphere" size="0.2" rgba="0.8 0 0 1" contype="1" conaffinity="1" mass="2"/>
            <joint name="object_0_slide_x" type="slide" axis="1 0 0" range="-3.0 3.0" damping="0"/>
            <joint name="object_0_slide_y" type="slide" axis="0 1 0" range="-3.0 3.0" damping="0"/>
        </body>

        <body name="object_1" pos="0 0 0.2">
            <geom name="object_1_geom" type="sphere" size="0.2" rgba="0 0.8 0 1" contype="1" conaffinity="1" mass="2"/>
            <joint name="object_1_slide_x" type="slide" axis="1 0 0" range="-3.0 3.0" damping="0"/>
            <joint name="object_1_slide_y" type="slide" axis="0 1 0" range="-3.0 3.0" damping="0"/>
        </body>

        <body name="object_2" pos="0 0 0.2">
            <geom name="object_2_geom" type="sphere" size="0.2" rgba="0 0 0.8 1" contype="1" conaffinity="1" mass="2"/>
            <joint name="object_2_slide_x" type="slide" axis="1 0 0" range="-3.0 3.0" damping="0"/>
            <joint name="object_2_slide_y" type="slide" axis="0 1 0" range="-3.0 3.0" damping="0"/>
        </body>

        <body name="walls" pos="0 0 0">
            <geom type="box" pos="2.0 0 0.1" size="0.1 2.1 0.1" rgba="0.5 0.5 0.5 1" density="10000000" contype="1" conaffinity="3"/>
            <geom type="box" pos="-2.0 0 0.1" size="0.1 2.1 0.1" rgba="0.5 0.5 0.5 1" density="10000000" contype="1" conaffinity="3"/>
            <geom type="box" pos="0 2.0 0.1" size="2.1 0.1 0.1" rgba="0.5 0.5 0.5 1" density="10000000" contype="1" conaffinity="3"/>
            <geom type="box" pos="0 -2.0 0.1" size="2.1 0.1 0.1" rgba="0.5 0.5 0.5 1" density="10000000" contype="1" conaffinity="3"/>
        </body>
    </worldbody>

    <keyframe>
        <key name="init_pose" qpos="0.5 0 -0.5 0 0 0.5" qvel="-1 0.1 1 -0.1 0.3 0.4"/>
    </keyframe>
</mujoco>
"""

# load model
model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

# set inital positions and velocities to keyframe
key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "init_pose")
mujoco.mj_resetDataKeyframe(model, data, key_id)

# simulate
while data.time < 1:
    mujoco.mj_step(model, data)
    print(data.qpos)  # print positions
    # print(data.qvel) # print velocities
