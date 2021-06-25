#!/usr/bin/env python3
"""
Example of how bodies interact with each other. For a body to be able to
move it needs to have joints. In this example, the "robot" is a red ball
with X and Y slide joints (and a Z slide joint that isn't controlled).
On the floor, there's a cylinder with X and Y slide joints, so it can
be pushed around with the robot. There's also a box without joints. Since
the box doesn't have joints, it's fixed and can't be pushed around.
"""
from mujoco_py import load_model_from_xml, MjSim, MjViewer
import math
import os

MODEL_XML = """
<?xml version="1.0" encoding="utf-8"?>
<robot name="dofbot">
    <mujoco>
        <compiler meshdir="D:/Research/robots/DofRL/dofbot_moveit/meshes/" balanceinertia="true"/>
    </mujoco>
    <link name="base_link">
        <visual>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/base_link.STL"/>
            </geometry>
            <material
                    name="">
                <color
                        rgba="0.7 0.7 0.7 1"/>
            </material>
        </visual>
        <collision>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/base_link.STL"/>
            </geometry>
        </collision>
    </link>
    <link name="link1">
        <visual>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link1.STL"/>
            </geometry>
            <material
                    name="">
                <color
                        rgba="0 0.7 0 1"/>
            </material>
        </visual>
        <collision>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link1.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint1" type="revolute">
        <origin
                xyz="0 0 0.064"
                rpy="0 0 0"/>
        <parent
                link="base_link"/>
        <child
                link="link1"/>
        <axis
                xyz="0 0 1"/>
        <limit
                lower="-1.5708"
                upper="1.5708"
                effort="100"
                velocity="1"/>
    </joint>
    <link name="link2_mm_simplified">
        <visual>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link2_mm_simplified.STL"/>
            </geometry>
            <material
                    name="">
                <color
                        rgba="0.7 0.7 0.7 1"/>
            </material>
        </visual>
        <collision>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link2_mm_simplified.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint2" type="revolute">
        <origin
                xyz="0 0 0.0435"
                rpy="0 1.5708 0"/>
        <parent
                link="link1"/>
        <child
                link="link2_mm_simplified"/>
        <axis
                xyz="0 0 1"/>
        <limit
                lower="-1.5708"
                upper="1.5708"
                effort="100"
                velocity="1"/>
    </joint>
    <link name="link3_mm_simplified">
        <visual>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link3_mm_simplified.STL"/>
            </geometry>
            <material
                    name="">
                <color
                        rgba="0 0.7 0 1"/>
            </material>
        </visual>
        <collision>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link3_mm_simplified.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint3" type="revolute">
        <origin
                xyz="-0.08285 0 0"
                rpy="0 0 0"/>
        <parent
                link="link2_mm_simplified"/>
        <child
                link="link3_mm_simplified"/>
        <axis
                xyz="0 0 1"/>
        <limit
                lower="-1.5708"
                upper="1.5708"
                effort="100"
                velocity="1"/>
    </joint>
    <link name="link4_mm_simplified">
        <visual>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link4_mm_simplified.STL"/>
            </geometry>
            <material
                    name="">
                <color
                        rgba="0.7 0.7 0.7 1"/>
            </material>
        </visual>
        <collision>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link4_mm_simplified.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint4" type="revolute">
        <origin
                xyz="-0.08285 0 0"
                rpy="0 0 0"/>
        <parent
                link="link3_mm_simplified"/>
        <child
                link="link4_mm_simplified"/>
        <axis
                xyz="0 0 1"/>
        <limit
                lower="-1.5708"
                upper="1.5708"
                effort="100"
                velocity="1"/>
    </joint>
    <link name="link5">
        <visual>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link5.STL"/>
            </geometry>
            <material
                    name="">
                <color
                        rgba="1 1 1 1"/>
            </material>
        </visual>
        <collision>
            <origin
                    xyz="0 0 0"
                    rpy="0 0 0"/>
            <geometry>
                <mesh
                        filename="package://dofbot_moveit/meshes/link5.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint5" type="revolute">
        <origin
                xyz="-0.07385 -0.00215 0"
                rpy="0 -1.5708 0"/>
        <parent
                link="link4_mm_simplified"/>
        <child
                link="link5"/>
        <axis
                xyz="0 0 1"/>
        <limit
                lower="-1.5708"
                upper="3.1416"
                effort="100"
                velocity="1"/>
    </joint>
    
    
</robot>
"""

model = load_model_from_xml(MODEL_XML)
sim = MjSim(model)
viewer = MjViewer(sim)
t = 0
while True:
    # sim.data.ctrl[0] = math.cos(t / 10.) * 0.01
    # sim.data.ctrl[1] = math.sin(t / 10.) * 0.01
    t += 1
    sim.step()
    viewer.render()
    if t > 100 and os.getenv('TESTING') is not None:
        break
