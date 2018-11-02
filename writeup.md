## Project: Kinematics Pick & Place

My solution for the Robotics NanoDegree [Project #2](https://github.com/ladrians/RoboND-kINEMATICS-Project-P2).

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/13_10_01.png
[image5]: ./misc_images/13_10_02.png
[image6]: ./misc_images/13_10_03.png
[image7]: ./misc_images/13_10_04.png
[image8]: ./misc_images/13_10_05.png
[image9]: ./misc_images/17_eq.png
[image10]: ./misc_images/sample_result01.png

---
### Description

Implement the Inverse Kinematics for the [KUKA KR210](https://www.robots.com/robots/kuka-kr-210) manipulator, in order to perform a pick and place task in a [Gazebo](http://gazebosim.org/) simulated environment.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The robot arm consists of **6 revolute joints** ( to control the Forward kinematics ) and **2 prismatic joints**.

![alt text][image5]

The joints are labeled as follows:

![alt text][image6]

The label links from 0 to n:

![alt text][image7]

Common normals and reference frame origins:

![alt text][image8]

Finally the two-finger gripper needs an extra frame. It represents the point on the end effector that we actually care about. It differs from frame 6 only by a translation in the Z direction.

The data for the associated joints can be found in the [kr210.urdf.xacro](kuka_arm/urdf/kr210.urdf.xacro) file, the important parts are highlighted as follows:

```xml
  <!-- joints -->
  <joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
  </joint>
  <joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
  </joint>
  <joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
  </joint>
  <joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
  </joint>
  <joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
  </joint>
  ...
  <joint name="gripper_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_link"/>
    <origin xyz="0.11 0 0" rpy="0 0 0"/><!--0.087-->
    <axis xyz="0 1 0" />
  </joint>
 ```

The values from the URDF representation can be resumed in the following table:

Joint | x(m) | y(m) | z(m)
--- | --- | --- | ---
1 | 0 | 0 | 0.33
2 | 0.35 | 0 | 0.42
3 | 0 | 0 | 1.25
4 | 0.96  | 0 | -0.054
5 | 0.54 | 0 | 0
6 | 0.193 | 0 | 0
gripper | 0.11 | 0 | 0

Notice the parameters associated to roll, pitch, yaw are 0 in all of the associated joints.

The DH representation was solved using the previous information using the relative distances and joint axis and assigning the corresponding DH frames. The dictionary with the know parameters is isolated on the `init_dh_parameters` method from the [DHmodel](kuka_arm/scripts/DHmodel.py) class.

```python
self.s = {
    self.alpha0:      0,  self.a0:      0, self.d1:  0.75, self.q1:          self.q1, 
    self.alpha1: -pi/2.,  self.a1:   0.35, self.d2:     0, self.q2: -pi/2. + self.q2,
    self.alpha2:      0,  self.a2:   1.25, self.d3:     0, self.q3:          self.q3,
    self.alpha3: -pi/2.,  self.a3: -0.054, self.d4:   1.5, self.q4:          self.q4,
    self.alpha4:  pi/2.,  self.a4:      0, self.d5:     0, self.q5:          self.q5,
    self.alpha5: -pi/2.,  self.a5:      0, self.d6:     0, self.q6:          self.q6,
    self.alpha6:      0,  self.a6:      0, self.d7: 0.303, self.q7:          0
}
 ```
where:

 * `alpha*` represents the twist angles.
 * `a*` represents the link lengths.
 * `d*` represents the link offsets.
 * `q*` represents the joint variables.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The DH parameter table using the previous information for the DH representation is detailed as follows:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054  | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

The homogeneous transforms between individual neighbouring links starting at 0 and ending at the gripper and how to incrementally is constructed can be checked on the `init_transform_matrices` method from the [DHmodel](kuka_arm/scripts/DHmodel.py) class.

```python
    self.T0_1  = self.TF_Matrix(self.alpha0, self.a0, self.d1, self.q1).subs(self.s)
    self.T1_2  = self.TF_Matrix(self.alpha1, self.a1, self.d2, self.q2).subs(self.s)
    self.T2_3  = self.TF_Matrix(self.alpha2, self.a2, self.d3, self.q3).subs(self.s)
    self.T3_4  = self.TF_Matrix(self.alpha3, self.a3, self.d4, self.q4).subs(self.s)
    self.T4_5  = self.TF_Matrix(self.alpha4, self.a4, self.d5, self.q5).subs(self.s)
    self.T5_6  = self.TF_Matrix(self.alpha5, self.a5, self.d6, self.q6).subs(self.s)
    self.T6_EE = self.TF_Matrix(self.alpha6, self.a6, self.d7, self.q7).subs(self.s)

    self.T0_EE = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_EE
```

![alt text][image9]

We need to compensate for a rotation discrepancy between DH parameters and the URDF definition. The matrix is encapsulated in the `get_rot_end_effector` method from the [DHmodel](kuka_arm/scripts/DHmodel.py) class.

We solve it applying a body fix rotation about the Z axis and then about the Y axis, the implementation is as follows:

```python
def get_rot_error(self, y_angle, p_angle):

    self.Rot_Error = self.ROT_z.subs(self.y, y_angle) * self.ROT_y.subs(self.p, p_angle)
    return self.Rot_Error
...
Rot_Error = dh_model.get_rot_error(radians(180), radians(-90))
```

We get the forward kinematics of the Kuka arm.

```python
ROT_EE = ROT_EE * Rot_Error
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The last three joints for the Kuka arm are revolute and their joint axes intersect at a single point, we have a case of spherical wrist with joint_5 being the common intersection point and hence the wrist center. Kinematically we can decouple the problem into Inverse Position and Inverse Orientation.

##### 3.1 Inverse Position

The arm is a spherical wrist on the joints 4,5,6 where the position of this center is governed by the first three joints. We can obtain the wrist center position by using the complete transformation matrix (last section) based on the end-effector pose.

One such convention is the x-y-z extrinsic rotations. The resulting rotational matrix using this convention to transform from one fixed frame to another, would be:

```python
ROT_EE = ROT_EE.subs({'r':roll, 'p':pitch, 'y':yaw})

EE = Matrix([
    [px],
    [py],
    [pz]
    ])

WC = EE - (0.303) * ROT_EE[:,2]
```

Once the wrist center (WC) position is calculated, calculating theta 1 can be calculated as:

```python
theta1 = atan2(WC[1], WC[0])
```

Theta 2 and 3 are trickier to visualize but can be calculated using trigonometry and the Cosine Laws:

![alt text][image2]

The labels 2, 3 and WC are Joint 2, Joint 3, and the Wrist Center, respectively.

```python
# SSS triangle for theta 2 and theta3
side_a = 1.501
side_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35),2) + pow((WC[2]-0.75),2))
side_c = 1.25

angle_a = acos((side_b*side_b+side_c*side_c-side_a*side_a)/(2*side_b*side_c))
angle_b = acos((side_a*side_a+side_c*side_c-side_b*side_b)/(2*side_a*side_c))
angle_c = acos((side_a*side_a+side_b*side_b-side_c*side_c)/(2*side_a*side_b))

theta2 = pi / 2 - angle_a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m
```

##### 3.2 Inverse Orientation

We need to find values of the final three joint variables. Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation.

```python
R0_3 = dh_model.T0_1[0:3,0:3] * dh_model.T1_2[0:3,0:3] * dh_model.T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={dh_model.q1:theta1, dh_model.q2:theta2, dh_model.q3:theta3})

R3_6 = R0_3.transpose() * ROT_EE

# Euler angles from rotation matrix

theta4 = atan2(R3_6[2,2], -R3_6[0,2])
a = sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2])
theta5 = atan2(a, R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The code was arranged initially taken all possible tips from the [walkthrough project](https://www.youtube.com/watch?v=Gt8DRm-REt4).

Once the basics were working using the [IK_debug](kuka_arm/scripts/IK_debug.py) most of the code were refactored to be better reused, the [DHmodel](kuka_arm/scripts/DHmodel.py) class was created.

During the `IK_server` node initialization, the [DHmodel](kuka_arm/scripts/DHmodel.py) class is initialized.

The `handle_calculate_IK` callback function receives the end-effector positions and when valid the following is executed to create all the parameters non-dependant on the Trajectory Points, meaning the individual transformation matrices from the DH Parameters table.

```python
# Create symbols
dh_model.init_variables()

# Create Modified DH parameters
dh_model.init_dh_parameters()

# Define Modified DH Transformation matrix
# Create individual transformation matrices
dh_model.init_transform_matrices()
```

Then, all calculus is executed for the list of received points (theta1 to theta6 for all points) and the response is populated as a response to the simulator.

The following image details the final part of a Pick and Place sample where the blue can is dropped.

![alt text][image10]

### Resources

* [Project Walkthrough](https://www.youtube.com/watch?v=Gt8DRm-REt4)
* [Project Baseline](https://github.com/ladrians/RoboND-Rover-Project-P1)
* [Original Repository](https://github.com/udacity/RoboND-Kinematics-Project)
* [Rubric](https://review.udacity.com/#!/rubrics/972/view)
