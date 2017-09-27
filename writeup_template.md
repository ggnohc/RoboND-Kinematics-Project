## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[forward_kinematics.rviz]: ./misc_images/forward_kinematics.rviz.jpg
[DH_diagram]: ./misc_images/DH_diagram2.png
[total_transform_formula]: ./misc_images/total_transform_formula.png
[2_rot_2_trans]: ./misc_images/2_rot_2_trans.png
[extrinsicxyz]: ./misc_images/extrinsicxyz.png
[euler_beta]: ./misc_images/euler_beta.png
[theta2_theta3]: ./misc_images/theta2_theta3.png
[consine_rule]: ./misc_images/consine_rule.gif
[consine_rule_formula]: ./misc_images/consine_rule_formula.png
[WC]: ./misc_images/WC.png
[inverse-kinematics]: ./misc_images/inverse-kinematics.png
[kuka_drop]: ./misc_images/kuka_drop.png
[kuka_10_bottle]: ./misc_images/kuka_10_bottle.png



## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

- Below is the Kuka 210 as shown in Rviz, with joints labelled.
![alt text][forward_kinematics.rviz]

- Below is the DH table populated (more details on extraction steps as below)
Note: *J1_Z* denote Joint 1, Z position; *J2_X* denote Joint 2, X position, and so on.

  i  | alpha(i-1) | a(i-1)        | d(i)              | theta(i)
  ---| ---        | ---           | ---               | ---
  1  | 0          | 0             | 0.75 (J1_Z+J2_Z)  | q1
  2  | -pi/2      | 0.35 (J2_X)   | 0                 | q2-pi/2
  3  | 0          | 1.25 (J3_Z)   | 0                 | q3
  4  | -pi/2      | -0.054 (J4_Z) | 1.5 (J4_X+J5_X)   | q4
  5  | pi/2       | 0             | 0                 | q5
  6  | -pi/2      | 0             | 0                 | q6
  EE | 0          | 0             | 0.303 (J6_X+JG_X) | 0


  - alpha(i−1) (twist angles) = angle between Z(i−1) and Z(i) measured about X(i−1) in a right-hand sense. "alpha" value are shown at bottom right of **Figure 1** below.

  ![Insert sketch for kr210 joint and link here][DH_diagram]

  **Figure 1**


  From "kr210.urdf.xacro' file excerpt below, the "a" and "d" parameter can be obtained from **X** or **Z** value of "origin" field depending on axis orientation.  *Note*: URDF file has fixed XYZ orientation based on world coordinate, while DH table has orientation based on joint axis.

  - a(i-1) (link length) = distance from Z(i−1) to Z(i) measured along X(i-1) where X(i-1) is perpendicular to both Z(i−1) to Z(i)

  - d(i) (link offset) = signed distance from X(i-1) to X(i) measured along Z(i).

  ```xml
    <!-- joints -->
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

  <!--Two-finger gripper-->
  </joint>
  <joint name="gripper_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_link"/>
    <origin xyz="0.11 0 0" rpy="0 0 0"/><!--0.087-->
    <axis xyz="0 1 0" />
  </joint>
  ```

  - theta(i) (joint angle) = angle between X(i-1) to X(i) measured about Z(i) in a right-hand sense. A variable since all kr210 joints are revolute joint, except EE joint

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

![Insert total transform formula here][total_transform_formula]

By referring to formula above, individual transformation matrix are constructed as below, with transformation matrix from base frame to frame 1 as example below:

  ``` python
    T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                   [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                   [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                   [                   0,                   0,            0,               1]])

    T0_1 = T0_1.subs(s)
  ```

Base link to gripper link homogeneous transform is denoted as below:
  ```
    T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
  ```

To compensate for orientation difference of gripper link in URDF vs DH convention, a *correction matrix* that account for 180 degree rotation about the Z-axis, followed by -90 degree about the Y-axis is added:

  ```
    R_y = Matrix([[ cos(-pi/2),  0, sin(-pi/2), 0],
                  [          0,  1,          0, 0],
                  [ -sin(-pi/2), 0, cos(-pi/2), 0],
                  [       0,     0,          0, 1]])
  ```

  ```
    R_z = Matrix([[ cos(pi), -sin(pi), 0, 0],
                  [ sin(pi),  cos(pi), 0, 0],
                  [       0,        0, 1, 0],
                  [       0,        0, 0, 1]])
  ```

The total transformation from base link to gripper is then denoted as:

  ```
    T_tot = T0_G * R_z * R_y
  ```  
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.



* Use Inverse Position Kinematics to calculate **Theta1, 2 and 3**.
  * Since the kuka arm KR210 satisfies the design of having spherical wrist with the common point of intersection, joint 5 (J5) being the wrist center (WC), we can simplify the calculation for Theta1, 2 and 3 by figuring out the relationship between base frame and WC (Xc, Yc and Zc)

  * The WC can be obtained from formula below:
    ![inverse-kinematics][inverse-kinematics]

    ![WC formula][WC]

    where Px, Py, Pz = end-effector (EE) a.k.a gripper (G) positions; WCx, WCy, WCz = wrist positions;
    dG = from DH table as depicted in _Figure 1_ of _Section 1_ above.

    By adopting _x-y-z extrinsic rotations_ convention, R0_6 can be rewritten as:

    _Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr_, where _R_corr_ is the correctional rotation matrix to compensate for mismatch between URDF and DH table.  The roll, pitch and yaw values of EE are obtained from ROS simulation and converted from quaternions to radian using transformation.euler_from_quaternion() method.



  * Theta1 (a.k.a. q1) is calculated by projecting Zc to based frame and hence:
    ```
    q1 = atan2(Yc, Xc)
    ```

  * Theta2 and Theta3 is derived from side A, B and C with cosine rule:
    where _cos(a) = (B\*\*2 + C\*\*2 - A\*\*2)/(2BC)_ and similarly for other angles/sides of the triangle.
    ![theta2_theta3 calculation][theta2_theta3]

    Note: _When referring to external resources it is critical to know the convention being employed, as Udacity course material is based on **modified** DH parameters while external resources might be using conventional DH parameters, their difference is outlined as below:_
    https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters

* Use Inverse Position to calculate **Theta4, 5 and 6**.
  * From relationship below:
    * **R0_6 = R0_1 * R1_2 * R2_3 * R3 * R4_5 * R5_6** (resultant transform based on multiplication of individual rotation)
    * **R0_6 = Rrpy** (homogeneous roll pitch yaw rotation between base link and gripper)
  * Comparing the RHS (Right Hand Side), we can deduce:
    * **R3_6 = inv(R0_3) * Rrpy** where by _Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr_

  * Once R3_6 is obtained, Theta 4, 5 and 6 can be retrieved by extracting the euler angle from rotation matrix from formulas below (source: _Lesson 2-8 Euler Angles from a Rotation Matrix_):

    ![Insert composite rotation matrix i formula here][extrinsicxyz]

    * alpha = atan2(r21, r11)
    * beta = atan2 (-r31, sqrt(r11*r11+r21*r21))
    * gamma = atan2(r32, r33)



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


Kuka arm drooping the last bottle:
![kuka dropping last bottle][kuka_drop]

10 bootles in the drop bin!
![10 bottles!][kuka_10_bottle]
