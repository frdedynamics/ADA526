---
title: Kinematics
layout: default
parent: Theoretical Background
nav_order: 1
has_toc: true
---

# Kinematics

Kinematics describes the motion of the robot without considering the forces that cause the motion. It connects the joint positions of the robot to the position and orientation of its end effector.

In **forward kinematics**, the joint positions and the robot geometry are known. We use them to calculate the pose of the end effector. **Inverse kinematics** asks the opposite question: which joint positions give us a desired end-effector pose? We start with forward kinematics because it gives us the model that is also needed when solving the inverse problem.

## Following the frames

We describe the geometry of the robot using coordinate frames. There is a frame at the robot base, frames along the kinematic chain, and usually a final frame at the tool center point (TCP). A transformation matrix describes the position and orientation of one frame relative to another frame.

For one joint, \\(^{0}T_{1}\\) takes coordinates expressed in frame \\(\{1\}\\) and expresses them in frame \\(\{0\}\\). For a complete robot we follow the frames from the base to the tool by multiplying all neighboring transformations:

$$
{}^{0}T_{n} = {}^{0}T_{1}\,{}^{1}T_{2}\,{}^{2}T_{3}\cdots{}^{n-1}T_{n}
$$

The same idea also lets us include a robot that is mounted somewhere in the world and a tool mounted after the last robot link:

$$
{}^{W}T_{E} = {}^{W}T_{0}\,{}^{0}T_{1}\cdots{}^{n-1}T_{n}\,{}^{n}T_{E}
$$

This is the frame path we use for forward kinematics: start in the world or base frame, follow the kinematic chain one frame at a time, and end at the TCP. The order is important. Matrix multiplication does not commute, so changing the order generally gives a different pose.

## Describing the robot with DH parameters

Transformation matrices are convenient for calculations, but a table of matrices is not a very readable description of a robot. Denavit-Hartenberg (DH) parameters describe each step between two neighboring frames using four values.

The standard DH convention used here fixes how the coordinate frames are placed. **The motion axis of a joint is always a \\(z\\)-axis.** More precisely, the axis of joint \\(j\\) is \\(z_{j-1}\\). A revolute joint rotates around this axis, while a prismatic joint translates along it. This index shift is important when reading a DH table.

Before reading the parameters, we place the frames as follows:

1. Align \\(z_{j-1}\\) with the motion axis of joint \\(j\\).
2. Place \\(x_j\\) along the shortest line, called the common normal, from \\(z_{j-1}\\) to \\(z_j\\).
3. Place the origin of frame \\(\{j\}\\) where \\(x_j\\) meets \\(z_j\\).
4. Choose \\(y_j\\) so that \\(x_j\\), \\(y_j\\), and \\(z_j\\) form a right-handed coordinate frame.

For intersecting joint axes, the common normal has zero length. For parallel axes, there are several possible common normals and we choose the one that gives the most useful frame placement. Once the frames are placed, the four parameters have a clear order:

<div class="kinematics-table-scroll" markdown="1">

| Parameter | Operation | Geometric meaning |
|:----------|:----------|:------------------|
| \\(\theta_j\\) | Rotate about \\(z_{j-1}\\) | Joint angle between \\(x_{j-1}\\) and \\(x_j\\) |
| \\(d_j\\) | Translate along \\(z_{j-1}\\) | Link offset |
| \\(a_j\\) | Translate along \\(x_j\\) | Link length, or distance between the two joint axes |
| \\(\alpha_j\\) | Rotate about \\(x_j\\) | Link twist between \\(z_{j-1}\\) and \\(z_j\\) |

</div>

For a revolute joint, \\(\theta_j\\) is the joint variable while the other parameters are constant. For a prismatic joint, \\(d_j\\) is the joint variable.

### Build one DH transformation

The figure below separates the four operations that are combined in the usual DH figure. The step buttons show the fixed transformation order \\(\theta_j \rightarrow d_j \rightarrow a_j \rightarrow \alpha_j\\). At each step, the current and previous parameters remain adjustable, while parameters from later steps are greyed out. Add another link to continue from the new frame; the builder supports up to six links. The joint-angle sliders in the table can then be used to jog the complete robot. Pay particular attention to the two joint axes. After the \\(a_j\\) translation they are separated, but they remain parallel until the \\(\alpha_j\\) rotation introduces the link twist.

{% include dh_transform_demo.html %}

<details class="kinematics-example-details">
<summary>Try it: recreate a UR5</summary>
<div markdown="1">

The UR5 has six revolute joints. Add links until the builder contains six DH rows, then enter the parameters below. The angles \\(\theta_1\\) to \\(\theta_6\\) are the joint variables, so you can choose their initial values and use the table sliders to jog the robot afterwards.

The dimensions are given in metres. Use the nearest value available with the 0.01 m slider increments. A negative value for \\(a_j\\) describes the direction of the selected \\(x_j\\)-axis; it does not mean that the physical link has a negative length.

<div class="kinematics-table-scroll" markdown="1">

| Link \\(j\\) | \\(\theta_j\\) | \\(d_j\\) | \\(a_j\\) | \\(\alpha_j\\) |
|:-------------:|:----------------:|:---------:|:---------:|:-----------------:|
| 1 | \\(\theta_1\\) | 0.089159 | 0 | \\(90^\circ\\) |
| 2 | \\(\theta_2\\) | 0 | -0.425 | \\(0^\circ\\) |
| 3 | \\(\theta_3\\) | 0 | -0.39225 | \\(0^\circ\\) |
| 4 | \\(\theta_4\\) | 0.10915 | 0 | \\(90^\circ\\) |
| 5 | \\(\theta_5\\) | 0.09465 | 0 | \\(-90^\circ\\) |
| 6 | \\(\theta_6\\) | 0.0823 | 0 | \\(0^\circ\\) |

</div>

These are the nominal standard DH parameters published by [Universal Robots](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics).

</div>
</details>

The four operations give one neighboring-frame transformation:

$$
{}^{j-1}T_j = R_z(\theta_j)\,T_z(d_j)\,T_x(a_j)\,R_x(\alpha_j)
$$

Multiplying these four elementary transformations gives the standard DH transformation matrix:

$$
{}^{j-1}T_j =
\begin{bmatrix}
c_{\theta} & -s_{\theta}c_{\alpha} & s_{\theta}s_{\alpha} & a c_{\theta} \\
s_{\theta} & c_{\theta}c_{\alpha} & -c_{\theta}s_{\alpha} & a s_{\theta} \\
0 & s_{\alpha} & c_{\alpha} & d \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Here, \\(c_{\theta}=\cos(\theta_j)\\), \\(s_{\theta}=\sin(\theta_j)\\), and the same shorthand is used for \\(\alpha_j\\). The upper-left \\(3\times3\\) part describes orientation. The last column describes position.

## Worked example 7.1: planar arm

Consider a planar robot with two revolute joints and link lengths \\(a_1\\) and \\(a_2\\). All joint axes point out of the plane and are therefore parallel. The frame \\(z\\)-axes do not need a twist or an offset, which makes the DH table quite simple.

<figure class="kinematics-example-figure">
  <img src="{{ '/assets/images/kinematics/planar-arm-frames.svg' | relative_url }}" alt="Two-link planar robot with coordinate frames 0, 1, and 2, link lengths a1 and a2, and joint angles theta1 and theta2">
  <figcaption>The frames used for the DH description of the planar arm. The joint axes point out of the drawing.</figcaption>
</figure>

### Step 1: Write the DH table

<div class="kinematics-table-scroll" markdown="1">

| Link \\(j\\) | \\(\theta_j\\) | \\(d_j\\) | \\(a_j\\) | \\(\alpha_j\\) |
|:--------------:|:----------------:|:-----------:|:-----------:|:----------------:|
| 1 | \\(\theta_1\\) | 0 | \\(a_1\\) | 0 |
| 2 | \\(\theta_2\\) | 0 | \\(a_2\\) | 0 |

</div>

The angle \\(\theta_1\\) is measured from \\(x_0\\) to \\(x_1\\). The angle \\(\theta_2\\) is measured from \\(x_1\\) to \\(x_2\\), not from the base frame. This is why the orientation of link 2 in the base frame becomes \\(\theta_1+\theta_2\\).

### Step 2: Simplify the neighboring transformations

For this planar arm, \\(d_j=0\\) and \\(\alpha_j=0\\). Substituting these values into the general DH matrix gives:

$$
{}^{j-1}T_j =
\begin{bmatrix}
\cos\theta_j & -\sin\theta_j & 0 & a_j\cos\theta_j \\
\sin\theta_j & \cos\theta_j & 0 & a_j\sin\theta_j \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

For link 1, substitute \\(\theta_1\\), \\(d_1=0\\), \\(a_1\\), and \\(\alpha_1=0\\) into the general DH matrix. This gives the transformation from frame \\(\{1\}\\) to frame \\(\{0\}\\):

$$
{}^{0}T_1 =
\begin{bmatrix}
\cos\theta_1 & -\sin\theta_1 & 0 & a_1\cos\theta_1 \\
\sin\theta_1 & \cos\theta_1 & 0 & a_1\sin\theta_1 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

For link 2, substitute \\(\theta_2\\), \\(d_2=0\\), \\(a_2\\), and \\(\alpha_2=0\\). This gives the transformation from frame \\(\{2\}\\) to frame \\(\{1\}\\):

$$
{}^{1}T_2 =
\begin{bmatrix}
\cos\theta_2 & -\sin\theta_2 & 0 & a_2\cos\theta_2 \\
\sin\theta_2 & \cos\theta_2 & 0 & a_2\sin\theta_2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

### Step 3: Follow the frame path

Now we have both neighboring transformations. To express frame \\(\{2\}\\) in the base frame, multiply them in frame order:

$$
{}^{0}T_2 = {}^{0}T_1\,{}^{1}T_2
$$

After multiplying and collecting the angle terms:

$$
{}^{0}T_2 =
\begin{bmatrix}
\cos(\theta_1+\theta_2) & -\sin(\theta_1+\theta_2) & 0 & a_1\cos\theta_1+a_2\cos(\theta_1+\theta_2) \\
\sin(\theta_1+\theta_2) & \cos(\theta_1+\theta_2) & 0 & a_1\sin\theta_1+a_2\sin(\theta_1+\theta_2) \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

### Step 4: Read the result

The last column gives the position of the end of link 2 in the base frame:

$$
x = a_1\cos\theta_1+a_2\cos(\theta_1+\theta_2)
$$

$$
y = a_1\sin\theta_1+a_2\sin(\theta_1+\theta_2)
$$

The upper-left \\(3\times3\\) block gives the orientation of frame \\(\{2\}\\) relative to the base frame:

$$
{}^{0}R_2 =
\begin{bmatrix}
\cos(\theta_1+\theta_2) & -\sin(\theta_1+\theta_2) & 0 \\
\sin(\theta_1+\theta_2) & \cos(\theta_1+\theta_2) & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

This has the same form as the standard rotation matrix for a rotation by an angle \\(\phi\\) around the \\(z\\)-axis:

$$
R_z(\phi) =
\begin{bmatrix}
\cos\phi & -\sin\phi & 0 \\
\sin\phi & \cos\phi & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

Comparing the two matrices shows that the total orientation is:

$$
\phi = \theta_1+\theta_2
$$

The first joint rotates frame \\(\{1\}\\) by \\(\theta_1\\) relative to the base. The second joint then rotates frame \\(\{2\}\\) by another \\(\theta_2\\) relative to frame \\(\{1\}\\). Since both rotations are around parallel \\(z\\)-axes, their angles add.

Use the sliders below to change the robot configuration. The joint frames make it visible that \\(\theta_2\\) is a relative rotation, while the matrices show how the same frame path is evaluated numerically.

{% include planar_fk_demo.html %}

The position equations from the last column can also be understood directly from the drawing. Link 1 contributes \\(a_1\cos\theta_1\\) in the \\(x\\)-direction and \\(a_1\sin\theta_1\\) in the \\(y\\)-direction. Link 2 starts at the end of link 1. Its angle relative to the base is \\(\theta_1+\theta_2\\), so it adds \\(a_2\cos(\theta_1+\theta_2)\\) in the \\(x\\)-direction and \\(a_2\sin(\theta_1+\theta_2)\\) in the \\(y\\)-direction. Adding the contribution from both links gives the same position equations that we read from \\(^{0}T_2\\).

For a simple planar robot, this direct trigonometric approach is manageable. For a three-dimensional robot, the joint axes can point in different directions and each link can introduce both translations and rotations. Homogeneous transformation matrices provide a systematic way to follow these changes from one frame to the next while calculating position and orientation together.

<script src="https://cdn.plot.ly/plotly-4.0.0.min.js" charset="utf-8"></script>
<script src="{{ '/assets/js/kinematics-demos.js' | relative_url }}"></script>
