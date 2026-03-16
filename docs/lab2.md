Lab 2: Implementing Simplified Kalman Filter for Vertical State Estimation
==============

***EE4308 Autonomous Robot Systems***

***AY25/26 Sem 2***

**&copy; Lai Yan Kai, National University of Singapore**


# Table of Contents

[1&emsp;Administrative Matters](#1administrative-matters)

&emsp;[1.1&emsp;Submittables](#11submittables)

&emsp;&emsp;[1.1.1&emsp;Answers to L2 Questions](#111answers-to-l2-questions)

&emsp;&emsp;[1.1.2&emsp;Zip file to L2](#112zip-file-to-l2)

&emsp;[1.2&emsp;Download, Build, and Run Files](#12download-build-and-run-files)

[2&emsp;Implementing $z$-axis Estimation](#2implementing--axis-estimation)

&emsp;[2.1&emsp;Relevant Files for Estimator Node](#21relevant-files-for-estimator-node)

&emsp;[2.2&emsp;Prediction using Kinematic Equations](#22prediction-using-kinematic-equations)

&emsp;&emsp;[2.2.1&emsp;Implement `Estimator::callbackSubIMU_()`](#221implement-estimatorcallbacksubimu_)

&emsp;[2.3&emsp;Correction](#23correction)

&emsp;&emsp;[2.3.1&emsp;Implement `Estimator::callbackSubSonar_()`](#231implement-estimatorcallbacksubsonar_)

[3&emsp;Questions](#3questions)

# 1&emsp;Administrative Matters
Take note of the deadline and install all relevant software before proceeding to the lab. This section and all subsequent sections assume that instructions in the installation guide were followed.

| Overview | Description |
| -- | -- |
| **Effort** | Individual work. |
| **Deadline** | W10 Fri, 23:59. | 
| **Submission** | Submit answers to **L2 Questions** on Canvas, and a zip file to **L2** on Canvas. More details below. |
| **Software** | ROS2 Jazzy on Ubuntu 24.04 LTS. |
| **Lab Computers**| If installation is not possible after trying, a lab computer can be loaned. Please email the TA / GA / Lecturer for a computer, based on instructions on Canvas. |

## 1.1&emsp;Submittables

### 1.1.1&emsp;Answers to L2 Questions
Submit the answers for the three questions from [3 Questions](#3questions) on the Canvas Quiz.

### 1.1.2&emsp;Zip file to L2
Submit a zip file containing the code to Canvas. To avoid penalties, 
1. Named the zip as `l2_<matric>.zip` where `<matric>` is your student number in **lowercase** (e.g. `l2_a0987654n.zip`). 
2. Rename the workspace folder to `l2_<matric>`.
3. Ensure that the zip file have the following structure. Do not zip other files or directories.

<table><tbody><tr><td>
    <details open>
        <summary><code>l2_a0987654n.zip</code>&emsp;The zip file.</summary>
        <dl>
            <dd><details open>
                <summary><code>l2_a0987654n/</code>&emsp;The renamed workspace directory.</summary>
                <dl>
                    <dd><details open> 
                        <summary><code>src/</code>&emsp;Contains packages.</summary>
                        <dl>
                            <dd><details open>
                                <summary><code>ee4308_bringup/</code>&emsp;Package containing scripts that start the projects.</summary>
                                <dl>
                                    <dd><details open>
                                        <summary><code>params/</code></summary>
                                        <dl>
                                            <dd><code>proj2.yaml</code>&emsp;Contains adjustable parameter values.</dd>
                                        </dl>
                                    </details></dd>
                                </dl>
                            </details></dd>
                            <dd><details open>
                                <summary><code>ee4308_drone/</code>&emsp;Package implementing the drone's estimator</summary>
                                <dl>
                                    <dd><details open>
                                        <summary><code>include/</code>&emsp;Directory for <code>.hpp</code> header files.</summary>
                                        <dl>
                                            <dd><details open>
                                                <summary><code>ee4308_drone/</code></summary>
                                                <dl>
                                                    <dd><code>estimator.hpp</code>&emsp;Contains code declarations for the estimator.</dd>
                                                </dl>
                                            </details></dd>
                                        </dl>
                                    </details></dd>
                                    <dd><details open>
                                        <summary><code>src/</code>&emsp;Directory for <code>.cpp</code> files.</summary>
                                        <dl>
                                            <dd><code>estimator.cpp</code>&emsp;Contains code definitions for the estimator. To complete <code>callbackSubImu_()</code> and <code>callbackSubSonar_()</code>.</dd>
                                        </dl>
                                    </details></dd>
                                </dl>
                            </details></dd>
                        </dl>
                    </details></dd>
                </dl>
            </details></dd>
        </dl>
    </details>
</td></tr></tbody></table>


## 1.2&emsp;Download, Build, and Run Files

1. Clone the project 2 branch of the EE4308 repository:
    ```bash
    cd ~
    git clone -b 2520-proj2 https://github.com/LaiYanKai/ee4308
    ```
2. Build the repository:
    ```bash
    cd ~/ee4308
    colcon build --symlink-install
    ```

3. Source the build to tell the terminal where the built and installed executables for the project are:
    ```bash
    cd ~/ee4308
    source install/setup.bash
    ```
    The `setup.bash` file only needs to be run once everytime a terminal is newly opened. If the `setup.bash` file is not run at least once, the project cannot be run.

4. To run the code:
    - If using VirtualBox,
        ```bash
        ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=True
        ```
    - If dual booted and using Ubuntu natively,
        ```bash
        ros2 launch ee4308_bringup proj2_sim.launch.py
        ```

5. If using a lab computer or a computer connected to a shared network, ensure that no one is publishing into your `ROS_DOMAIN_ID`. 
The check can be done by sending the command `ros2 topic list`. 
If someone is publishing into your channel, more than two topics will be listed.

6. The turtlebot will begin to run, and the drone will remain stationary. The simulation can be stopped by sending `Ctrl+C` on the terminal.

# 2&emsp;Implementing $z$-axis Estimation

In this lab, we focus on deriving the kinematic equations for estimating the drone states (position and velocity) in the world frame, and fitting the equations into the Kalman filter to estimate the states.

## 2.1&emsp;Relevant Files for Estimator Node

| Name | Path | Description |
| --- | --- | --- |
| Estimator header file | `ee4308/ src/ ee4308_drone/ include/ ee4308_drone/ estimator.hpp` | Declare functions and class variables here. Modifications will require re-building with `colcon build`. |
| Estimator source file | `ee4308/ src/ ee4308_drone/ src/ estimator.cpp` | Define the functions here. Modifications will require re-building. |
| Parameter file | `ee4308/ src/ ee4308_bringup/ params/ proj2.yaml` | Contains parameter values that are fed into the programs when launched. Modifications do not require re-building. |


## 2.2&emsp;Prediction using Kinematic Equations
Consider an object moving at constant acceleration $a$ over a period $\Delta t$. Given an initial displacement $s_0$, final displacement $s$, initial velocity $u$, final velocity $v$, the equations of motion are:
```math
\begin{align}
    s &= s_0 + u\Delta t + \frac{1}{2} a (\Delta t)^2 \\
    v &= u + a \Delta t
\end{align}
```

We next adapt these equations of motion to the project's motion model of a drone. Between two time steps $k-1$ and $k$, the vertical position $z$ and velocity $\dot{z}$ of the drone can be estimated by assuming a constant acceleration over a small time interval $\Delta t$:
```math
\begin{align}
    z_{k|k-1} &= z_{k-1|k-1} + \dot{z}_{k-1|k-1} \Delta t + \frac{1}{2}a_{z,k}(\Delta t)^2 \\
    \dot{z}_{k|k-1} &= \dot{z}_{k-1|k-1} + a_{z,k}\Delta t
\end{align}
```
The subscript $k|k-1$ indicates the predicted (the current time step's *prior*) states, while $k-1|k-1$ indicates the previous (the previous time step's *posterior*) states. The rest are self-explanatory.

By combining these equations of motions into matrices, we arrive at the state prediction equations of the Kalman Filter for our simplified motion model:
```math
\begin{align}
    \begin{bmatrix} z_{k|k-1} \\ \dot{z}_{k|k-1} \end{bmatrix}
        &= \mathbf{F}_{z,k} \begin{bmatrix} z_{k-1|k-1} \\ \dot{z}_{k-1|k-1} \end{bmatrix}
        + \mathbf{W}_{z,k}a_{z,k} \\
    \hat{\mathbf{X}}_{z,k|k-1} 
        &= \mathbf{F}_{z,k} \hat{\mathbf{X}}_{z,k-1|k-1} 
        + \mathbf{W}_{z,k}\mathbf{U}_{z,k} 
\end{align}
```
where $\mathbf{F}_ {z,k}$ is a $2\times 2$ matrix and $\mathbf{W}_ {z,k}$ is a $2\times 1$ vector to be determined. $\mathbf{U}_ {z,k}$ is the input, which in this case is just the scalar acceleration $a_{z,k}$.
As can be inferred from the previous paragraph, $\hat{\mathbf{X}}_ {z,k|k-1}$ contains the current *prior* states, while $\hat{\mathbf{X}}_{z,k-1|k-1}$ contains the previous *posterior* states for the $z$-axis.

The Kalman filter models each state as a Gaussian variable, which has a mean described in $\hat{\mathbf{X}}$ and covariances described in $\mathbf{P}$. 
*In addition*, let $\mathbf{Q}$ be the covariance of the input, which in this case is the variance of $a$ (think of $a$ as a noisy, uncertain value).
When the mean of a Gaussian variable is changed (updated) by some amount, the variable's covariances must be updated by a related amount:
```math
\begin{equation}
    \mathbf{P}_{z,k|k-1} = \mathbf{F}_{z,k} \mathbf{P}_{z,k-1|k-1} \mathbf{F}_{z,k}^\top 
        + \mathbf{W}_{z,k} \mathbf{Q}_z, \mathbf{W}_{z,k}^\top 
\end{equation}
```
To understand the amount that is being transformed,
we first see the term $\mathbf{F}\hat{\mathbf{X}}$ as *multiplying* or *transforming* $\hat{\mathbf{X}}$ by an *amount* $\mathbf{F}$. This will result in the covariance $\mathbf{P}$ being *multiplied* or *transformed* in the manner that is $\mathbf{F}\mathbf{P}\mathbf{F}^\top$. An easier way to understand this is to examine what happens when a scalar $f$ is multiplied to a scalar gaussian variable $x$ (e.g. multiplying to a list of noisy data values) &mdash;  its standard deviation $\sigma$ is scaled by $f$ to become $f\sigma$, and as such its variance $\sigma^2$ is scaled by $f^2$ to become $f^2 \sigma^2$.

### 2.2.1&emsp;Implement `Estimator::callbackSubIMU_()`
Using the prediction equations, implement the Kalman Filter's prediction stage for the drone's $z$ position and velocity in the world frame. The equations are:

```math
\begin{align}
    \hat{\mathbf{X}}_{z,k|k-1} 
        &= \mathbf{F}_{z,k} \hat{\mathbf{X}}_{z,k-1|k-1} 
        + \mathbf{W}_{z,k}\mathbf{U}_{z,k} \\
    \mathbf{P}_{z,k|k-1} &= \mathbf{F}_{z,k} \mathbf{P}_{z,k-1|k-1} \mathbf{F}_{z,k}^\top 
        + \mathbf{W}_{z,k} \mathbf{Q}_z, \mathbf{W}_{z,k}^\top
\end{align} 
```

The prediction is implemented in the subscriber callback for the IMU sensor, which is run everytime the `Estimator` node receives IMU sensor data.

The gravitational acceleration $g$ will be measured by the IMU, and will alter the measured acceleration.
In the simplified motion model, **roll and pitch are ignored**, and the $z$-axis of the drone frame can be assumed to *coincide* with the world frame's $z$-axis. 
As such, $g$ can be accounted for by simply adding or subtracting to the measured value when finding the acceleration $a_{z,k}$ of the drone:
```math
\begin{equation}
    a_{z,k} = u_{z,k} + ? 
\end{equation}
```
Determine if the expression above is an addition or subtraction of $g$ by verifying it in simulation.

In addition, keep in mind that $\mathbf{U}_ {z,k} = a_{z,k}$ and $\mathbf{Q}_ z = \sigma_{imu,z}^2$ are scalars. The sizes of the other matrices can be found from the previous section.

The following variables / functions should be used. **Those listed as parameters can only be tuned from `proj2.yaml`**.
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `msg` | `sensor_msgs::msg::Imu` | No | To read the linear acceleration force vector from `msg.linear_acceleration`. |
| `GRAVITY` | `double` | No | The acceleration due to gravity. |
| `dt` | `double` | No | $\Delta t$. To read the elapsed time from the last prediction. |
| `var_imu_z_` | `double` | Yes | $\sigma_{imu,z}^2$. To read the variance of IMU linear accelearation measurement along the drone's $z$-axis. |
| `Xz_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_z$. To store the drone's predicted $z$ position and velocity in the world frame. `Xz_(0)` contains the position. `Xz_(1)`  contains the velocity. Accessing these properties are not required for the matrix calculations here. |
| `Pz_` | `Eigen::Matrix2d` | No | $\mathbf{P}_z$. To store the predicted covariance matrix for the states in $\mathbf{\hat{X}}_z$. |

## 2.3&emsp;Correction
Kalman Filter correction minimizes the error between a sensor observation $\mathbf{Y}$ and the current state $\hat{\mathbf{X}}$, to get the best estimate of a new state that compromises both $\mathbf{Y}$ and $\mathbf{X}$ knowing that both are *flawed* (i.e. corrupted with noise). 
The filter correction assumes that the observation and state are Gaussian variables.

```math
\begin{align}
  \mathbf{K}_k &= \mathbf{P}_{k|k-1} \mathbf{H}_k^\top
    \left(
      \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^\top
      + \mathbf{V}_k \mathbf{R}_k \mathbf{V}_k^\top
    \right)^{-1} \\
  \mathbf{\hat{X}}_{k|k} &= \mathbf{\hat{X}}_{k|k-1} + \mathbf{K}_k
    \left(
      \mathbf{Y}_k - \mathbf{H}_k \mathbf{\hat{X}}_{k|k-1}
    \right) \\
  \mathbf{P}_{k|k} &= \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}
\end{align}
```

Here, $\mathbf{H}$ projects $\hat{\mathbf{X}}$ into the sensor frame. 
The term $(\mathbf{Y} - \mathbf{H}\hat{\mathbf{X}})$ is also called the *innovation*, which is the discrepancy between the observation and what is expected from the current state.

$\mathbf{V}$ is meant to project the sensor noise $\mathbf{R}$ into the sensor frame. 
Together, $\mathbf{V}\mathbf{R}\mathbf{V}^\top$ can be interpreted as the measurement noise. The term $\mathbf{H}\mathbf{P}\mathbf{H}^\top +\mathbf{V}\mathbf{R}\mathbf{V}^\top$ is also known as the *innovation covariance*. 
$\mathbf{K}$ is the Kalman gain, which determines how much the current states in $\hat{\mathbf{X}}$ and their covariances in $\mathbf{P}$ should be updated based on the observation.

An easier way to interpret the update equations is to see the Kalman gain as a weight &mdash; if all the matrices are scalar, the update equations are essentially just weighted sums.

### 2.3.1&emsp;Implement `Estimator::callbackSubSonar_()`
Here, we implement the correction for the $z$ states when a sonar message is received. 
The sonar points downward from the drone, and provides a good measurement of the drone's height from the closest horizontal surface.

The following variables / functions should be used. **Those listed as parameters can only be tuned from `proj2.yaml`**.
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `msg` | `sensor_msgs::msg::Range` | No | To read the sonar measurement from `msg.range`. |
| `var_sonar_` | `double` | Yes | $\sigma_{snr,z}^2$. To read the variance of sonar measurements along the world's $z$-axis. |
| `Ysonar_` | `double` | No | $z_{snr}$. To store the sonar measurement. |
| `Xz_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_z$. To store the drone's corrected $z$ osition and velocity in the world frame. |
| `Pz_` | `Eigen::Matrix2d` | No | $\mathbf{P}_z$. To store the corrected covariance matrix for the states in $\mathbf{\hat{X}}_z$. |

For lab 2, the relevant matrices are
```math
\begin{align}
  \mathbf{Y}_{snr,z,k} &= z_{snr} = z_{k|k-1}+ \varepsilon_{snr,k}\\

  \mathbf{H}_{snr,z,k} &= \frac{\partial \mathbf{Y}_{snr,z,k}}{\partial \mathbf{\hat{X}}_{z,k|k-1}}
    = \begin{bmatrix}
      \frac{\partial z_{snr}}{\partial z_{k|k-1}} & \frac{\partial z_{snr}}{\partial \dot{z}_{k|k-1}}
    \end{bmatrix}
    = \begin{bmatrix}1 & 0 \end{bmatrix}\\

  \mathbf{V}_{snr,z,k} &= 1 \\

  \mathbf{R}_{snr,z,k} &= \sigma^2_{snr,z}
\end{align}
```
You may face some difficulties with the matrix multiplications in `Eigen` if you are unfamiliar with it. 
Some calculations can be in scalar form where `double` can be used, particularly for the innovation covariance. 
As such, `Pz_(0, 0)` can be used to extract the element from the first row and first column if it is easier for you to code this way. 
Otherwise, make use of the `.transpose()` or `.inverse()` methods of each `Eigen` matrix. See [tips.md](tips.md).

In addition, please find a way to determine the value of $\sigma_{snr,z}^2$. 

# 3&emsp;Questions
On Canvas **L2 Questions** Quiz,

1. Determine the expression for $a_{z,k}$.

2. What are the matrix / vector sizes for $\mathbf{H}$, the innovation, the innovation covariance, and $\mathbf{K}$?

3. In order to determine $\sigma_{snr,z}^2$, one way is to make the drone stationary and collect about 100 samples from the sonar sensor and determine the variance from the samples. 
However, the drone may slowly drift upwards. 
In this situation, concisely describe your process of determining $\sigma_{snr,z}^2$ by using a best fit line and further tuning. An essay is **not** expected for this answer.


[//]: # (What! Is your name? ... What! Is your quest? ... What! is the air-speed velocity of an unladen swallow? -- Bridgekeeper, Monty Python and the Holy Grail) 
