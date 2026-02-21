# Intro 
In autonomous driving, high-rate state estimation is critical. This repository provides a GINS backend that solves the classic "drift vs. rate" problem.

We use IMU Pre-integration to condense high-rate raw sensor data into discrete motion constraints, which are then optimized alongside GNSS anchors in a Factor Graph. This approach allows the system to remain computationally efficient while simultaneously calibrating accelerometer and gyroscope biases in a globally consistent frame.

## 📚 Credits & References
This work is a custom implementation based on the logic from the [SLAM in Autonomous Driving Book](https://github.com/gaoxiang12/slam_in_autonomous_driving).

# Static IMU INIT 
This module handles the initial calibration of an Inertial Measurement Unit (IMU) when the carrier (robot/vehicle) is stationary. It provides the initial state estimates (Biases) required to begin IMU Pre-integration and Factor Graph optimization.

### IMU Measurement Model
- Acceleration  $\tilde{\mathbf{a}} = \mathbf{R}_{WB}^\top (\mathbf{a}_W - \mathbf{g}_W) + \mathbf{b}_a + \mathbf{\eta}_a$

- Gyro $\tilde{\mathbf{\omega}} = \mathbf{\omega}_{WB} +  \mathbf{b}_g + \mathbf{\eta}_g$

Assuming robot is static $\mathbf{a}_{WB} = 0$,  and $\mathbf{\omega}_{WB} = \mathbf{0}$, fixed attitude [identity] $\mathbf{R}_{WB} = I.$

### Gyro Bias: 
- $\tilde{\mathbf{\omega}} = \mathbf{b}_g + \mathbf{\eta}_g$, thus gyro readings contains only bias $\mathbf{b}_g$.
- mean  $\hat{\mathbf{b}}_g = \frac{1}{N} \sum_{i=1}^{N} \tilde{\mathbf{\omega}}_i$

- its covariance $\mathbf{\Sigma}_g = \frac{1}{N-1} \sum_{i=1}^{N} (\tilde{\mathbf{\omega}}_i - \hat{\mathbf{b}}_g)(\tilde{\mathbf{\omega}}_i - \hat{\mathbf{b}}_g)^T$

### ACC Bias
Acc bias need a bit effort, as it influenced by gravity vector
 $$\tilde{\mathbf{a}} = (- \mathbf{g}_W) + \mathbf{b}_a + \mathbf{\eta}_a$$

Thus : To compute bias, we should compute mean and covariance for the following term :
 $$ \mathbf{b}_a = \tilde{\mathbf{a}} + \mathbf{g}$$

We run the mean/covariance computation twice, FIRSTLY to get gravity vector

$$\mathbf{g}_{body} = -\left( \frac{\bar{\mathbf{a}}}{\|\bar{\mathbf{a}}\|} \right) \cdot 9.81$$

SECONDLY to get bias : 
$$\mathbf{b}_a = \bar{\mathbf{a}} + \mathbf{g}_{body}$$

$$\mathbf{\Sigma}_a = \frac{1}{N-1} \sum_{i=1}^{N} (\tilde{\mathbf{a}}_i + \mathbf{g}_{body} - \mathbf{b}_a)(\tilde{\mathbf{a}}_i + \mathbf{g}_{body} - \mathbf{b}_a)^T$$

# IMU Pre Integration 
Starting from $\Delta R_{ii} = \mathbf{I}$, $\Delta v_{ii} = \mathbf{0}$, and $\Delta p_{ii} = \mathbf{0}$, we accumulate IMU samples at each time step  $k \in [i, j-1]$, assume fix the bias  $\mathbf{b}_{gi}$ , $\mathbf{b}_{ai}$.

## Measurement Update 

Rotation  
$$\Delta \mathbf{R}_{i, k+1} = \Delta \mathbf{R}_{ik} \cdot \text{Exp}((\tilde{\omega}_k - \mathbf{b}_{gi})\Delta t)$$

Velocity 
$$\Delta \mathbf{v}_{i, k+1} = \Delta \mathbf{v}_{ik} + \Delta \mathbf{R}_{ik} (\tilde{\mathbf{a}}_k - \mathbf{b}_{ai})\Delta t$$

Position  
$$\Delta \mathbf{p}_{i, k+1} = \Delta \mathbf{p}_{ik} + \Delta \mathbf{v}_{ik} \Delta t + \frac{1}{2} \Delta \mathbf{R}_{ik} (\tilde{\mathbf{a}}_k - \mathbf{b}_{ai})\Delta t^2$$

### Noise Update 
Rotation
$$\delta \phi_{k+1} \approx \Delta \mathbf{R}_{k,k+1}^\top \delta \phi_k - \mathbf{J}_r \Delta t \delta \mathbf{b}_{gk} - \mathbf{J}_r \Delta t \boldsymbol{\eta}_{gk}$$
$\Delta \mathbf{R}_{k,k+1}^\top$: Propagates the previous rotation error.

$-\mathbf{J}_r \Delta t$: How much a bias shift or noise "tilts" the integrated rotation.

Velocity 
$$\delta \mathbf{v}_{k+1} \approx \delta \mathbf{v}_k - \Delta \mathbf{R}_{ik} (\tilde{\mathbf{a}}_k - \mathbf{b}_{ai})^\wedge \Delta t \delta \phi_k - \Delta \mathbf{R}_{ik} \Delta t \delta \mathbf{b}_{ak} - \Delta \mathbf{R}_{ik} \Delta t \boldsymbol{\eta}_{ak}$$
$- \Delta \mathbf{R}_{ik} (\tilde{\mathbf{a}}_k - \mathbf{b}_{ai})^\wedge \Delta t$: This is the "tilt" effect. If your rotation is wrong by $\delta \phi$, the gravity/acceleration vector is projected into the wrong direction.
$- \Delta \mathbf{R}_{ik} \Delta t$: Direct impact of accelerometer bias and noise.


Position 
$$\delta \mathbf{p}_{k+1} \approx \delta \mathbf{p}_k + \delta \mathbf{v}_k \Delta t - \frac{1}{2} \Delta \mathbf{R}_{ik} (\tilde{\mathbf{a}}_k - \mathbf{b}_{ai})^\wedge \Delta t^2 \delta \phi_k - \frac{1}{2} \Delta \mathbf{R}_{ik} \Delta t^2 \delta \mathbf{b}_{ak} - \frac{1}{2} \Delta \mathbf{R}_{ik} \Delta t^2 \boldsymbol{\eta}_{ak}$$


Stack Noise  : $$\delta \boldsymbol{\eta}_{k+1} = \mathbf{A} \delta \boldsymbol{\eta}_k + \mathbf{B} \mathbf{w}$$ 

$\delta \boldsymbol{\eta}_{k} = [\delta \phi_k, \delta \mathbf{v}_k, \delta \mathbf{p}_k], \ and \ \ \mathbf{w} = [\boldsymbol{\eta}_{gk}, \boldsymbol{\eta}_{ak}]$

$$\mathbf{A}_k = 
\begin{bmatrix}
\Delta \mathbf{R}_{k,k+1}^\top & \mathbf{0} & \mathbf{0}  \\
-\Delta \mathbf{R}_{ik} (\tilde{\mathbf{a}}_k - \mathbf{b}_{ai})^\wedge \Delta t & \mathbf{I} & \mathbf{0} & \\
-\frac{1}{2} \Delta \mathbf{R}_{ik} (\tilde{\mathbf{a}}_k - \mathbf{b}_{ai})^\wedge \Delta t^2 & \mathbf{I} \Delta t & \mathbf{I} 
\end{bmatrix}$$

$$\mathbf{B}_k = 
\begin{bmatrix}
\mathbf{J}_r \Delta t & \mathbf{0} \\
\mathbf{0} & \Delta \mathbf{R}_{ik} \Delta t \\
\mathbf{0} & \frac{1}{2} \Delta \mathbf{R}_{ik} \Delta t^2 \\
\end{bmatrix}$$

### Update Covariance 
$$\mathbf{\Sigma}_{i, k+1} = \mathbf{A}_k \mathbf{\Sigma}_{ik} \mathbf{A}_k^\top + \mathbf{B}_k \text{Cov}(\mathbf{w}_k) \mathbf{B}_k^\top$$


## Covariance Update Summary 
To propagate covariance, we linearize the system such that $\delta \boldsymbol{\eta}_{k+1} = \mathbf{A}_k \delta \boldsymbol{\eta}_k + \mathbf{B}_k \mathbf{w}_k$.

| State Error | Source of Error | Coefficient (in $A_k$) | Noise Impact (in $B_k$) |
| :--- | :--- | :--- | :--- |
| **Rotation** ($\delta \phi$) | Prev Rotation, Gyro Bias | $\Delta \mathbf{R}_{k,k+1}^\top$, $-\mathbf{J}_r \Delta t$ | $-\mathbf{J}_r \Delta t$ |
| **Velocity** ($\delta \mathbf{v}$) | Prev Velocity, Rotation Error, Accel Bias | $\mathbf{I}$, $-\Delta \mathbf{R}_{ik} (\mathbf{corr})^\wedge \Delta t$, $-\Delta \mathbf{R}_{ik} \Delta t$ | $-\Delta \mathbf{R}_{ik} \Delta t$ |
| **Position** ($\delta \mathbf{p}$) | Prev Pos, Prev Vel, Rotation Error, Accel Bias | $\mathbf{I}$, $\mathbf{I}\Delta t$, $-\frac{1}{2}\Delta \mathbf{R}_{ik} (\mathbf{a}_{corr})^\wedge \Delta t^2$ | $-\frac{1}{2}\Delta \mathbf{R}_{ik} \Delta t^2$ |

## Bias Update 
Previous discussions assumed constant IMU biases at time i for computational convenience

Rotation w.r.t Gyro Bias:

$\mathbf{J}_{R, bg}^{k+1} = \Delta \mathbf{R}_{k, k+1}^\top \mathbf{J}_{R, bg}^k - \mathbf{J}_r \Delta t$

Velocity w.r.t Accel Bias:

$\mathbf{J}_{v, ba}^{k+1} = \mathbf{J}_{v, ba}^k - \Delta \mathbf{R}_{ik} \Delta t$ 

Velocity w.r.t Gyro Bias:

$\mathbf{J}_{v, bg}^{k+1} = \mathbf{J}_{v, bg}^k - \Delta \mathbf{R}_{ik} (\tilde{a}_k - \mathbf{b}_{ai})^\wedge \mathbf{J}_{R, bg}^k \Delta t$

Position w.r.t Accel Bias:

$\mathbf{J}_{p, ba}^{k+1} = \mathbf{J}_{p, ba}^k + \mathbf{J}_{v, ba}^k \Delta t - \frac{1}{2} \Delta \mathbf{R}_{ik} \Delta t^2$

Position w.r.t Gyro Bias:

$\mathbf{J}_{p, bg}^{k+1} = \mathbf{J}_{p, bg}^k + \mathbf{J}_{v, bg}^k \Delta t - \frac{1}{2} \Delta \mathbf{R}_{ik} (\tilde{a}_k - \mathbf{b}_{ai})^\wedge \mathbf{J}_{R, bg}^k \Delta t^2$


## Implementation Tips
- $\mathbf{J}_r$: The Right Jacobian of $SO(3)$ is often approximated as $\mathbf{I}$ 

- for very small $\Delta t$, but for high precision, use the full formula.

# Pre-Integration Factor Graph 
## Residual
Rotation

$$\mathbf{r}_{\Delta R} = \text{Log} \left( \Delta \tilde{\mathbf{R}}_{ij}(\mathbf{b}_{gi})^\top \mathbf{R}_i^\top \mathbf{R}_j \right)$$

Velocity

$$\mathbf{r}_{\Delta v} = \mathbf{R}_i^\top (\mathbf{v}_j - \mathbf{v}_i - \mathbf{g} \Delta t_{ij}) - \Delta \tilde{\mathbf{v}}_{ij}(\mathbf{b}_{gi}, \mathbf{b}_{ai})$$

Position

$$\mathbf{r}_{\Delta p} = \mathbf{R}_i^\top (\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i \Delta t_{ij} - \frac{1}{2} \mathbf{g} \Delta t_{ij}^2) - \Delta \tilde{\mathbf{p}}_{ij}(\mathbf{b}_{gi}, \mathbf{b}_{ai})$$

## Jacobian of Residual w.r.t Each Vertex-Dim  
### Rotation Residual Jacobians:

#### With respect to Vertex $i$:

Rotation ($\delta \phi_i$):
$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \delta \phi_i} = -\mathcal{J}_r^{-1}(\mathbf{r}_{\Delta R}) \mathbf{R}_j^\top \mathbf{R}_i$$

Gyro Bias ($\delta \mathbf{b}_{gi}$)
$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \delta \mathbf{b}_{gi}} = -\mathcal{J}_r^{-1}(\mathbf{r}_{\Delta R}) \text{Exp}(\mathbf{r}_{\Delta R})^\top \mathbf{J}_{R,bg}$$

#### With respect to Vertex $j$:
$$\frac{\partial \mathbf{r}_{\Delta R}}{\partial \delta \phi_j} = \mathcal{J}_r^{-1}(\mathbf{r}_{\Delta R})$$

### Velocity Residual Jacobians
#### With respect to Vertex i:
Rotation ($\delta \phi_i$):
$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \delta \phi_i} = \left( \mathbf{R}_i^\top (\mathbf{v}_j - \mathbf{v}_i - \mathbf{g} \Delta t) \right)^\wedge$$

Velocity ($\delta \mathbf{v}_i$): 
$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \delta \mathbf{v}_i} = -\mathbf{R}_i^\top$$

Gyro Bias ($\delta \mathbf{b}_{gi}$):
 $$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \delta \mathbf{b}_{gi}} = -\mathbf{J}_{v,bg}$$

Accel Bias ($\delta \mathbf{b}_{ai}$): 
$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \delta \mathbf{b}_{ai}} = -\mathbf{J}_{v,ba}$$

#### With respect to Vertex $j$
Velocity ($\delta \mathbf{v}_j$): 
$$\frac{\partial \mathbf{r}_{\Delta v}}{\partial \delta \mathbf{v}_j} = \mathbf{R}_i^\top$$

### Position Residual Jacobians ($r_{\Delta p}$)
#### With respect to Vertex $i$:
Rotation ($\delta \phi_i$): 
$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta \phi_i} = \left( \mathbf{R}_i^\top (\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i \Delta t - \frac{1}{2} \mathbf{g} \Delta t^2) \right)^\wedge$$

Position ($\delta \mathbf{p}_i$): 
$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta \mathbf{p}_i} = -\mathbf{R}_i^\top$$

Velocity ($\delta \mathbf{v}_i$): 
$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta \mathbf{v}_i} = -\mathbf{R}_i^\top \Delta t$$

Gyro Bias ($\delta \mathbf{b}_{gi}$): 
$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta \mathbf{b}_{gi}} = -\mathbf{J}_{p,bg}$$

Accel Bias ($\delta \mathbf{b}_{ai}$): 
$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta \mathbf{b}_{ai}} = -\mathbf{J}_{p,ba}$$

#### With respect to Vertex $j$:
Position ($\delta \mathbf{p}_j$): 
$$\frac{\partial \mathbf{r}_{\Delta p}}{\partial \delta \mathbf{p}_j} = \mathbf{R}_i^\top$$

### Bias Random Walk Jacobians
If you are also optimizing the change in bias between frames ($r_{bg} = \mathbf{b}_{gj} - \mathbf{b}_{gi}$ and $r_{ba} = \mathbf{b}_{aj} - \mathbf{b}_{ai}$):

$\frac{\partial \mathbf{r}_{bg}}{\partial \mathbf{b}_{gj}} = \mathbf{I}$, $\frac{\partial \mathbf{r}_{bg}}{\partial \mathbf{b}_{gi}} = -\mathbf{I}$

$\frac{\partial \mathbf{r}_{ba}}{\partial \mathbf{b}_{aj}} = \mathbf{I}$, $\frac{\partial \mathbf{r}_{ba}}{\partial \mathbf{b}_{ai}} = -\mathbf{I}$


### Implementation Notes:
Skew-Symmetric $(\dots)^\wedge$: These terms represent how a small rotation of the starting frame $i$ shifts the prediction of the end frame in global coordinates.

$\mathcal{J}_r^{-1}$: This is the inverse right Jacobian of $SO(3)$. If your residuals are small, using $\mathcal{J}_r^{-1} \approx \mathbf{I}$ is a common and usually safe optimization in many solvers.

Pre-integration Jacobians: $\mathbf{J}_{R,bg}$, $\mathbf{J}_{v,bg}$, etc., are the ones you computed during the IMU integration loop.

## GINS Execution Logic Summary

| Step | Phase | Action | 
| :--- | :--- | :--- |
| A| Init | Set $P, V$ to zero; $R$ to Identity; GNSS buffers to null.
| B | Static Calib| Average raw IMU data. Calculate initial $\mathbf{b}_g, \mathbf{b}_a$.
| C | Pre-integration Init | Initialize the Preintegration object with calibrated biases.
| D | Pre-integration Acc | Accumulate IMU  
| E | Wait For GNSS | No: Keep calling Preint.integrate(). Yes: Move to Step E.
| F| First GNSS | If 1st GNSS, create the Prior Vertex (starting point) and loop back.
|G | Prediction| On 2nd GNSS, use Preint.predict() to create an initial guess for the new vertex.
| H | Factor Graph | Add Vertex i, Vertex j, IMU Edge, and GNSS Edge.
| M | Optimization | Run the solver to find the best R,P,V and Biases.
| N | Update | Update Bias based on Optimized Values.
| O | Reset | Reset IMU Pre-integration Object. 
| L | Update | Update this/lastframe, and this/last GNSS. 