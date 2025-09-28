# Train Speed Profile Optimization Based on Dynamic Programming




## Algorithm Introduction

This project aims to find the optimal energy-saving speed profile for a single train operating on a single track section. The problem is formulated as an optimal control problem and solved using Dynamic Programming (DP).

### 1. Train Dynamics Model

The movement of the train is governed by fundamental principles of physics. The model considers the primary forces acting on the train: tractive force, braking force, and running resistance.

#### Tractive Force

The tractive force $f_t$ is determined by the traction control coefficient $\mu_t \in [0,1]$ and the maximum available tractive force $F_{max}(v)$, which is a function of the train's speed $v$.

$$
f_t = \mu_t F_{max}(v)
$$

$$
F_{max}(v) = \begin{cases} 
203 & 0 \leq v \leq 51.5 \, \text{km/h} \\
-0.002032v^3 + 0.4928v^2 - 42.13v + 1342 & 51.5 < v \leq 80 \, \text{km/h}
\end{cases}
$$

Here, $F_{max}(v)$ is in kN.

#### Braking Force

Similarly, the braking force $f_d$ is determined by the braking control coefficient $\mu_d \in [0,1]$ and the maximum available braking force $B_{max}(v)$.

$$
f_d = \mu_d B_{max}(v)
$$

$$
B_{max}(v) = \begin{cases} 
166 & 0 \leq v \leq 77 \, \text{km/h} \\
0.1343v^2 - 25.07v + 1300 & 77 < v \leq 80 \, \text{km/h}
\end{cases}
$$

Here, $B_{max}(v)$ is in kN.

#### Basic Running Resistance

The basic running resistance is modeled using the Davis equation, a quadratic function of speed $v$ (in km/h).

$$
w_0(v) = A + Bv + Cv^2
$$

The total basic resistance force $W_0(v)$ (in kN) is then:

$$
W_0(v) = w_0(v) \cdot M \cdot g \cdot 10^{-3}
$$

where $w_0(v)$ is the unit basic resistance (N/kN), $M$ is the train mass (t), and $A, B, C$ are empirical coefficients.

#### Additional Line Resistance

This resistance arises from track gradients and curves. The total additional unit resistance $w_j(s)$ at position $s$ is the sum of gradient resistance $w_i(s)$ and curve resistance $w_c(s)$.

$$
w_j(s) = w_i(s) + w_c(s)
$$

$$
w_i(s) = i 
$$

$$
w_c(s) = \frac{c}{R}
$$

where $i$ is the gradient in per mille (‰), $R$ is the curve radius (m), and $c$ is an empirical constant (typically 600). The total additional resistance force $W_j(s)$ (in kN) is:

$$
W_j(s) = w_j(s) \cdot M \cdot g \cdot 10^{-3}
$$

### 2. Optimization Model Formulation

The problem of finding the most energy-efficient speed profile is structured as an optimal control problem.

**a. Objective Function:**

The primary goal is to minimize the net energy consumption, $J$. This is calculated as the total tractive energy consumed, adjusted for efficiency, minus the energy recovered through regenerative braking.

$$
\min J = \int_{s_{\text{start}}}^{s_{\text{end}}} \left(\frac{f_t(s) \cdot 10^3}{\eta} - \alpha \eta f_d(s) \cdot 10^3\right) ds
$$

Where $s_{start}$ and $s_{end}$ are the start and end positions, $\eta$ is the overall electromechanical efficiency, $\alpha$ is the regenerative braking utilization rate, and $f_t(s)$ and $f_d(s)$ are the tractive and braking forces as functions of position.

**b. State Equations:**

The train's motion is described by two fundamental differential equations. The first relates the change in velocity to the net forces acting on the train, and the second relates the change in time to the train's velocity.

$$
M(1 + \gamma) \frac{dv(s)}{ds} = \frac{f_t(s) - f_d(s) - W_0(v(s)) - W_j(s)}{v(s)}
$$

$$
\frac{dt}{ds} = \frac{1}{v(s)}
$$

**c. Speed Limit Constraint:**

The train's speed $v(s)$ must not exceed the permissible speed limit of the track $v_{\text{lim}}(s)$ at any position.

$$
0 \leq v(s) \leq v_{\text{lim}}(s)
$$

**d. Traction and Braking Force Constraints:**

The applied tractive force $f_t(s)$ and braking force $f_d(s)$ are limited by the train's physical capabilities, which are functions of its current speed.

$$
0 \leq f_t(s) \leq F_{\max}(v(s))
$$

$$
0 \leq f_d(s) \leq B_{\max}(v(s))
$$

**e. Acceleration Constraint:**

To ensure passenger comfort, the acceleration $a$ is constrained within acceptable limits.

$$
-1 \leq a = \frac{f_t(s) - f_d(s) - W_0(v(s)) - W_j(s)}{M(1 + \gamma)} \leq 1 \quad (\text{m/s}^2)
$$

**f. Travel Time Constraint:**

The total travel time $T$ must adhere to the operational schedule, staying within a specified tolerance $\delta$ of the scheduled time $T_{set}$.

$$
T_{set} - \delta \leq T = \int_{s_{start}}^{s_{end}} \frac{1}{v(s)} ds \leq T_{set} + \delta
$$

**g. Endpoint Constraints:**

The train must start from a standstill at the initial station and come to a complete stop at the terminal station.

$$
v(s_{start}) = 0
$$

$$
v(s_{end}) = 0
$$


### 3. Solving the Optimization Model Based on Dynamic Programming

The continuous optimal control problem is discretized to be solved with Dynamic Programming (DP). The route is divided into $N$ segments of length $\Delta s_k$, creating $N+1$ decision stages. At each stage, the speed is also discretized into multiples of a unit speed $\Delta v$.

A **state** $x_{k,i}$ is defined by the train's position and speed, $(s_k, v_{k,i})$. A **decision** $u_{k,i}$ is the choice of action (e.g., traction, coasting, braking) taken at a given state, which determines the transition to the next state. The **state transition** is governed by the equation:

$$
x_{k+1, j} = h(x_{k, i}, u_{k, i})
$$

To incorporate the travel time constraint into the energy minimization objective, a penalty method is used. The cost of transitioning from state $x_{k,i}$ to state $x_{k+1,j}$, denoted as $C_{k,i}^{k+1,j}$, is a weighted sum of energy consumption $e_k$ and travel time $T_k$.

$$
C_{k,i}^{k+1,j} = \begin{cases} e_k + \lambda T_k = \left(\frac{f_t(s_k) \cdot 10^3}{\eta} - \alpha \eta f_d(s_k) \cdot 10^3\right) \cdot \Delta s_k + \lambda \frac{2 \Delta s_k}{v_{k,i} + v_{k+1,j}} \\ +\infty \quad (\text{if transition violates constraints}) \end{cases}
$$

The required force $f(s_k)$ for this transition is calculated from the change in kinetic energy and resistive forces:

$$
f(s_k) = M(1 + \gamma) \cdot a + W_0(v_{k,i}) + W_j(s_k) = \begin{cases} f_t(s_k), & \text{if } f(s_k) \geq 0 \\ f_d(s_k), & \text{if } f(s_k) \leq 0 \end{cases}
$$

where the acceleration $a$ and discretized speeds are given by:

$$
a = \frac{v_{k+1,j}^2 - v_{k,i}^2}{2\Delta s_k}
$$
$$
v_{k,i} = (i - 1) \cdot \Delta v, \quad v_{k+1,j} = (j - 1) \cdot \Delta v
$$

Here, $\lambda$ is the time penalty weight coefficient. Any transition that violates physical or operational constraints is assigned an infinite cost.

The problem possesses the memoryless property (optimal substructure), which is fundamental to DP. This means the optimal policy from any state onward is independent of the path taken to reach that state. We can therefore apply backward dynamic programming.

Let $(C_{k,i}^{N+1,1})^*$ be the optimal (minimum) cumulative cost from state $x_{k,i}$ to the final terminal state. The solution is found via the following Bellman equation, which defines the recursive relationship between stages:

$$
(C_{k,i}^{N+1,1})^* = \min_j \{C_{k,i}^{k+1,j} + (C_{k+1,j}^{N+1,1})^*\}
$$

This equation states that the optimal cost from stage $k$ is the minimum of the costs of taking any valid action, which consists of the immediate transition cost plus the optimal cost from the resulting next state. By solving this equation iteratively backward from the final stage to the first, we can find the optimal path.

### 4.The DP Algorithm for Energy-Saving Optimization

Based on the DP formulation above, the algorithm proceeds as follows:

**Step 1: Initialization**
Initialize all parameters: line data (gradients, speed limits), train characteristics, scheduled travel time $T_{\text{set}}$, initial time penalty weight $\lambda$, step sizes for position ($\Delta s_k$) and speed ($\Delta v$), and the allowed time error $\delta$.

**Step 2: Discretization**
Discretize the route into $N$ segments, creating $N+1$ stages. Discretize the allowable speed range at each stage into $M$ possible values.

**Step 3: Data Structures**
Create two $M \times N$ matrices:
- `Matrix_C`: To store the optimal cumulative cost from each state to the destination.
- `Matrix_S`: To store the optimal decision (the index of the next state) for each state.

**Step 4: Backward DP Calculation**
- **Stage `k = N`:** For each state at the final stage $N$, calculate the cost to transition to the single terminal state (at position $s_{N+1}$ with speed 0). Populate the last column of `Matrix_C` and `Matrix_S`.
- **Stages `k = N-1` down to `2`:** For each state $x_{k,i}$ at the current stage, iterate through all possible next states $x_{k+1,j}$. Calculate the transition cost $C_{k,i}^{k+1,j}$. Using the Bellman equation and the already computed costs in `Matrix_C` for stage $k+1$, find the decision `j` that minimizes the total cost. Store this minimum cost in `Matrix_C(i, k)` and the corresponding optimal decision `j` in `Matrix_S(i, k)`.
- **Stage `k = 1`:** The train starts at a single state (position $s_1$, speed 0). Calculate the costs to transition to all possible states at stage 2 and find the optimal one.

**Step 5: Forward Trajectory Reconstruction**
Once the backward pass is complete, `Matrix_C(1, 1)` contains the minimum total cost for the entire journey. Starting from the initial state, use the `Matrix_S` to trace the optimal path forward through the stages, reconstructing the optimal decision sequence and the corresponding optimal speed profile $\{v_1, v_2, \dots, v_{N+1}\}$.

**Step 6: Time Constraint Check and $\lambda$ Update**
- Calculate the total travel time $T$ of the generated speed profile.
- **If $|T - T_{set}| \leq \delta$**: The solution is valid. Terminate and output the results (speed profile, energy consumption, etc.).
- **If $|T - T_{set}| > \delta$**: The travel time is not within the acceptable range. The time penalty weight $\lambda$ needs adjustment. Update $\lambda$ using a feedback rule, such as $\lambda_{new} = \lambda_{old} + \frac{T - T_{set}}{T_{set}}\lambda_{old}$, and return to Step 4 to re-run the entire DP calculation with the new $\lambda$. This iterative process repeats until the time constraint is satisfied.


### 5. Simulation and Results

The algorithm was implemented in MATLAB and tested on a real-world track section ($A_6 - A_7$).

**Simulation Parameters:**

| Parameter                | Value                                           |
| ------------------------ | ----------------------------------------------- |
| Train Weight (t)         | 194.295                                         |
| Max Speed (km/h)         | 80                                              |
| Basic Resistance (N/kN)  | `2.031 + 0.0622v + 0.001807v²`                  |
| Scheduled Time (s)       | 110                                             |
| Position Step Δs (m)     | 5                                               |
| Speed Precision Δv (m/s) | 0.01                                            |
| Time Tolerance δ (s)     | 1.1                                             |

**Results Summary:**

The resulting speed profile aligns with the principles of optimal control theory (Pontryagin's Maximum Principle), exhibiting a sequence of maximum traction, constant speed (or coasting), and maximum braking. The corresponding force profile shows the control actions taken by the train. Any small, non-zero forces during the coasting phase are attributable to the discrete nature of the state space.

| Metric                        | Value                 |
| ----------------------------- | --------------------- |
| Traction Energy (J)           | 3.6639e+07            |
| Regenerated Energy (J)        | 0                     |
| **Total Net Energy (J)**      | **3.6639e+07**        |
| **Actual Travel Time (s)**    | **111.083**           |
| Final Time Penalty Weight λ   | 632858.85             |
| Total Computation Time (s)    | 1315.53               |
| Number of Iterations          | 18                    |

The final travel time of 111.083 s successfully meets the requirement of being within 1.1 s of the 110 s target.