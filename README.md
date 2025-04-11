# Adaptive Cruise Control (ACC) System: Design, Simulation, and Analysis

## 1. Introduction & Objective

Adaptive Cruise Control (ACC) is a crucial driver-assistance system in modern vehicles. Unlike standard cruise control which maintains a fixed speed, ACC adapts the vehicle's speed to maintain a desired speed *and* a safe following distance from a lead vehicle detected by sensors (like radar or cameras).

This project aimed to:

*   **Design and Simulate:** Develop a functional ACC system model incorporating core logic for speed control and distance keeping.
*   **Implement Control:** Utilize a Proportional-Integral-Derivative (PID) controller for longitudinal speed regulation.
*   **Incorporate Adaptivity:** Implement logic to adjust the target speed based on the distance to a lead vehicle, switching between maintaining set speed and following the lead vehicle.
*   **Simulate Cybersecurity Attack:** Model a sensor spoofing attack (specifically, manipulating the perceived distance) and observe the system's reaction.
*   **Achieve Stability:** Iteratively tune the control system parameters to achieve smooth, stable, and realistic performance, eliminating initial instabilities observed during development.
*   **Analyze Performance:** Evaluate the system's behavior through simulation plots under various conditions, including normal following, lead vehicle speed changes, and sensor attacks.
*   **Prepare for Verification:** Outline the system structure for potential formal verification using hybrid system models.

The primary tool used for simulation, logic implementation, and analysis was MATLAB, employing a discrete-time simulation script. A corresponding Simulink model structure was also described.

## 2. System Architecture

The simulated system consists of several key interacting components managed within the MATLAB script (`simulation_script.m`):

*   **Lead Vehicle Model:** Simulates the behavior (speed changes over time) of the vehicle ahead.
*   **Ego Vehicle Dynamics Model:** A simplified physics-based model representing the controlled vehicle's response to acceleration/braking commands, including mass and aerodynamic drag.
*   **Sensor Model (Distance):** Calculates the actual relative distance and simulates the *sensed* distance, incorporating a mechanism to inject spoofing errors.
*   **ACC Logic Unit:**
    *   Calculates the dynamic safe following distance.
    *   Compares sensed distance to safe distance.
    *   Selects the appropriate target speed (`v_target`) based on this comparison (using hysteresis).
*   **PID Speed Controller:** Calculates the necessary acceleration/deceleration command (`F_control`, converted to `a_ego`) to minimize the error between the target speed (`v_target`) and the actual ego vehicle speed (`v_ego`).
*   **Simulation Engine:** The MATLAB script itself, running a discrete-time loop (`for k = 1:n_steps-1`), updating states at each time step `dt`.
*   **Visualization:** MATLAB plotting functions to generate graphs of speeds, distances, and acceleration over time.

*(A corresponding Simulink model (`ACC_Model.slx`) can represent this architecture graphically, potentially using workspace variables defined by the MATLAB script for initialization.)*
*(An XML file (`Adaptive_Cruise_Control.xml`) was prepared as a template for formal hybrid system modeling, defining system modes, invariants, and transitions for verification tools.)*

## 3. Design & Implementation Details

### 3.1. Relative Distance Calculation

The actual distance between the ego vehicle (`pos_ego`) and the lead vehicle (`pos_lead`) was tracked over time. Assuming the ego vehicle starts at position 0 and the lead vehicle starts at `d_lead_initial_offset`:

*   `pos_lead(k+1) = pos_lead(k) + v_lead(k) * dt`
*   `pos_ego(k+1) = pos_ego(k) + v_ego(k+1) * dt`
*   `d_rel(k) = pos_lead(k) - pos_ego(k)`

Alternatively, relative speed (`v_rel = v_lead - v_ego`) can be integrated:

*   `d_rel_change(k) = integral(v_rel(t) dt)` from t=0 to t=k*dt
*   `d_rel(k) = d_lead_initial_offset + d_rel_change(k)`

### 3.2. Safe Following Distance

A dynamic safe distance (`d_safe`) was calculated at each time step based on the ego vehicle's current speed (`v_ego`), a fixed minimum distance cushion (`d_min`), and a desired time gap (`T_gap`):

*   `d_safe(k) = d_min + T_gap * v_ego(k)`
*   Ensured `d_safe(k)` is always at least `d_min`.

### 3.3. Target Speed Selection (Adaptive Logic with Hysteresis)

The core adaptive logic determines the target speed (`v_target`) for the PID controller:

1.  **Compare Sensed Distance to Safe Distance:** Check if `d_sensed(k) < d_safe(k)`.
2.  **Hysteresis:** To prevent rapid switching when `d_sensed` hovers near `d_safe`, a state variable (`was_too_close`) and a hysteresis buffer (`hyst_delta`) were introduced.
    *   **Switching to Follow:** If the state was "aiming for set speed" and `d_sensed(k) < d_safe(k)`, switch `v_target(k) = v_lead(k)` and set `was_too_close = true`.
    *   **Switching back to Set Speed:** If the state was "following lead speed" (`was_too_close == true`), only switch back (`v_target(k) = v_set`, `was_too_close = false`) if `d_sensed(k) >= d_safe(k) + hyst_delta`. Otherwise, continue following `v_lead(k)`.
3.  **Clamping:** `v_target` is clamped between 0 and `v_max_limit`.

### 3.4. Sensor Spoofing Simulation

A simple offset attack was simulated:

*   If `enable_spoofing` is true and the current time `t(k)` is within the `[attack_start_time, attack_end_time)` interval:
    *   `d_sensed(k) = d_rel(k) + spoofing_value` (where `spoofing_value` was -30m in the test).
*   Otherwise, `d_sensed(k) = d_rel(k)`.
*   `d_sensed` was ensured to be non-negative.

### 3.5. PID Speed Controller

The PID controller calculates the required control force (`F_control`) to minimize the speed error (`speed_error = v_target - v_ego`):

*   **Error Terms:**
    *   Proportional: `P_term = Kp * speed_error(k)`
    *   Integral: `I_term = Ki * integral_error` (where `integral_error` accumulates `speed_error * dt`)
    *   Derivative: `D_term = Kd * (speed_error(k) - previous_error) / dt`
*   **Control Force:** `F_control(k) = P_term + I_term + D_term`
*   **Saturation:** `F_control` was limited between `F_min` and `F_max`.

**Tuning:** Initial simulations showed extreme instability (oscillations). The gains `Kp`, `Ki`, `Kd` required significant iterative tuning (reducing `Kp` drastically, reintroducing `Ki`, then adding `Kd`) to achieve the final stable response.

### 3.6. Ego Vehicle Dynamics

A simplified physics model was used:

*   **Forces:**
    *   `F_drag = 0.5 * rho * Cd * A * v_ego(k)^2`
    *   `F_net = F_control(k) - F_drag`
*   **Acceleration:** `a_ego(k) = F_net / m`
*   **Integration (Euler method):**
    *   `v_ego(k+1) = v_ego(k) + a_ego(k) * dt`
    *   `pos_ego(k+1) = pos_ego(k) + v_ego(k+1) * dt`
*   **Limits:** `v_ego` was limited between 0 and `v_max_limit`. `a_ego` was effectively limited by `F_min`/`F_max`.

## 4. Key Formulas Used

*   **Safe Distance:** `d_safe = d_min + T_gap * v_ego`
*   **Relative Distance:** `d_rel = integral(v_lead - v_ego) dt + d_rel(0)`
*   **Sensed Distance (Attack):** `d_sensed = d_rel + spoofing_offset` (during attack)
*   **Speed Error:** `error = v_target - v_ego`
*   **PID Control Force:** `F_control = Kp*error + Ki*integral(error) + Kd*d(error)/dt`
*   **Drag Force:** `F_drag = 0.5 * rho * Cd * A * v_ego^2`
*   **Newton's Second Law:** `a_ego = (F_control - F_drag) / m`
*   **Euler Integration:** `state(k+1) = state(k) + derivative(state(k)) * dt`

## 5. Simulation Setup (Final Stable Run)

The successful, stable simulation results were achieved using the following key parameters (from command window output):

*   Simulation Time (`T_sim`): 60.0 s
*   Time Step (`dt`): 0.050 s
*   Ego Vehicle Mass (`m`): 1500 kg
*   Max Acceleration (`a_max`): 2.0 m/s²
*   Max Deceleration (`a_min`): -5.0 m/s²
*   Set Speed (`v_set`): 30.0 m/s
*   Time Gap (`T_gap`): 1.8 s
*   Min Distance (`d_min`): 5.0 m
*   Hysteresis Delta (`hyst_delta`): 2.0 m
*   **PID Gains:** `Kp=150.0`, `Ki=15.0`, `Kd=50.0`
*   Spoofing Attack: Enabled, Offset=-30.0 m, Time=[25.0s, 40.0s]

## 6. Results & Discussion

The simulation results after PID tuning and incorporating hysteresis demonstrated a stable and effective ACC system.

```markdown
![image](https://github.com/user-attachments/assets/5cf9f3e4-3ab8-478b-983e-41ecaf55a4e1)
<!-- Placeholder for the final simulation results image -->
<!-- Replace the line below with the actual markdown for your image -->
![Final Simulation Results](placeholder_for_final_stable_plot.png "ACC Simulation Results after Tuning and Hysteresis")
<!-- Example: ![Final Simulation Results](acc_results_stable.png "ACC Simulation Results after Tuning and Hysteresis") -->
Analysis of Final Results:
Stable Speed Control: The ego vehicle smoothly tracked the target speed, whether it was the set speed or the lead vehicle's speed. Initial oscillations were completely eliminated.
Smooth Following: The vehicle maintained a safe distance from the lead vehicle without jerky acceleration or braking during steady following.
Appropriate Response to Lead Vehicle: The system reacted smoothly and appropriately when the lead vehicle decelerated at t=10s and accelerated at t=50s.
Handled Spoofing Attack: During the sensor spoofing attack (t=25s-40s), the system perceived a much smaller distance (d_sensed). It correctly interpreted this as an unsafe condition and applied smooth, sustained braking, causing the actual distance (d_rel) to increase as a safety measure.
Stable Recovery: After the attack ended at t=40s, the system smoothly accelerated to close the large gap created during the attack, eventually re-establishing the correct following distance based on the actual sensor readings.
Hysteresis Benefit: The hysteresis logic prevented potential rapid switching of the target speed when the sensed distance was close to the safe distance threshold (visible around t=42s), contributing to the overall smoothness.
Importance of Tuning: The process highlighted the critical importance of PID tuning. The initial gains led to extreme instability, while the final tuned gains resulted in stable, desirable performance.
7. Formal Verification Aspect
While detailed formal analysis was beyond the scope of the immediate simulation tuning, the project structure included an XML template (Adaptive_Cruise_Control.xml) for representing the system as a hybrid automaton. This model defines:
Modes: Discrete operating states (Following, Decelerating, EmergencyBraking, AttackedResponse).
Flows: Differential equations describing continuous variable evolution (v_ego, d_rel) within each mode.
Invariants: Conditions that must hold true while in a specific mode (e.g., d_rel >= 0, v_ego <= v_max_limit).
Transitions: Guard conditions that trigger switches between modes.
Using tools like SpaceEx, this formal model could be analyzed to mathematically prove safety properties, such as collision avoidance (d_rel >= 0) or adherence to speed limits, under specified assumptions about lead vehicle behavior and system parameters.
8. Conclusion
This project successfully designed, simulated, and tuned an Adaptive Cruise Control system using MATLAB. The simulation demonstrated:
Stable PID-based speed control.
Effective adaptive logic for maintaining safe following distances.
Appropriate reaction to lead vehicle speed changes.
A predictable (braking) response to a simulated sensor spoofing attack.
The critical role of PID tuning and the benefit of hysteresis for achieving smooth, stable performance.
The project provides a solid foundation for understanding ACC principles and serves as a basis for further exploration into more advanced control strategies, sensor modeling, attack detection/mitigation, and formal verification.


