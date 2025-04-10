# ACC Project Implementation Guide

This guide provides step-by-step instructions to complete the Adaptive Cruise Control (ACC) project, including model development, verification, cybersecurity, and neural network integration.

---

## 1. Complete the Simulink Model (`ACC_Model.slx`)

### 1.1 Sensor Subsystem

- Simulate radar/lidar distance measurement.
- Add noise and attack injection points:
  - Use **Sum** blocks to add attack signals.
  - Create an **Attack Signal** input port.

### 1.2 Controller Subsystem (Stateflow)

- States:
  - **Cruise:** Maintain set speed.
  - **Follow:** Adjust speed to maintain safe distance.
  - **Brake:** Decelerate if too close.
- Inputs:
  - Relative distance
  - Relative speed
- Outputs:
  - Acceleration or braking commands
- Add logic for **anomaly detection**:
  - If sensor data is inconsistent, switch to safe mode.

### 1.3 Vehicle Dynamics Subsystem

- Integrate acceleration to update speed and position.
- Include actuator dynamics if needed.

### 1.4 Signal Logging

- Log:
  - Distance
  - Vehicle speed
  - Acceleration
  - Control commands
- Use **To Workspace** blocks with `Array` format.

---

## 2. Develop Verification Properties

### 2.1 Example STL Properties

- Safe distance:
  ```
  [] (distance > 5)
  ```
- Speed limit:
  ```
  [] (speed <= 30)
  ```
- Smooth acceleration:
  ```
  [] (abs(acceleration) < 3)
  ```

### 2.2 Add to `verification/verification_script.m`

- Define multiple properties.
- Use Breach or S-TaLiRo to falsify each property.
- Analyze counterexamples and refine the model.

---

## 3. Simulate Cybersecurity Attacks

### 3.1 Attack Injection

- Create attack signals:
  - Constant bias
  - Random noise
  - Replay attacks
- Inject into sensor inputs via **Sum** blocks.

### 3.2 Detection and Mitigation

- Add logic to detect anomalies:
  - Sudden jumps
  - Implausible values
- Switch to safe fallback mode if attack detected.

### 3.3 Verification Under Attack

- Re-run STL property checks with attacks enabled.
- Ensure safety properties still hold.

---

## 4. Neural Network Controller (Optional)

### 4.1 Training

- Collect data from simulations.
- Train a neural network to mimic or improve the controller.
- Use MATLAB's Deep Learning Toolbox.

### 4.2 Integration

- Export the trained network.
- Import into Simulink using the **NNV toolbox** or **Deep Learning Toolbox** blocks.
- Replace or augment the Stateflow controller.

### 4.3 Verification

- Use NNV to verify robustness and safety.
- Compare with traditional controller.

---

## 5. Automation and Batch Processing

- Enhance `simulation_script.m` to:
  - Run multiple scenarios automatically.
  - Save plots and data.
  - Include attack scenarios.

- Enhance `verification/verification_script.m` to:
  - Batch verify multiple properties.
  - Log results and counterexamples.

---

## 6. Documentation

- Update `docs/design_overview.md` with:
  - New diagrams
  - Verification results
  - Cybersecurity strategies
  - Neural network integration details

- Convert Markdown files to PDF for sharing.

---

## 7. Version Control Tips

- Commit changes frequently.
- Use branches for new features.
- Tag stable versions.

---

*Follow this guide to systematically complete and extend your ACC project.*
