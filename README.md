# Project: Design, Verification, and Repair of an Adaptive Cruise Control (ACC) System

## 1. Objective

This project aims to develop, simulate, verify, and enhance a reliable, safe, and adaptive cruise control (ACC) system for vehicles. Key goals include:

*   Designing a Simulink model for ACC logic and vehicle dynamics.
*   Developing a MATLAB script for simulation, control implementation, adaptive logic, and cybersecurity (sensor spoofing) testing.
*   Creating a formal hybrid system model (XML) for safety and performance verification using formal methods tools (like SpaceEx).
*   Integrating PID control, adaptive distance-keeping logic, and basic fault tolerance against sensor attacks.

## 2. File Structure

```
ACC_Project/
├── README.md                 # This file: Overview, objectives, instructions.
├── ACC_Model_Description.txt # Detailed textual guide to build the Simulink model.
├── simulation_script.m       # MATLAB script for running simulations and testing.
├── Adaptive_Cruise_Control.xml # XML formal model template for hybrid system verification.
└── .gitignore                # Specifies files to ignore for Git version control.
```

*(Note: You will create `ACC_Model.slx` based on the description file)*

## 3. File Descriptions

*   **`ACC_Model_Description.txt`**: Contains a step-by-step textual guide detailing the blocks, parameters, and connections required to build the `ACC_Model.slx` file within the Simulink environment. Since `.slx` files are graphical and binary, this description serves as the blueprint.
*   **`simulation_script.m`**: The main MATLAB executable script.
    *   Sets up simulation parameters (time, step size).
    *   Defines vehicle, controller (PID gains), and ACC parameters (safe distance settings). **These require tuning!**
    *   Initializes vehicle state (speed, position).
    *   Runs a time-step simulation loop implementing:
        *   Lead vehicle behavior simulation.
        *   Relative distance calculation.
        *   Optional sensor spoofing injection.
        *   Adaptive logic to determine target speed based on sensed distance vs. safe distance.
        *   PID speed control calculation.
        *   Simple vehicle dynamics simulation (integrating acceleration to get speed and position).
    *   Plots key results (speeds, distances, acceleration) for analysis.
*   **`Adaptive_Cruise_Control.xml`**: An XML template representing the ACC system as a hybrid automaton, suitable for formal verification tools like SpaceEx.
    *   Defines system variables (speed, distance).
    *   Specifies discrete modes (e.g., Following, Decelerating, Attacked).
    *   Includes continuous dynamics (differential equations/flows) within each mode.
    *   Defines safety invariants (conditions that must always hold in a mode).
    *   Specifies transitions (guards and resets) between modes.
    *   **This is a template and requires adaptation** for your chosen tool and specific safety properties/attack models.

## 4. How to Use

1.  **Prerequisites:** MATLAB and Simulink installed. A formal verification tool (like SpaceEx) is needed for the XML part.
2.  **Setup:**
    *   Clone or download this repository/these files into a single folder on your computer.
    *   Open MATLAB and navigate its Current Folder browser to this project directory.
3.  **Build the Simulink Model:**
    *   Open Simulink (type `simulink` in MATLAB command window or use the toolbar).
    *   Create a new blank model (`Blank Model`).
    *   Follow the instructions in `ACC_Model_Description.txt` carefully to add blocks, set parameters, and connect signals.
    *   Save the model as `ACC_Model.slx` in the project folder.
4.  **Run the Simulation:**
    *   Open `simulation_script.m` in the MATLAB Editor.
    *   Review and **adjust the parameters** (especially PID gains `Kp, Ki, Kd`, `T_gap`, `d_min`, vehicle mass `m`) in the script to suit your system or desired behavior. Tuning is essential!
    *   Run the script by pressing the green "Run" button in the Editor tab or by typing `simulation_script` in the MATLAB Command Window.
    *   Observe the generated plots showing speeds, distances, and control action over time. Analyze the system's response, including behavior during simulated spoofing attacks (if enabled in the script parameters).
5.  **Formal Verification (Optional):**
    *   Adapt the `Adaptive_Cruise_Control.xml` file for your chosen formal methods tool (e.g., ensure syntax compatibility with SpaceEx).
    *   Define specific safety properties to verify (e.g., "Is d_rel always >= 0?", "Is v_ego always <= v_max?").
    *   Import the XML model into the tool and run reachability analysis or model checking according to the tool's documentation.

## 5. Disclaimer

This code provides a starting point and conceptual implementation. The MATLAB script uses simplified physics and requires significant parameter tuning for realistic performance. The Simulink model structure is a guide. The XML model is a template requiring adaptation and expertise in formal methods. This is intended for educational and conceptual purposes and is **not** suitable for real-world vehicle control without extensive validation, safety analysis, and adherence to automotive standards.
