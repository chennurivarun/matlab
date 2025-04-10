# Adaptive Cruise Control (ACC) Project

This project involves the **design, verification, and repair** of an Adaptive Cruise Control (ACC) system using MATLAB, Simulink, Stateflow, Breach, S-TaLiRo, and optionally the NNV toolbox for neural network verification.

---

## Project Objectives

- **Design** an ACC system with sensor, controller, and vehicle dynamics models.
- **Verify** safety properties using Signal Temporal Logic (STL) and formal tools.
- **Simulate** cyber-attacks and develop fault-tolerant strategies.
- **(Optional)** Integrate and verify a neural network-based controller.

---

## Repository Structure

```
ACC_Project/
├── ACC_Model.slx               # Simulink model of the ACC system
├── simulation_script.m         # MATLAB script to run simulations
├── verification/               # Formal verification scripts
│   └── verification_script.m
├── docs/                      # Documentation and design notes
│   └── design_overview.pdf    # (To be created) Detailed design document
├── .gitignore                 # Files and patterns ignored by Git
└── README.md                  # Project overview and instructions
```

---

## Setup Instructions

1. **Requirements:**
   - MATLAB R2021a or later
   - Simulink
   - Stateflow
   - Breach toolbox
   - S-TaLiRo toolbox
   - (Optional) NNV toolbox for neural network verification

2. **Clone the repository:**

```bash
git clone <repository_url>
cd ACC_Project
```

3. **Add required toolboxes to MATLAB path.**

---

## Usage Instructions

### Run the ACC Simulation

1. Open MATLAB in the project directory.
2. Run the simulation script:

```matlab
simulation_script
```

3. The script will:
   - Open the Simulink model
   - Set simulation parameters
   - Run the simulation
   - (Optional) Plot results if configured

### Perform Formal Verification

1. Edit `verification/verification_script.m` to define STL properties and verification scenarios.
2. Run the script:

```matlab
verification/verification_script
```

3. Review the output to check if safety properties are satisfied.

### Documentation

- The `docs/` folder will contain detailed design documents and diagrams.
- Refer to `design_overview.pdf` (to be created) for system architecture and verification approach.

---

## Key Features

- **Sensor Modeling:** Simulates radar/lidar distance measurements.
- **Controller Logic:** Maintains safe distance and desired speed using Stateflow.
- **Vehicle Dynamics:** Models acceleration, braking, and speed changes.
- **Verification:** Uses STL properties to ensure safety.
- **Cybersecurity:** Simulates attack scenarios and repairs.
- **Neural Network (Optional):** Integrates AI-based control with formal verification.

---

## References

- [Breach Toolbox](https://github.com/decyphir/breach)
- [S-TaLiRo](https://sites.google.com/a/asu.edu/s-taliro)
- [NNV Toolbox](https://github.com/verivital/nnv)
- [MathWorks Simulink](https://www.mathworks.com/products/simulink.html)
- [MathWorks Stateflow](https://www.mathworks.com/products/stateflow.html)

---

## License

Specify your license here.

---

## Contact

Add your contact information or project contributors here.
