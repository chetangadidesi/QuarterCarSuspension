# 🚗 Quarter Car Suspension System – Modeling & Control in MATLAB

This project simulates and controls a **quarter car suspension system**, focusing on ride comfort and system stability. Using MATLAB, I modeled the dynamics of a 2-mass system representing the car body and wheel assembly and tested multiple control strategies to improve ride quality under road disturbances.

---

## 🛠️ Features

- Full dynamic model of a quarter car suspension system
- Open-loop and closed-loop simulation with:
  - PID Controller
  - 2-Lead Compensator
  - Full-State Feedback Controller (Pole Placement)
  - Digital Discrete-Time Controller with Integral Action
- Step response analysis for a 0.1-meter road disturbance
- Unified subplot visualization to compare controller performance

---

## 📊 Technologies Used

- MATLAB Control System Toolbox
- State-space modeling
- Transfer function design
- Discrete-time control
- Pole placement (`place()` method)

---

## 🖥️ Running the Simulation

To run the script:

```matlab
% Open MATLAB and run:
quarter_car_simulation.m
