# Airbrake PID Optimization with RocketPy & Evolutionary Algorithms  
*(Digital Twin for EuroC Rocket)*

## Overview

This project implements a **digital twin** of an EuroC‑style rocket using RocketPy.  
It integrates:

- Full rocket flight simulation  
- Simulated noisy sensors (barometer, IMU, servo feedback)  
- PID‑controlled airbrakes  
- Evolutionary optimization of PID gains  
- Monte‑Carlo robustness testing  

The objective is to tune the airbrake controller to reach an **apogee target of 3000 m** despite disturbances.

---

## Project Structure

### `starPi.py` – Digital Twin + Sensors + PID Controller
Contains:  
- RocketPy environment & rocket model  
- Airbrake subsystem  
- Noisy sensor simulation  
- Servo actuator model  
- PID control logic  
- Full‑flight simulation via `simulate()`

### `pid.py` – PID Controller
Contains the standard PID implementation with:  
- proportional, integral, derivative terms  
- saturation  
- anti‑windup via clamping

### `evolve.py` – Evolutionary Search
Contains:  
- Random‑mutation evolutionary algorithm  
- Fitness function based on apogee error + actuator effort  
- Multi‑generation optimization  
- Final simulation & Monte‑Carlo analysis

---

## Sensor Models Used

### Barometer (Altimeter)
Noise:
- Gaussian noise σ ≈ 1.5 m  
- Slow drift (random walk)

### IMU Vertical Velocity (vz)
Noise:
- accel noise ≈ 0.03 g  
- integrated to velocity → noisy vz estimate

### Servo Airbrake Feedback
Noise & dynamics:
- jitter ±0.02  
- deadband ±1%  
- rate limit ≈ 0.25 per second  

These models make the digital twin more realistic and prevent overfitting to perfect data.

---

## Control Architecture

### PID Controller
The PID computes airbrake deployment based on:

```
error = v_reference(h) – vz_measured
```

Where:

- `vz_measured` comes from noisy IMU  
- `v_reference` is shaped to slow ascent near the apogee target  

### Evolutionary Optimization
The evolutionary algorithm searches for the best:

```
(Kp, Ki, Kd)
```

using fitness:

```
cost = |apogee_error| + 10 * actuator_effort
```

### Typical Results

- Manual PID error ≈ 150 m  
- Optimized PID error ≈ 0.1–20 m depending on noise level  
- Robust to turbulence and sensor noise  

---

## What’s Missing for a Full Real‑World Digital Twin

### 1. Real Flight Data (Airbrakes OFF)
Perform a flight with airbrakes disabled to record:

- Altitude profile  
- IMU data  
- Acceleration  
- Real apogee  
- Wind conditions  

This allows calibration of:
- Drag coefficients  
- Mass & CG  
- Atmosphere model  
- Sensor noise covariance  

### 2. Wind & Turbulence Models
To be added:
- Kaimal or Dryden turbulence  
- Shear profile  
- Real EuroC radiosonde data  

### 3. Better Sensor Models
To add realism:
- Temperature‑dependent baro noise  
- IMU bias drift  
- GPS latency (200–300 ms)  
- Sensor dropout during boost  

### 4. State Estimation
Real rockets require:
- Complementary filter  
- Kalman filter  
- Full sensor fusion  

### 5. Multi‑Scenario Monte‑Carlo
Include variations in:
- Mass ±5%  
- Motor impulse ±3%  
- Drag ±10%  
- Different winds  

This yields a *robust* controller valid across uncertainties.

---

## Real‑World Deployment Procedure

### Step 1 — Baseline Flight (No Airbrakes)
Record full telemetry.

### Step 2 — Digital Twin Calibration
Adjust:
- drag model  
- mass/inertia  
- sensor noise  
- wind model  

### Step 3 — Evolutionary Optimization on Calibrated Twin
Generate optimal PID gains.

### Step 4 — Static Testing of Airbrakes

### Step 5 — Flight Test with Airbrakes Active

---

## Summary

This project implements a full pipeline for designing an **AI‑optimized PID controller** for rocket airbrakes, including:

- realistic sensing  
- actuator limitations  
- digital twin physics  
- evolutionary tuning  

The system is designed as a foundation for real EuroC rocket flights.

---

## Author
Lorenzo Menchini  
2025 – EuroC Project  
