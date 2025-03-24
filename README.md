# Drone Flight Simulator

A Python-based simulator that models a drone’s realistic trajectory following a given flight plan.

---

## 📋 Description

This project computes the path a drone would take between a series of waypoints, automatically adjusting turns using a configurable turn radius. The core of this simulator is its ability to model acceleration and deceleration based on turn curvature, producing realistic speed and position profiles throughout the flight.

---

## ⚙️ Key Features

- Precise turn simulation with adjustable turn radii  
- Dynamic acceleration/deceleration modeling during turns  
- Discrete output of position, velocity, and acceleration at configurable time intervals  
- Telemetry‑comparison tool: simulated vs. real  
- Validated against 50+ real drone flight paths, with a maximum time deviation of 8 seconds for flights longer than 3 minutes  

---

## 📺 YouTube Examples

1. Basic turn‑focused trajectory simulation: https://youtu.be/1aBcDeFgHIJ  
2. Real vs. simulated telemetry comparison: https://youtu.be/2KlMnOpQrSt  
3. Complex paths with adjustable turn radii: https://youtu.be/3UvWxYzAbCd
