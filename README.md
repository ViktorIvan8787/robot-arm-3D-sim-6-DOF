# 6-DOF Pick and Place Robotic Arm

A 6 Degrees of Freedom (DOF) articulated robotic arm project inspired by the kinematic dimension ratios of the KUKA KR3 R540[cite: 1]. The repository contains the physical hardware specifications, control architecture, and a real-time interactive 3D simulation built using C++17 and Raylib.

---

## System Overview

The system is divided into three primary operational modules[cite: 1]:

1. **Motion System:** 6-axis articulated movement driven by smart serial bus servos with embedded encoders and position feedback[cite: 1].
2. **Vacuum Effector System:** Dual 12V DC mini vacuum pumps operating a suction cup mechanism via MOSFET switching[cite: 1].
3. **Vision System:** Fixed-position camera tracking utilizing OpenCV and Python for automated object location and coordinate calculation[cite: 1].