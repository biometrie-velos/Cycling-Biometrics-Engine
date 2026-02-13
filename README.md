# Cycling Biometrics Engine

**Real-Time Cycling Biomechanics, Motion Analysis & Performance Engineering**

---

## 🚀 Overview

The Cycling Biometrics Engine is a high-performance research and development
platform dedicated to real-time cycling biomechanics analysis.

It combines advanced computer vision, GPU acceleration, embedded electronics,
and custom firmware to deliver accurate, reproducible, and field-ready
measurements for performance optimization and injury prevention.

This system is actively used in professional retail environments and applied
research contexts.

---

## 🧠 Core Technologies

### Software & Algorithms
- Computer Vision (OpenCV / EmguCV)
- GPU Acceleration (CUDA, OpenGL, OpenCL)
- Real-Time Kalman Filtering & Signal Smoothing
- Dynamic ROI Tracking
- Pose Estimation (YOLOv8 / ONNX Runtime)
- Aerodynamic Modeling (CdA, Drag, Power Breakdown)
- Multi-threaded Processing Pipelines

### Hardware, Electronics & Firmware
- Custom FSR Pressure Sensor Matrices (16×16 / 48×48)
- Embedded Acquisition Boards (Custom PCB)
- Microcontroller Firmware (C / Embedded C++)
- High-Speed Sensor Sampling
- USB / Serial / HID Communication Protocols
- Real-Time Synchronization with Vision Pipelines

### Visualization & UI
- OpenGL Core Profile Rendering
- Real-Time Overlays & Metrics
- WPF / WinForms Hybrid Interfaces
- Dynamic Plotting (OxyPlot / Custom GPU Charts)
- Interactive Calibration & ROI Editors

---

## 🏗️ System Architecture

Cameras / Sensors  
        ↓  
GPU Preprocessing (CUDA / OpenCL)  
        ↓  
Pose & Color Detection (ONNX / OpenCV)  
        ↓  
Kalman Filtering & Sensor Fusion  
        ↓  
Biomechanical & Aerodynamic Models  
        ↓  
OpenGL Visualization Layer  
        ↓  
Report Generation & Data Export  

---

## Main Modules

- Vision Processing Engine  
- GPU Acceleration Layer  
- Kalman & Signal Processing Core  
- Knee & Joint Angle Analysis  
- Aerodynamic Surface Estimation  
- Pressure Mapping Integration  
- Embedded Sensor Interface  
- Calibration & Validation Tools  

---

## Applications

- Professional Bike Fitting  
- Performance Optimization  
- Injury Risk Detection  
- Aerodynamic Position Testing  
- Retail & Applied Sports Science  
- Long-Term Athlete Monitoring  

---

## Research & Engineering Philosophy

This project is designed around the following principles:

- Measurement repeatability and accuracy  
- Real-time reliability  
- Hardware-software co-design  
- GPU-first processing pipelines  
- Embedded system integration  
- Field-ready robustness  

All published modules are derived from validated internal systems.

---

## Repository Structure

/src        → Core software modules  
/firmware   → Embedded firmware sources  
/hardware   → PCB designs & electronics documentation  
/demo       → Demonstration projects  
/docs       → Technical documentation  

---

## Intellectual Property

This repository contains selected open components of a proprietary biomechanics platform.

Advanced algorithms, calibration procedures, and commercial deployments remain private.

Reuse, modification, or integration requires explicit authorization.

---

## Author

Hubert de Carvalho  
Founder – AI-Enabled Cycling Biomechanics Lab  

Cycling Biometrics.  
Real-Time Motion Analysis & Performance Engineering.

---

## Links

Website: https://www.biometrie-velos.fr  
Medium: https://medium.com/@biometrie.velos  
