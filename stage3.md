# COUNCIL ON SCIENCE AND GUJARAT TECHNOLOGY
## Department of Science and Technology, Government of Gujarat

# FINAL TECHNICAL REPORT SUBMISSION FOR LEVEL-III "PROTOTYPE" OF ROBOFEST-GUJARAT 4.0

## Table of Contents
1. [Description of Robot](#1-description-of-robot)
2. [Salient Features](#2-salient-features)
3. [Mechanical Design](#3-mechanical-design)
4. [Working Methodology](#4-working-methodology)
5. [Software Implementation](#5-software-implementation)
6. [Manufacturing Details](#6-manufacturing-details)
7. [Electronic Components](#7-electronic-components)
8. [Electronic System Working](#8-electronic-system-working)
9. [System Architecture](#9-system-architecture)
10. [Future Applications](#10-future-applications)

## 1. Description of Robot

Our hexapod robot represents a groundbreaking innovation in robotics, combining the stability of six-legged locomotion with the efficiency of rolling movement. The robot features a unique dual-mode transformation capability, allowing it to switch between walking and rolling modes seamlessly.

The design consists of:
- Two circular end plates connected by a central aluminum rod
- Six independently controlled legs with servo motors
- Integrated rolling mechanism with outer disks
- Advanced sensor suite including LIDAR and depth cameras
- Centralized control system with Raspberry Pi and Arduino

Key innovations include:
- Transformation capability between walking and rolling modes
- Real-time terrain adaptation
- Autonomous navigation system
- High-efficiency power management
- Modular design for easy maintenance

## 2. Salient Features

### 2.1 Dual-Mode Locomotion
- Seamless transition between walking and rolling modes
- Automatic mode selection based on terrain
- Efficient power usage in both modes

### 2.2 Advanced Control System
- Multi-processor architecture
- Real-time sensor processing
- Adaptive gait control
- Autonomous navigation capabilities

### 2.3 Mechanical Innovation
- Custom gear system for rolling mechanism
- High-torque servo motors for precise leg control
- Integrated slip ring for continuous rotation
- Robust aluminum frame construction

### 2.4 Sensor Integration
- Stereoscopic 3D depth camera
- IMU for stability control
- Distance sensors for obstacle detection
- Encoder feedback for precise movement

## 3. Mechanical Design

### 3.1 Frame Structure
- Dual Circular End Plates:
  * Blue-coated aluminum outer plates with precision-cut segments
  * White inner plates with integrated mounting points
  * Green and white accent sections for visual identification
  * Multiple mounting holes for leg assemblies and components
- Central Electronics Housing:
  * Two-tier design for component separation
  * Upper tier housing Raspberry Pi with USB and network ports
  * Lower tier containing power management and motor drivers
  * Structured cable management system
- Structural Integration:
  * Central aluminum rod providing core stability
  * Multiple intermediate plates for component mounting
  * Precision-aligned bearing housings
  * Integrated cooling vents in plates

### 3.2 Drive Mechanism
- Rolling System:
  * Large diameter outer rings with internal gear teeth
  * Precision-machined spur gears for power transmission
  * Dual DC motor drive system
  * Synchronized rotation control
- Leg Actuation:
  * Six modular leg assemblies
  * Dual servo configuration per leg
  * Quick-disconnect mounting system
  * Integrated limit switches
- Power Transfer:
  * Central slip ring assembly for continuous rotation
  * Multi-channel power and signal routing
  * Sealed bearing assemblies at both ends
  * Redundant power distribution paths

### 3.3 Transformation System
- Leg Integration:
  * Symmetrical leg placement around outer disks
  * Precision-engineered folding mechanism
  * Spring-loaded deployment system
  * Position feedback sensors
- Disk Mechanism:
  * Synchronized outer disk rotation
  * Internal gear teeth for drive engagement
  * Balanced weight distribution
  * Emergency release mechanism
- Motion Control:
  * Coordinated servo movement for transformation
  * Position-sensing feedback loop
  * Automatic alignment system
  * Fail-safe locking mechanism

### 3.4 Component Layout
- Central Section:
  * Two-layer PCB mounting system
  * Accessible maintenance ports
  * Thermal management channels
  * Modular component organization
- Power Distribution:
  * Centralized battery compartment
  * Voltage regulation modules
  * Current monitoring systems
  * Emergency power cutoff
- Sensor Integration:
  * Strategic sensor placement points
  * Protected cable routing channels
  * Calibration reference markers
  * Modular sensor mounting system

### 3.5 Materials and Finish
- Primary Structure:
  * Blue anodized aluminum outer plates
  * White powder-coated inner components
  * Green accent elements for visual guides
  * High-grade stainless steel fasteners
- Precision Components:
  * CNC-machined aluminum parts
  * Industrial-grade bearings
  * Hardened steel gears
  * High-impact plastic covers

## 4. Working Methodology

### 4.1 Walking Mode
- Tripod gait pattern implementation
- Real-time terrain adaptation
- Dynamic stability control
- Independent leg coordination

### 4.2 Rolling Mode
- Synchronized disk rotation
- Center body stabilization
- Return wheel mechanism
- IMU-based orientation control

### 4.3 Transformation Process
1. Initial position assessment
2. Leg retraction sequence
3. Disk alignment
4. Mode transition completion

## 5. Software Implementation

### 5.1 Control Architecture
- ROS-based system integration
- Python for high-level control
- Arduino for motor control
- Real-time sensor processing

### 5.2 Navigation System
- Path planning algorithms
- Obstacle avoidance
- Terrain mapping
- Autonomous decision making

## 6. Manufacturing Details

### 6.1 Materials Used
- Aluminum Components:
  * 6061-T6 aluminum for main plates
  * Custom anodized finish in blue
  * Precision-machined gear teeth
  * Surface treated for durability
- Mechanical Components:
  * High-precision bearings rated for continuous rotation
  * Hardened steel gears for drive system
  * Custom-designed slip rings
  * Industrial-grade servo motors
- Electronic Integration:
  * Custom-designed PCB layouts
  * Shielded cable assemblies
  * Temperature-resistant connectors
  * Vibration-isolated mounting systems

### 6.2 Assembly Process
- Precision Assembly:
  * Computer-aided alignment procedures
  * Torque-controlled fastening
  * Calibrated gear mesh setting
  * Bearing preload adjustment
- Quality Control:
  * Digital measurement verification
  * Motion smoothness testing
  * Power system validation
  * Sensor calibration procedures
- Integration Testing:
  * Systematic component testing
  * Full motion range verification
  * Power consumption monitoring
  * Communication system validation

## 7. Electronic Components

### 7.1 Processing Units
- Raspberry Pi 4
- Arduino Mega
- Custom control boards

### 7.2 Sensors
- MPU9250 IMU
- Depth camera
- Encoders
- Distance sensors

### 7.3 Power System
- 14.8V LiPo battery
- Voltage regulators
- Power distribution board

## 8. Electronic System Working

### 8.1 Power Management
- Multi-level voltage regulation
- Battery monitoring
- Safe shutdown system

### 8.2 Control Hierarchy
- High-level planning
- Real-time control
- Motor coordination
- Sensor integration

## 9. System Architecture

[Block diagram showing system integration]

## 10. Future Applications

### 10.1 Industrial Use
- Hazardous environment inspection
- Manufacturing automation
- Quality control

### 10.2 Search and Rescue
- Disaster response
- Emergency services
- Urban search and rescue

### 10.3 Research Applications
- Robotics development
- AI integration
- Autonomous systems

---

## Team Information

[Team member details and signatures to be added]

## Appendices

1. Technical Documentation
2. Test Results
3. CAD Drawings
4. Source Code
5. Photo Documentation 