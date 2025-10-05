# ChooChoo OS 

Welcome to ChooChoo OS, a lightweight, real-time operating system designed to support a multitasking train control project. This repository contains both the OS implementation and related projects, with detailed documentation for each component.

# Key Components

- **Operating System**: Explore the architecture, kernel design, and system-level features in the [**OS Report**](docs/report-k4.md)
.

- **Train Project**: Learn how the OS powers a dynamic train control system with preemptive scheduling, message passing, and path planning in the [**Train Control Report**](docs/report-TC1.md)
.

# ChooChoo OS - Build and Operation Guide

## Building the System

### Prerequisites
- ARM64 cross-compiler toolchain
- CMake (version 3.10 or higher)
- Make
- Ninja

### Build Instructions

#### Standard Kernel Build

For CS Environment:
```bash
make
```

The build process will:
1. Clean and create a fresh build directory
2. Configure CMake with appropriate cross-compiler and MMU settings
3. Build the kernel with embedded user applications
4. Generate `kernel.img` ready for deployment


#### Cleaning Build Artifacts

```bash
make clean
```

## Operating the System

### Starting Up
1. Load the `kernel.img`
2. The system will boot and initialize the TUI (Text User Interface)
3. Use command `reset <A/B>` to reset/initialize the track
4. Use command `spawn <train> <Sensor>` to spawn a train at a sensor location

#### Initial Setup
1. **Reset the track**: `reset A` or `reset B` (depending on track type)
2. **Spawn a train**: `spawn <train_number> <sensor_name>`
   - Example: `spawn 14 A1` (spawns train 14 at sensor A1)
3. **Get the train moving**: `random <train_number> on`
   - Example: `random 14 on`
