# Pip-Boy Interface

![C++](https://img.shields.io/badge/C++-17-blue)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi-green)
![Status](https://img.shields.io/badge/Status-Prototype-orange)
![UI](https://img.shields.io/badge/UI-Retro%20Futuristic-9cf)

**Pip-Boy Interface** is a retro-futuristic embedded dashboard inspired by the iconic Pip-Boy devices from the *Fallout* universe. The project explores the design of responsive human–machine interfaces (HMI) for embedded systems using **C++** and lightweight graphical frameworks.

The interface provides a real-time monitoring dashboard capable of displaying system information, interacting with connected hardware, and acting as a control panel for embedded devices.

The project focuses on **performance, modular architecture, and embedded compatibility**, making it suitable for platforms such as **Raspberry Pi** or similar embedded systems.

---

# Project Overview

Pip-Boy Interface combines retro-futuristic UI design with embedded system interaction. The system acts as a modular dashboard capable of visualizing system data and controlling connected components.

Inspired by wearable computing concepts, the interface is designed to simulate a **compact control device** capable of monitoring system states and interacting with hardware modules.

The project serves as both:

* an experiment in **embedded UI design**
* a **prototype monitoring dashboard** for hardware-connected systems.

---

# Features

* Retro-futuristic Pip-Boy inspired interface
* Real-time monitoring dashboard
* Modular screen architecture
* Hardware interaction ready
* Lightweight implementation in C++
* Embedded-friendly design
* Expandable module system

---

# Architecture

The system follows a layered architecture separating interface rendering, application logic, and hardware interaction.

```text
User
 │
 ▼
Interface Layer
(C++ UI rendering)
 │
 ▼
Application Layer
(Control logic modules)
 │
 ▼
Hardware Layer
(Raspberry Pi GPIO / external devices)
```

This separation allows the system to scale easily as new modules, sensors, or hardware interfaces are added.

---

# Tech Stack

The system is built using technologies suitable for high-performance embedded applications.

* **C++**
* **Embedded UI frameworks**
* **Raspberry Pi**
* **GPIO hardware interaction**
* **Embedded HMI concepts**

---

# Project Structure

```text
pipboy-interface
├── src/            # Core application source code
├── ui/             # Interface components and screens
├── modules/        # Functional dashboard modules
├── assets/         # Visual resources and themes
├── include/        # Header files
├── CMakeLists.txt  # Build configuration
└── README.md
```

The modular structure allows new dashboard screens or control modules to be integrated without modifying the core system.

---

# Build Instructions

Clone the repository:

```bash
git clone https://github.com/qetrfd/pipboy-interface.git
cd pipboy-interface
```

Create a build directory:

```bash
mkdir build
cd build
```

Compile the project:

```bash
cmake ..
make
```

Run the application:

```bash
./pipboy
```

---

# Development Status

⚠️ **Prototype / Early Development**

This project is currently under active development and represents an experimental prototype.

Current limitations may include:

* incomplete hardware integration
* experimental UI components
* ongoing architectural improvements

The repository primarily documents the early development of the interface and its embedded design approach.

---

# Project Goals

The main objectives of the Pip-Boy Interface include:

* Exploring embedded HMI design patterns
* Developing responsive control dashboards
* Integrating software interfaces with hardware platforms
* Experimenting with retro-futuristic UI design
* Creating a modular embedded monitoring system

---

# Future Development

Planned improvements include:

* Real-time sensor monitoring
* GPIO device integration
* Touch-friendly interface layout
* Additional dashboard modules
* Serial / UART device support
* Embedded performance optimization

---

# Author

**Fernando Hiram Santillán Rodríguez**

---

# License

This project is intended for **educational, experimental, and portfolio purposes**.
