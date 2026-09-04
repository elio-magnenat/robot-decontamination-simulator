# Robot Decontamination Simulator

A C++ simulation and trajectory-planning project developed as part of the **HEIG-VD Summer HES 2026** program.

The project simulates autonomous differential-drive robots moving in a 2D environment. Their objective is to reach and decontaminate particles while respecting movement constraints, avoiding invalid collisions, and generating valid simulation timelines.

This project was developed by a **team of six students** during only two weeks.

---

## Overview

The simulator models robots controlled through independent left and right wheel speeds.

Depending on the wheel commands, a robot can:

- move in a straight line,
- rotate,
- follow circular arcs,
- reach and decontaminate particles,
- interact with obstacles and world boundaries.

The project also includes a trajectory planner capable of evaluating several possible commands and estimating their consequences before selecting a movement.

---

## Main Features

- Differential-drive robot simulation
- Straight and arc trajectory generation
- Robot and particle interaction
- Collision and contact detection
- Candidate wheel-command simulation
- Trajectory evaluation
- Lookahead planning
- Remaining-time / ETA estimation
- Timeline generation
- Validation scenarios
- Graphical visualization of the simulation

---

## My Contribution

My primary contribution to the project was the **trajectory subsystem**, located mainly in:

```text
src/simulation/trajectory/
```

I worked on the logic used to predict robot movements and evaluate possible trajectories.

This included:

- simulation of wheel commands,
- straight and circular trajectory handling,
- prediction of robot positions and orientations,
- particle contact detection,
- collision prediction,
- simulation of candidate commands,
- trajectory planning and command comparison,
- lookahead evaluation of future robot states,
- integration with timeline generation,
- debugging and validation of trajectory behavior.

The trajectory planner does not only evaluate the immediate result of a command. It can also simulate possible future decisions, allowing the robot to prefer commands that lead to better future states.

---

## Trajectory Planning

For each available wheel command, the planner simulates what would happen if the robot executed that command.

A candidate can contain information such as:

- final robot position,
- final orientation,
- elapsed time,
- particle contact,
- collision,
- decontamination opportunity.

The resulting candidates are evaluated and compared.

A simplified view of the process is:

```text
Available wheel commands
          |
          v
  Simulate each command
          |
          v
   Generate candidates
          |
          v
   Evaluate candidates
          |
          v
  Explore future states
          |
          v
 Select the best command
```

This allows the robot to consider both the immediate movement and its potential consequences.

---

## Differential-Drive Motion

The robots use two independently controlled wheels.

When both wheels move at the same speed, the robot follows a straight trajectory.

```text
Left wheel  =  Right wheel
        |
        v
   Straight motion
```

When the wheel speeds are different, the robot follows a circular arc.

```text
Left wheel  !=  Right wheel
        |
        v
    Arc trajectory
```

This motion model is used throughout the trajectory simulation and planning system.

---

## Project Structure

```text
robot-decontamination-simulator/
│
├── Data/
│   └── Validation/
│
├── src/
│   ├── common/
│   ├── render/
│   ├── simulation/
│   │   └── trajectory/
│   └── CMakeLists.txt
│
├── .gitignore
└── README.md
```

### `src/common`

Shared structures and utilities used by the application.

### `src/render`

Graphical rendering and visualization of the simulation.

### `src/simulation`

Simulation logic for robots, particles, interactions and trajectories.

### `src/simulation/trajectory`

Trajectory prediction and planning subsystem.

### `Data/Validation`

Validation scenarios used to verify the behavior of the simulator.

---

## Technologies

- **C++**
- **CMake**
- **Qt**
- **Git**
- Object-oriented programming
- Computational geometry
- Simulation and trajectory planning

---

## Validation

The simulator was tested using several predefined validation scenarios.

These scenarios verify that robot movement, interactions, trajectory generation and generated timelines remain consistent with the simulated physical state.

Validation data is available in:

```text
Data/Validation/
```

---

## What I Learned

This project gave me practical experience with:

- working on a larger C++ codebase,
- designing algorithms around geometric constraints,
- debugging simulation behavior,
- predicting continuous movement from discrete commands,
- detecting collisions and contacts,
- evaluating multiple possible future actions,
- integrating independently developed modules,
- working with Git in a team,
- understanding and maintaining code written by other developers.

One of the main challenges was ensuring that trajectory prediction remained consistent with the simulation, especially around collisions, particle contacts and very short command intervals.

---

## Project Context

This repository is a portfolio version of a university team project developed during the **HEIG-VD Summer HES 2026** program.

The original project was developed collaboratively by six students.

The complete application therefore contains contributions from several team members. The **trajectory simulation and planning subsystem** was my primary area of work.

Educational instructions, competition data and course material are intentionally not included in this repository.