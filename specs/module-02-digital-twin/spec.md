# Module 02: Digital Twin for Robotic Systems

## Description

Build a working digital twin pipeline for robotic systems using Gazebo for physics-accurate simulation and Unity for high-fidelity rendering and human-robot interaction. Students will simulate dynamics, sensors (LiDAR, depth cameras, IMU), perception pipelines, and create a synchronized twin between Gazebo and Unity for visualization and human-in-the-loop testing.

## Duration

3 weeks (can be extended to 4 weeks for larger cohorts)

## Prerequisites

- Basic proficiency with Linux (Ubuntu 20.04/22.04)
- Familiarity with ROS 2 (topics, nodes, messages)
- Python programming (3.8+)
- Basic Unity knowledge (scene, GameObjects) desirable but not mandatory

## Learning Objectives (Measurable)

By the end of the module, each student should be able to:

1. Configure and run Gazebo simulations of ground/mobile robots and simple humanoid models.
2. Implement and tune physics parameters (mass, inertia, friction, collision meshes) to match real robot behaviour.
3. Simulate LiDAR, depth cameras, and IMUs in Gazebo and publish sensor data over ROS 2 topics.
4. Create a Unity scene that mirrors Gazebo state (poses, joint angles, sensor visuals) using a bridging mechanism.
5. Build a perception demo that consumes simulated sensor streams (e.g., pointcloud from LiDAR) and runs basic processing (e.g., obstacle detection, depth segmentation).
6. Evaluate differences between simulated and expected physical behaviour, and perform domain randomization to reduce sim-to-real gap.

## Tools & Software Stack

- Ubuntu 20.04/22.04
- ROS 2 (recommended: Humble or Rolling depending on course timeframe)
- Gazebo (classic) or Ignition Gazebo (specify which your infra supports)
- ros_gz / ros_ign bridge (if using Ignition), or ros_control plugins where required
- Unity (LTS version recommended) + Unity Robotics Hub or ROS# for bridging
- Python 3.8+, rclpy, numpy, open3d (optional)
- RViz2 for ROS visualisation
- Git & GitHub/GitLab for submissions

## Module Structure (Week-by-week)

### Week 1 — Foundations: Gazebo physics & sensors

- **Lecture (theory)**: Rigid-body dynamics, collision shapes, joint types, contact/friction parameters, and solver settings.
- **Lab 1**: Launch a simple differential-drive robot in Gazebo, inspect URDF/ SDF, tune mass and friction to match expected response.
- **Lab 2**: Add and configure sensors: LiDAR (scan), depth camera, IMU. Publish sensor topics and view them in RViz2.
- **Mini-assignment**: Produce a short report (2 pages) showing how changes in friction/inertia affected wheel slip and stopping distance (include recorded plots).

### Week 2 — Unity integration & visualization

- **Lecture (theory)**: Unity rendering pipeline, transforms and coordinate frames, Unity physics vs Gazebo physics, network/bridge patterns.
- **Lab 3**: Set up Unity scene, import robot model (URDF/SDF conversion or FBX), and animate based on published ROS topics.
- **Lab 4**: Implement a simple bridge (Unity Robotics Hub / ROS#) to subscribe to robot pose and joint states.
- **Mini-assignment**: Create a Unity scene that mirrors Gazebo's robot pose in real-time with correct coordinate transforms. Submit short demo video (1–2 min).

### Week 3 — Perception, sim-to-real, and capstone

- **Lecture (theory)**: Sensor noise models, domain randomization, calibration, timing and synchronization, clock drift.
- **Lab 5**: Build a perception node that consumes Gazebo LiDAR or depth camera and performs simple obstacle detection or ground plane extraction.
- **Lab 6 (Integration)**: Run the full pipeline: Gazebo simulation → ROS 2 topics → Unity visualization → perception node reacts and issues commands back to simulation.
- **Final Project (capstone)**: Students build a digital twin for a selected platform (differential drive robot, manipulator arm, or humanoid subset) demonstrating: accurate physics tuning, simulated sensors, Unity visualization, and a perception/control demo. Include quantitative metrics comparing expected vs simulated behavior.

## SP.Tasks (Concise, Advanced)

Each task below is small but designed to be advanced and meaningful for an automated app or grading pipeline.

- **task-gz-physics-tune** — Given a URDF and target stopping-distance/response curve, write a script that sweeps friction and damping parameters in Gazebo, runs short simulations, and reports the parameter set that minimizes the error to target metrics.
- **task-lidar-to-pcd** — Subscribe to /scan or /points and convert LiDAR frames to timestamped PCD files, then compute per-frame density and coverage metrics.
- **task-unity-bridge** — Implement a Unity subscriber that receives geometry_msgs/PoseArray and spawns/updates GameObjects with correct orientation and scale. Provide a small unit test that asserts transforms match a ground-truth Gazebo publisher.
- **task-noise-model** — Implement a configurable noise-model wrapper that injects Gaussian and dropout noise into sensor streams. Provide CLI flags for mean/std, dropout probability, and seed.
- **task-sim2real-eval** — Run domain randomization across visual textures and physics params (N runs), collect performance of a basic controller (e.g., waypoint following), and compute sim-to-real transferability score.

## Assessment & Deliverables

- Labs & Mini-assignments: 40% (each lab graded on completeness, accuracy, and reports)
- Final Project (capstone): 50% (includes code, recorded demo, and a written report — see rubric below)
- Participation / Code reviews: 10%

## Final Project Rubric (suggested)

- Simulation fidelity & physics tuning — 25%
- Sensor realism & correct ROS integration — 20%
- Unity visualization correctness & UX — 15%
- Perception/control demo (functionality) — 25%
- Documentation, reproducibility, and tests — 15%

## Deliverables (for each student/team)

A Git repository with:

- Launch files (Gazebo + ROS 2)
- Unity project or package with bridge code
- Scripts for sensors and perception
- Automated test script that runs a short sim and validates expected outputs
- Demo video (3–5 minutes) showing the full pipeline
- Written report (4–6 pages) with quantitative results and discussion of sim-to-real strategies

## Example Capstone Ideas

- **Mobile Robot Twin**: Create a twin for a TurtleBot-like platform with LiDAR-based obstacle avoidance and Unity visualization.
- **Arm Manipulation Twin**: Simulate a 6-DOF manipulator performing pick-and-place with collision checking and depth-camera grasp planning.
- **Human-Robot Interaction Scene**: Use Unity to simulate a human avatar interacting with a mobile robot; perception is depth-camera based.

## Instructor Notes & Extensions

- Encourage students to containerize Gazebo/ROS components for reproducibility (Docker images with X11/virtual display or use headless rendering for CI).
- For larger classes, prepare pre-built VM or cloud instances with ROS 2, Gazebo, and Unity preinstalled.
- Optionally add an extra week focused on bridging to real hardware for students with access to robots.

## Recommended Readings & Resources

- ROS 2 documentation (nodes, topics, tf2)
- Gazebo / Ignition tutorials on sensors & physics
- Unity Robotics Hub documentation and examples
- Papers on domain randomization and sim-to-real transfer

## Quick Setup Checklist (for students)

- Install ROS 2 (course-specified distro). Configure environment.
- Install Gazebo (or Ignition) matching ROS distro compatibility.
- Clone course repo and test ros2 launch demo.
- Install Unity LTS and Unity Robotics Hub package (or ROS# as alternative).
- Run the provided smoke-test launch to confirm end-to-end topic flow.
