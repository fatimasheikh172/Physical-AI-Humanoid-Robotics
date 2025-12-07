
---
id: 001
title: Module 1: ROS 2 Plan
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: robotic-nervous-system
branch: 001-physical-ai-textbook
user: unknown
command: sp.plan
labels: ["robotics", "ROS2", "education", "curriculum", "plan"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/robotic-nervous-system/001-module-1-ros2-plan.plan.prompt.md
  - specs/001-physical-ai-textbook/plan.md
tests:

---

## Prompt

```
sp.plan

MODULE 1 NAME:
The Robotic Nervous System (ROS 2)

MODULE DURATION:
3 Weeks (Weeks 3–5 of the Quarter)

MODULE GOAL:
By the end of this module, the student must be able to:
- Understand ROS 2 architecture
- Build custom ROS 2 nodes using Python
- Establish communication using topics, services, and actions
- Control a simulated robot using ROS 2
- Bridge Python AI agents with ROS controllers using rclpy
- Understand and use URDF for humanoid robot structure

==================================================
WEEK 1 PLAN – ROS 2 FOUNDATIONS

LEARNING OBJECTIVES:
- Understand what ROS 2 is and why it is called the “Robotic Nervous System”
- Learn ROS 2 architecture and middleware concept (DDS)
- Understand Nodes and Message Passing
- Setup complete ROS 2 development environment

THEORY CONTENT TO DEVELOP:
- Introduction to ROS vs ROS 2
- ROS 2 architecture overview
- DDS middleware explanation
- Nodes, Topics, Publishers, Subscribers
- Real-world robotics communication examples

HANDS-ON LAB TASKS:
- Install Ubuntu 22.04
- Install ROS 2 Humble or Iron
- Setup ROS 2 workspace
- Run standard ROS 2 demo nodes (talker/listener)

WEEK 1 MINI PROJECT:
- Create your first custom ROS 2 Python node
- Publish random sensor data on a topic
- Subscribe and visualize the data

WEEK 1 ASSESSMENT:
- MCQs on ROS 2 core concepts
- Practical task: Node creation + topic communication
AI TUTOR INTEGRATION:
- AI explains ROS terms in beginner language
- AI auto-debugger for ROS install errors

==================================================
WEEK 2 PLAN – COMMUNICATION LAYER (TOPICS, SERVICES, ACTIONS)

LEARNING OBJECTIVES:
- Deep understanding of Topics, Services, and Actions
- Learn async communication in robots
- Understand message types and interfaces
- Learn parameter server usage

THEORY CONTENT TO DEVELOP:
- Topics vs Services vs Actions comparison
- Message structures
- QoS (Quality of Service)
- Parameters and configuration

HANDS-ON LAB TASKS:
- Create topic-based robot motion control
- Create service-based robot state change
- Create action-based navigation command
- Tune QoS parameters

WEEK 2 MINI PROJECT:
- Build a ROS 2-based remote robot controller using:
  - Topic for movement
  - Service for start/stop
  - Action for goal reaching

WEEK 2 ASSESSMENT:
- Practical ROS communication system test
- Debugging challenge

AI TUTOR INTEGRATION:
AI explains ROS terms in beginner language
- AI auto-debugger for ROS install errors

==================================================
WEEK 2 PLAN – COMMUNICATION LAYER (TOPICS, SERVICES, ACTIONS)

LEARNING OBJECTIVES:
- Deep understanding of Topics, Services, and Actions
- Learn async communication in robots
- Understand message types and interfaces
- Learn parameter server usage

THEORY CONTENT TO DEVELOP:
- Topics vs Services vs Actions comparison
- Message structures
- QoS (Quality of Service)
- Parameters and configuration

HANDS-ON LAB TASKS:
- Create topic-based robot motion control
- Create service-based robot state change
- Create action-based navigation command
- Tune QoS parameters

WEEK 2 MINI PROJECT:
- Build a ROS 2-based remote robot controller using:
  - Topic for movement
  - Service for start/stop
  - Action for goal reaching

WEEK 2 ASSESSMENT:
- Practical ROS communication system test
- Debugging challenge

AI TUTOR INTEGRATION:
- AI reviews message structures
- AI suggests correct QoS based on use case

==================================================
WEEK 3 PLAN – AI + ROS BRIDGE & HUMANOID STRUCTURE

LEARNING OBJECTIVES:
- Connect Python AI agents to ROS 2 using rclpy
- Control a robot using AI decisions
- Understand URDF for humanoid robot design
- Visualize robot structure using RViz & Gazebo

THEORY CONTENT TO DEVELOP:
- rclpy architecture
- AI-to-ROS control pipelines
- Introduction to URDF
- Joints, Links, Sensors in URDF
- Humanoid robot skeleton design

HANDS-ON LAB TASKS:
- Create an AI decision node
- Send ROS commands from AI agent
- Load URDF in RViz
- Simulate joints and sensors

WEEK 3 MINI PROJECT:
- AI-controlled virtual robot that:
  - Takes commands from Python AI agent
  - Moves using ROS 2
  - Visualizes structure in RViz

FINAL MODULE 1 ASSESSMENT:
- Theory test
- Practical robotic control test
- URDF design evaluation
- AI + ROS integration evaluation

==================================================
MODULE 1 OUTPUT DELIVERABLES:

- Fully written Module 1 textbook chapter
- Working ROS 2 labs
- AI-controlled ROS 2 demo
- Custom URDF humanoid skeleton
- ROS 2 quiz & practical assessment
- AI Tutor support enabled

==================================================
MODULE 1 SUCCESS CRITERIA:

- Student successfully builds multiple ROS 2 nodes
- Student controls a robot using ROS 2
- Student connects AI logic to ROS controllers
- Student understands humanoid robot digital structure
```

## Response snapshot

Creating PHR for Module 1 plan and then writing the plan to `specs/001-physical-ai-textbook/plan.md`.

## Outcome

- ✅ Impact: The plan for Module 1 has been recorded.
- 🧪 Tests: N/A
- 📁 Files: PHR file created.
- 🔁 Next prompts: Next, I will create the plan file for the module.
- 🧠 Reflection: Recorded the initial module plan for ROS 2.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
