# Week 2 Assessment: Humanoid Structure and Sensor Systems

## Multiple Choice Questions (2 points each)

1. What is the primary purpose of an IMU (Inertial Measurement Unit) in a humanoid robot?
   a) To provide visual information about the environment
   b) To measure acceleration, angular velocity, and orientation for balance control
   c) To detect objects at long range
   d) To enable speech recognition

2. Which sensor system is most appropriate for creating 3D maps of an environment?
   a) RGB camera only
   b) Microphone array
   c) LiDAR (Light Detection and Ranging)
   d) Force/torque sensors

3. What does DOF stand for in the context of humanoid robotics?
   a) Degree of Functionality
   b) Dynamic Operating Field
   c) Degrees of Freedom
   d) Digital Operation Format

4. Which sensor is most important for safe object manipulation by a humanoid robot?
   a) LiDAR
   b) Force/torque sensors
   c) IMU
   d) GPS

5. What is the main advantage of using multiple sensors rather than a single sensor type?
   a) Lower cost
   b) Sensor fusion allows for more robust perception than individual sensors
   c) Less computational requirements
   d) Simpler programming

6. Depth cameras are particularly useful for:
   a) Long-range navigation only
   b) Detecting sound sources
   c) Providing 3D spatial information for manipulation tasks
   d) Measuring temperature

7. What is a key challenge with LiDAR sensors?
   a) They only work in bright light
   b) They cannot detect transparent or highly absorbent objects
   c) They consume very little power
   d) They provide only 2D information

8. In humanoid robots, what does "dynamic balance" refer to?
   a) The ability to run fast
   b) Maintaining balance while moving and interacting with the environment
   c) The ability to lift heavy objects
   d) The ability to speak multiple languages

9. Which sensors are most important for the humanoid robot to interact safely with humans?
   a) Vision systems only
   b) Force/torque sensors and joint encoders for compliant control
   c) GPS for location tracking
   d) Microphones for voice commands

10. What is the field of view (FOV) typically important for in camera systems?
    a) Power consumption
    b) The extent of the scene that can be captured
    c) Color accuracy
    d) Weight of the sensor

## Short Answer Questions (10 points each)

11. Explain the difference between a servo motor and a regular DC motor in the context of humanoid robotics. What advantages do servo motors provide for humanoid robots?

12. Describe the concept of sensor fusion in humanoid robotics. Why is sensor fusion important, and provide an example of how two different sensors might work together to achieve a task.

13. What are the main advantages of a humanoid form factor compared to other robot designs (wheeled, tracked, or specialized robots) for human-centered environments? What are also the main disadvantages?

## Practical Application Questions (20 points each)

14. Design a simple sensor system for a humanoid robot that needs to pour a cup of coffee. List the minimum sensors required and explain how each sensor contributes to the successful completion of this task. Consider navigation to the coffee station, detecting the coffee pot, pouring motion, and avoiding spills.

15. Consider a humanoid robot that must navigate through a crowded room where people are walking around. Identify the key sensing requirements and explain how different sensors would work together to ensure safe navigation without bumping into people. What are the specific challenges in this scenario?

## Essay Question (25 points)

16. Discuss the trade-offs involved in sensor selection for humanoid robots. Consider factors such as cost, weight, power consumption, computational requirements, environmental robustness, and functional capabilities. How would you prioritize these factors differently for a research humanoid robot versus a commercial assistant robot? Support your answer with specific examples of sensor systems.

---

## Answer Key

**Multiple Choice Answers:**
1. b) To measure acceleration, angular velocity, and orientation for balance control
2. c) LiDAR (Light Detection and Ranging)
3. c) Degrees of Freedom
4. b) Force/torque sensors
5. b) Sensor fusion allows for more robust perception than individual sensors
6. c) Providing 3D spatial information for manipulation tasks
7. b) They cannot detect transparent or highly absorbent objects
8. b) Maintaining balance while moving and interacting with the environment
9. b) Force/torque sensors and joint encoders for compliant control
10. b) The extent of the scene that can be captured

**Short Answer Guide:**
11. Servo motors provide precise control of position, velocity, and acceleration with integrated feedback systems, making them ideal for humanoid robots that require accurate joint control. DC motors typically require external feedback systems to achieve similar precision.
12. Sensor fusion is the combination of data from multiple sensors to create a more complete and accurate understanding than any single sensor could provide. Example: A camera identifies an object while a range sensor provides distance information.
13. Advantages: Can use human-designed environments and tools, natural interaction with humans. Disadvantages: Mechanical complexity, stability challenges, higher cost.

**Grading Rubric for Practical and Essay Questions:**

Question 14 (20 points):
- Appropriate sensor selection (8 points)
- Clear explanation of each sensor's role (8 points)
- Understanding of sensor integration (4 points)

Question 15 (20 points):
- Identification of key sensing requirements (6 points)
- Explanation of sensor integration (6 points)
- Recognition of specific challenges (4 points)
- Logical reasoning (4 points)

Question 16 (25 points):
- Discussion of trade-offs (10 points)
- Consideration of multiple factors (5 points)
- Differentiation between research and commercial applications (5 points)
- Support with specific examples (5 points)