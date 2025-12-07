# Communication Patterns in ROS 2: Topics, Services, and Actions

## Overview

ROS 2 provides three primary communication patterns for nodes to interact with each other:
1. **Topics** - Asynchronous, many-to-many communication using publish/subscribe pattern
2. **Services** - Synchronous, one-to-one request/response communication
3. **Actions** - Asynchronous communication for long-running tasks with feedback and goal management

Each pattern serves different use cases based on the communication requirements of your robotic system.

## Topics - Publish/Subscribe Pattern

Topics implement an asynchronous, many-to-many communication pattern where publishers send messages to a topic and subscribers receive messages from that topic.

### Key Characteristics:
- **Async**: Publishers and subscribers don't need to run simultaneously
- **Loose coupling**: Publishers don't know who subscribes to their topics
- **Many-to-many**: Multiple publishers can send to the same topic, multiple subscribers can receive from the same topic
- **Best for**: Continuous data streams like sensor data, robot state, etc.

### Example Use Cases:
- Sensor data publishing (camera feeds, LIDAR scans, IMU readings)
- Robot state broadcasting (joint positions, current pose)
- Log messages and diagnostics

### Quality of Service (QoS) Settings:
ROS 2 provides QoS settings to control how messages are delivered:

- **Reliability**: Reliable (all messages delivered) vs Best Effort (some messages may be lost)
- **Durability**: Volatile (new nodes don't get old messages) vs Transient Local (new nodes get the last value for each topic)
- **History**: Keep All (all messages kept) vs Keep Last (only the most recent messages kept)
- **Depth**: How many messages to keep when history is set to Keep Last

## Services - Request/Response Pattern

Services implement synchronous, one-to-one communication where a client sends a request and waits for a response from a server.

### Key Characteristics:
- **Sync**: Client waits for the server response
- **One-to-one**: One client talks to one server at a time
- **Request/Response**: Client sends request data, server returns response data
- **Best for**: Operations with clear completion and result, like configuration changes, transformations, etc.

### Example Use Cases:
- Map services requesting a transformation between coordinate frames
- Configuration services changing robot parameters
- Planning services that get a path for a given start and goal

## Actions - Long-Running Tasks with Feedback

Actions combine features of topics and services to handle long-running tasks that require feedback and the ability to cancel the task.

### Key Characteristics:
- **Async**: Non-blocking, allows other work while action is running
- **Goal/Result/Feedback**: Three-part communication pattern
- **Cancel capability**: Ability to cancel a running action
- **Status tracking**: Can check on the current status of an action
- **Best for**: Navigation, robot manipulator planning and execution, calibration procedures

### Example Use Cases:
- Navigation to a specific goal location
- Robot manipulator moving to a target position
- Calibration processes that take time and provide feedback

## Comparing the Communication Patterns

| Aspect | Topics | Services | Actions |
|--------|--------|----------|---------|
| Communication | Async, pub/sub | Sync, request/response | Async, with feedback |
| Participants | Many-to-many | One-to-one | One-to-one |
| Response | No response | Immediate response | Deferred response |
| Long-running | No | No | Yes |
| Feedback | No | No | Yes |
| Cancelation | No | No | Yes |

## QoS Configuration

Quality of Service (QoS) settings are crucial for different communication patterns, particularly for Topics:

- **Reliable vs Best Effort**: Use Reliable for critical data (robot control, collision avoidance), Best Effort for less critical data (video streams, some sensor data)
- **Transient Local vs Volatile**: Use Transient Local for data that new subscribers should get immediately (map data, initial parameters)
- **History and Depth**: Controls memory usage and data availability

## Implementation Guidelines

1. **Use Topics** when you need to continuously stream data to multiple subscribers
2. **Use Services** for operations with a clear start and end, where you need a response
3. **Use Actions** for long-running operations that benefit from feedback and cancelation capability
4. **Consider QoS settings** carefully based on the criticality and nature of the data being communicated

## Security Considerations

All ROS 2 communication patterns can be secured using ROS 2's security framework, which includes:
- Authentication of participants
- Access control for topics and services
- Encryption of message content

## Performance Considerations

- Topics: Consider the frequency and size of messages to avoid network congestion
- Services: Be aware that synchronous communication can block clients
- Actions: Consider the computational complexity of long-running tasks

Understanding these communication patterns and choosing the right one for your use case is crucial for designing efficient and reliable ROS 2 systems.