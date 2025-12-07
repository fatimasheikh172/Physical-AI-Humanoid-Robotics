# ROS 2 Architecture Overview

## The DDS Middleware

The core of ROS 2 is built on DDS (Data Distribution Service), which serves as the communication middleware:

- **Data-Centricity**: Unlike traditional message-passing systems, DDS is data-centric. This means that nodes interact with a "data space" rather than directly with each other.
- **Quality of Service (QoS)**: DDS provides rich QoS policies that control how data is communicated, including reliability, durability, liveliness, and deadline policies.
- **Discovery**: Automatic peer-to-peer discovery of participants in the system.

## Node Graph and Data Flow

In ROS 2, the system is composed of nodes that communicate through:

- **Topics**: Asynchronous, many-to-many communication using publish/subscribe pattern
- **Services**: Synchronous, one-to-one request/response communication
- **Actions**: Asynchronous communication for long-running tasks with feedback

## Message Types and Serialization

ROS 2 uses IDL (Interface Definition Language) to define message types, supporting:

- Standard types like string, bool, int, float
- Complex nested message definitions
- Array and constant definitions
- Service and action definitions

## QoS Basics and Reliability Modes

Quality of Service (QoS) settings determine how messages are delivered:

- **Reliability**: Reliable (messages guaranteed to be delivered) or Best Effort (messages may be lost)
- **Durability**: Volatile (new nodes don't get old messages) or Transient Local (new nodes get the last value for each topic)
- **History**: Keep All (all messages kept) or Keep Last (only the most recent messages kept)