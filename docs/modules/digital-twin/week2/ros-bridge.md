# Week 2: ROS Bridge Implementation

## Overview

In this section, you'll implement the ROS-TCP bridge that connects your Gazebo simulation with Unity visualization. This bridge enables real-time synchronization of robot states, sensor data, and commands between the physics simulation and the visual representation.

## Learning Objectives

By the end of this section, you will be able to:

- Implement ROS-TCP communication between Gazebo and Unity
- Subscribe to robot pose and joint states in Unity
- Synchronize transforms between Gazebo and Unity coordinate systems
- Create a stable bidirectional communication channel

## Understanding the ROS-Unity Bridge Architecture

### Communication Flow

The ROS-Unity bridge follows this architecture:

1. **Gazebo Simulation**: Runs the physics simulation and publishes robot states and sensor data
2. **ROS TCP Endpoint**: Acts as a server or client to bridge ROS 2 messages to TCP
3. **Unity ROS Connector**: Receives TCP messages and converts them to Unity objects/components
4. **Unity Visualization**: Updates the visual representation based on received data

### Key Messages for Synchronization

- `nav_msgs/Odometry`: Robot position and orientation
- `tf2_msgs/TFMessage`: Coordinate transforms between frames
- `sensor_msgs/JointState`: Joint positions of the robot
- `geometry_msgs/TransformStamped`: Individual frame transformations

## Setting Up the ROS TCP Endpoint

### Installing ROS TCP Endpoint

First, install the ROS TCP endpoint package:

```bash
# For ROS 2 Humble
sudo apt update
sudo apt install ros-humble-ros-tcp-endpoint
```

### Running the ROS TCP Endpoint Server

```bash
# Start the ROS TCP endpoint server
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

## Implementing Unity Subscriber for Robot States

### Creating the ROS Subscriber Script

Create a Unity C# script to receive robot pose and joint states:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class RobotStateSubscriber : MonoBehaviour
{
    [SerializeField]
    private GameObject robotBase; // The base link of your robot
    [SerializeField]
    private Dictionary<string, GameObject> jointObjects = new Dictionary<string, GameObject>();
    
    private ROSConnection ros;
    
    // Topics to subscribe to
    private string odomTopic = "/odom";
    private string jointStatesTopic = "/joint_states";
    
    void Start()
    {
        ros = ROSConnection.instance;
        
        // Subscribe to topics
        ros.Subscribe<OdometryMsg>(odomTopic, OdomCallback);
        ros.Subscribe<JointStateMsg>(jointStatesTopic, JointStatesCallback);
    }
    
    void OdomCallback(OdometryMsg odom)
    {
        // Extract position and orientation
        Vector3 position = new Vector3(
            (float)odom.pose.pose.position.x,
            (float)odom.pose.pose.position.z, // Swap Y and Z for coordinate conversion
            -(float)odom.pose.pose.position.y // Negate for coordinate conversion
        );
        
        // Convert quaternion from ROS to Unity coordinates
        Quaternion rotation = new Quaternion(
            -(float)odom.pose.pose.orientation.x, // Negate for coordinate conversion
            -(float)odom.pose.pose.orientation.z, // Swap Y and Z for coordinate conversion
            (float)odom.pose.pose.orientation.y,  // Swap Y and Z for coordinate conversion
            (float)odom.pose.pose.orientation.w
        );
        
        // Apply the transformation to the robot base
        robotBase.transform.position = position;
        robotBase.transform.rotation = rotation;
    }
    
    void JointStatesCallback(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Array.Length; i++)
        {
            string jointName = jointState.name.Array[i];
            float jointPosition = (float)jointState.position.Array[i];
            
            if (jointObjects.ContainsKey(jointName))
            {
                GameObject jointObject = jointObjects[jointName];
                
                // Example: rotate a revolute joint around its axis
                // The exact rotation depends on your joint type and axis
                jointObject.transform.localRotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
            }
        }
    }
}
```

### Coordinate System Conversion

ROS and Unity use different coordinate systems. The standard conversion is:
- ROS: X-forward, Y-left, Z-up
- Unity: X-right, Y-up, Z-forward

The conversion function maps ROS coordinates to Unity coordinates:
```csharp
Vector3 RosToUnityCoordinates(Vector3 rosPosition)
{
    return new Vector3(
        rosPosition.y,      // Y becomes X
        rosPosition.z,      // Z becomes Y
        rosPosition.x       // X becomes Z
    );
}

Quaternion RosToUnityRotation(Quaternion rosQuaternion)
{
    return new Quaternion(
        rosQuaternion.y,    // Y becomes X
        rosQuaternion.z,    // Z becomes Y
        rosQuaternion.x,    // X becomes Z
        rosQuaternion.w     // W stays W
    );
}
```

## Implementing the Transform Synchronization

### TF (Transform) System Implementation

Since Unity doesn't have a native TF system, we need to implement our own:

```csharp
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Tf2;

public class TransformSynchronizer : MonoBehaviour
{
    private ROSConnection ros;
    private Dictionary<string, TransformData> transforms = new Dictionary<string, TransformData>();
    private string tfTopic = "/tf";
    
    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<TFMessage>(tfTopic, TfCallback);
    }
    
    void TfCallback(TFMessage tfMsg)
    {
        foreach (var transform in tfMsg.transforms)
        {
            // Process each transform in the message
            ProcessTransform(transform);
        }
    }
    
    void ProcessTransform(TransformStampedMsg tf)
    {
        string childFrame = tf.child_frame_id;
        string parentFrame = tf.header.frame_id;
        
        Vector3 position = new Vector3(
            (float)tf.transform.translation.x,
            (float)tf.transform.translation.z,  // Convert coordinate system
            -(float)tf.transform.translation.y  // Convert coordinate system
        );
        
        Quaternion rotation = new Quaternion(
            -(float)tf.transform.rotation.x,    // Convert coordinate system
            -(float)tf.transform.rotation.z,    // Convert coordinate system
            (float)tf.transform.rotation.y,     // Convert coordinate system
            (float)tf.transform.rotation.w
        );
        
        // Store the transform data
        transforms[childFrame] = new TransformData
        {
            position = position,
            rotation = rotation,
            parentFrame = parentFrame
        };
        
        // Apply transform to the corresponding Unity object if it exists
        ApplyTransformToGameObject(childFrame, position, rotation);
    }
    
    void ApplyTransformToGameObject(string frameName, Vector3 position, Quaternion rotation)
    {
        // Find the corresponding Unity GameObject
        GameObject go = GameObject.Find(frameName);
        if (go != null)
        {
            go.transform.position = position;
            go.transform.rotation = rotation;
        }
    }
}

[System.Serializable]
public class TransformData
{
    public Vector3 position;
    public Quaternion rotation;
    public string parentFrame;
}
```

## Testing the Bridge Connection

### Creating a Test Script

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class BridgeHealthCheck : MonoBehaviour
{
    private ROSConnection ros;
    private float lastMessageTime;
    private bool isConnected = false;
    
    [SerializeField] private float timeoutSeconds = 5.0f;
    
    void Start()
    {
        ros = ROSConnection.instance;
        lastMessageTime = Time.time;
    }
    
    void Update()
    {
        // Check if we've received a message recently
        if (Time.time - lastMessageTime > timeoutSeconds)
        {
            isConnected = false;
            Debug.LogWarning("Bridge connection timeout - no messages received recently");
        }
        else if (!isConnected)
        {
            isConnected = true;
            Debug.Log("Bridge connection established");
        }
    }
    
    // Call this method when receiving any ROS message to update connection status
    public void MessageReceived()
    {
        lastMessageTime = Time.time;
    }
}
```

### Testing Connection with ROS Commands

1. In a terminal, verify topics are publishing:
```bash
# Check available topics
ros2 topic list

# Echo odom topic to verify it's publishing
ros2 topic echo /odom

# Echo joint states topic
ros2 topic echo /joint_states
```

2. In Unity, use the debug console to verify messages are being received.

## Troubleshooting Common Issues

### Connection Issues
- **Problem**: Unity cannot connect to ROS
- **Solution**: 
  - Verify ROS TCP endpoint is running before starting Unity
  - Check IP addresses match between Unity and ROS
  - Ensure firewall allows communication on the specified port

### Coordinate System Issues
- **Problem**: Robot appears rotated incorrectly in Unity
- **Solution**:
  - Apply the correct coordinate transform as shown above
  - Check if Gazebo is using the correct reference frame

### Message Type Issues
- **Problem**: Unity receives messages but can't parse them correctly
- **Solution**:
  - Ensure Unity Robotics packages are properly installed
  - Verify message definitions match between ROS and Unity

## Performance Optimization

### Message Throttling
For high-frequency topics, consider throttling the rate at which Unity processes updates:

```csharp
private float lastUpdateTime = 0f;
private float updateInterval = 0.1f; // Update every 0.1 seconds

void OdomCallback(OdometryMsg odom)
{
    if (Time.time - lastUpdateTime > updateInterval)
    {
        UpdateRobotPosition(odom);
        lastUpdateTime = Time.time;
    }
}
```

### Visual Optimization
- Use object pooling for frequently updated visual elements
- Consider using Unity's Job System for transform updates
- Implement Level of Detail (LOD) for complex robot models when far from camera

## Next Steps

After implementing the ROS bridge, your Unity visualization should sync with your Gazebo simulation. The next step is to enhance the visualization with additional features like camera controls and telemetry displays.