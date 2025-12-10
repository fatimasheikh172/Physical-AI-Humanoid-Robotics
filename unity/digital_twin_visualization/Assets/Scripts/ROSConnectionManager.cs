using System;
using System.Collections;
using System.Linq;
using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class ROSConnectionManager : MonoBehaviour
{
    [Tooltip("IP address of the ROS machine. Enter '127.0.0.1' if running locally.")]
    public string rosIPAddress = "127.0.0.1";
    
    [Tooltip("Port number for the ROS TCP connection.")]
    public int rosPort = 10000;
    
    private ROSConnection ros;
    
    // Topics to subscribe/publish
    private string robotStateTopic = "/unity/robot_state";
    private string sensorDataTopic = "/unity/sensor_data";
    private string commandTopic = "/unity/commands";
    private string jointStatesTopic = "/unity/joint_states";

    // Robot transform reference
    public Transform robotTransform;

    // Robot model components for joint updates
    public Transform leftWheelTransform;
    public Transform rightWheelTransform;

    // Delegate for handling robot state updates
    public event Action<PoseMessage> OnRobotPoseReceived;
    public event Action<string> OnJointStatesReceived;
    
    // Start is called before the first frame update
    void Start()
    {
        // Get the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIPAddress, rosPort);

        // Subscribe to robot state topic
        ros.Subscribe<StringMsg>(robotStateTopic, OnRobotStateReceived);

        // Subscribe to joint states topic
        ros.Subscribe<StringMsg>(jointStatesTopic, OnJointStatesReceived);

        Debug.Log($"ROS Connection initialized to {rosIPAddress}:{rosPort}");
    }
    
    // Callback for when robot state is received from ROS
    private void OnRobotStateReceived(StringMsg message)
    {
        try
        {
            // Parse the JSON message containing robot state
            RobotStateData robotState = JsonUtility.FromJson<RobotStateData>(message.data);

            // Convert position and orientation from ROS coordinate system to Unity
            var unityPosition = new UnityEngine.Vector3(
                robotState.position.z,
                robotState.position.y,
                robotState.position.x
            );

            var unityRotation = new UnityEngine.Quaternion(
                robotState.orientation.z,
                robotState.orientation.y,
                robotState.orientation.x,
                robotState.orientation.w
            );

            // Update the robot transform if available
            if (robotTransform != null)
            {
                robotTransform.position = unityPosition;
                robotTransform.rotation = unityRotation;
            }

            // Trigger event for other components to handle
            OnRobotPoseReceived?.Invoke(new PoseMessage
            {
                position = new PointMsg(unityPosition.x, unityPosition.y, unityPosition.z),
                orientation = new QuaternionMsg(unityRotation.x, unityRotation.y, unityRotation.z, unityRotation.w)
            });
        }
        catch (Exception e)
        {
            Debug.LogError($"Error parsing robot state message: {e.Message}");
        }
    }

    // Callback for when joint states are received from ROS
    private void OnJointStatesReceived(StringMsg message)
    {
        try
        {
            // Parse the JSON message containing joint states
            JointStateData jointState = JsonUtility.FromJson<JointStateData>(message.data);

            // Update wheel joints if transforms are assigned
            if (leftWheelTransform != null)
            {
                // Update left wheel rotation based on joint position
                // Note: This is a simplified implementation; in practice, you'd use the exact joint position
                float rotation = jointState.jointStates.Find(js => js.name == "left_wheel_joint")?.position ?? 0f;
                leftWheelTransform.localRotation = UnityEngine.Quaternion.Euler(0, rotation * Mathf.Rad2Deg, 0);
            }

            if (rightWheelTransform != null)
            {
                // Update right wheel rotation based on joint position
                float rotation = jointState.jointStates.Find(js => js.name == "right_wheel_joint")?.position ?? 0f;
                rightWheelTransform.localRotation = UnityEngine.Quaternion.Euler(0, rotation * Mathf.Rad2Deg, 0);
            }

            // Trigger event for other components to handle
            OnJointStatesReceived?.Invoke(message.data);
        }
        catch (Exception e)
        {
            Debug.LogError($"Error parsing joint states message: {e.Message}");
        }
    }
    
    // Method to send commands to the robot
    public void SendVelocityCommand(float linearX, float angularZ)
    {
        var command = new StringMsg();
        var cmdData = new CommandData
        {
            type = "velocity_command",
            params = new VelocityParams
            {
                linear_x = linearX,
                linear_y = 0.0f,
                linear_z = 0.0f,
                angular_x = 0.0f,
                angular_y = 0.0f,
                angular_z = angularZ
            }
        };
        
        command.data = JsonUtility.ToJson(cmdData);
        ros.Publish(commandTopic, command);
        
        Debug.Log($"Sent command: linear_x={linearX}, angular_z={angularZ}");
    }
    
    // Method to send a general command
    public void SendCommand(string commandType, object parameters)
    {
        var command = new StringMsg();
        var cmdData = new CommandData
        {
            type = commandType,
            @params = parameters
        };
        
        command.data = JsonUtility.ToJson(cmdData);
        ros.Publish(commandTopic, command);
        
        Debug.Log($"Sent command: {commandType}");
    }
    
    // OnDestroy is called when the object is destroyed
    void OnDestroy()
    {
        if (ros != null)
        {
            ros.Close();
        }
    }
}

// Data classes for serialization/deserialization
[System.Serializable]
public class RobotStateData
{
    public PositionData position;
    public OrientationData orientation;
    public long[] timestamp; // [seconds, nanoseconds]
}

[System.Serializable]
public class PositionData
{
    public float x;
    public float y;
    public float z;
}

[System.Serializable]
public class OrientationData
{
    public float x;
    public float y;
    public float z;
    public float w;
}

[System.Serializable]
public class CommandData
{
    public string type;
    public object @params; // Using @ to avoid conflict with C# keyword
}

[System.Serializable]
public class VelocityParams
{
    public float linear_x;
    public float linear_y;
    public float linear_z;
    public float angular_x;
    public float angular_y;
    public float angular_z;
}

[System.Serializable]
public class JointStateData
{
    public JointState[] jointStates;
    public long[] timestamp; // [seconds, nanoseconds]
}

[System.Serializable]
public class JointState
{
    public string name;
    public float position;
    public float velocity;
    public float effort;
}

