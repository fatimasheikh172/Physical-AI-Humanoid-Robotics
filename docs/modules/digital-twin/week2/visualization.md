# Week 2: Visualization

## Overview

In this section, you'll enhance your Unity visualization with camera controls, UI elements, and advanced rendering features. This creates an immersive and informative view of your digital twin system that can be used for both monitoring and human-in-the-loop testing.

## Learning Objectives

By the end of this section, you will be able to:

- Implement camera controls for navigating the Unity scene
- Create UI elements to display telemetry data
- Set up advanced rendering features for high-fidelity visualization
- Optimize visualization performance

## Camera Controls Implementation

### Creating Multiple Camera Views

Add several camera perspectives to your scene:

1. **Fixed Overhead Camera**: Provides a complete view of the environment
2. **Follow Camera**: Follows the robot with configurable offset
3. **First-Person Camera**: Shows the robot's perspective
4. **Sensor Visualization Cameras**: Shows sensor-specific views (optional)

### Follow Camera Script

```csharp
using UnityEngine;

[RequireComponent(typeof(Camera))]
public class FollowCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform target;
    
    [Header("Camera Settings")]
    public Vector3 offset = new Vector3(0f, 10f, -10f);
    [Range(0.1f, 2f)]
    public float smoothFactor = 0.5f;
    
    [Header("Constraints")]
    public float minDistance = 5f;
    public float maxDistance = 30f;
    public float minHeight = 5f;
    public float maxHeight = 50f;
    
    private Vector3 velocity = Vector3.zero;
    private Camera cam;
    
    void Start()
    {
        cam = GetComponent<Camera>();
    }
    
    void LateUpdate()
    {
        if (target == null) return;
        
        // Calculate desired position
        Vector3 desiredPosition = target.position + offset;
        
        // Smoothly move towards the desired position
        Vector3 smoothedPosition = Vector3.SmoothDamp(transform.position, desiredPosition, ref velocity, smoothFactor);
        
        // Apply constraints
        smoothedPosition.y = Mathf.Clamp(smoothedPosition.y, minHeight, maxHeight);
        
        // Calculate distance to target
        float distanceToTarget = Vector3.Distance(target.position, smoothedPosition);
        
        if (distanceToTarget > maxDistance)
        {
            // Pull camera closer to target
            Vector3 direction = (target.position - smoothedPosition).normalized;
            smoothedPosition = target.position - direction * maxDistance;
        }
        else if (distanceToTarget < minDistance)
        {
            // Push camera away from target
            Vector3 direction = (target.position - smoothedPosition).normalized;
            smoothedPosition = target.position - direction * minDistance;
        }
        
        transform.position = smoothedPosition;
        
        // Look at the target
        transform.LookAt(target);
    }
}
```

### Camera Switching Script

```csharp
using UnityEngine;

public class CameraSwitcher : MonoBehaviour
{
    [SerializeField] private Camera[] cameras;
    private int currentCameraIndex = 0;
    
    void Start()
    {
        if (cameras.Length > 0)
        {
            ActivateCamera(currentCameraIndex);
        }
    }
    
    void Update()
    {
        // Switch cameras using number keys
        for (int i = 1; i <= cameras.Length; i++)
        {
            if (Input.GetKeyDown(KeyCode.Alpha0 + i))
            {
                SwitchToCamera(i - 1);
                break;
            }
        }
        
        // Alternative: Use arrow keys for next/previous
        if (Input.GetKeyDown(KeyCode.RightArrow))
        {
            SwitchToNextCamera();
        }
        else if (Input.GetKeyDown(KeyCode.LeftArrow))
        {
            SwitchToPreviousCamera();
        }
    }
    
    public void SwitchToCamera(int index)
    {
        if (index >= 0 && index < cameras.Length)
        {
            currentCameraIndex = index;
            ActivateCamera(currentCameraIndex);
        }
    }
    
    private void SwitchToNextCamera()
    {
        currentCameraIndex = (currentCameraIndex + 1) % cameras.Length;
        ActivateCamera(currentCameraIndex);
    }
    
    private void SwitchToPreviousCamera()
    {
        currentCameraIndex = (currentCameraIndex - 1 + cameras.Length) % cameras.Length;
        ActivateCamera(currentCameraIndex);
    }
    
    private void ActivateCamera(int index)
    {
        for (int i = 0; i < cameras.Length; i++)
        {
            cameras[i].gameObject.SetActive(i == index);
        }
    }
}
```

## Telemetry Display UI

### Creating a Telemetry Panel

1. Create a Canvas in your scene (GameObject > UI > Canvas)
2. Add UI elements to display robot information:
   - Text for position and orientation
   - Text for sensor values
   - Text for velocity
   - Images for sensor data visualization

### Telemetry Display Script

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;

public class TelemetryDisplay : MonoBehaviour
{
    [Header("UI References")]
    public Text positionText;
    public Text orientationText;
    public Text velocityText;
    public Text jointStateText;
    public Text sensorDataText;
    
    [Header("ROS Settings")]
    public string odomTopic = "/odom";
    public string jointStatesTopic = "/joint_states";
    public string laserScanTopic = "/scan";
    
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.instance;
        
        // Subscribe to ROS topics
        ros.Subscribe<OdometryMsg>(odomTopic, OdomCallback);
        ros.Subscribe<JointStateMsg>(jointStatesTopic, JointStateCallback);
        ros.Subscribe<LaserScanMsg>(laserScanTopic, LaserScanCallback);
    }
    
    void OdomCallback(OdometryMsg odom)
    {
        // Update position display
        if (positionText != null)
        {
            positionText.text = $"Position: X={(float)odom.pose.pose.position.x:F2}, Y={(float)odom.pose.pose.position.y:F2}, Z={(float)odom.pose.pose.position.z:F2}";
        }
        
        // Update orientation display
        if (orientationText != null)
        {
            // Convert quaternion to Euler angles
            var q = new Quaternion(
                (float)odom.pose.pose.orientation.x,
                (float)odom.pose.pose.orientation.y,
                (float)odom.pose.pose.orientation.z,
                (float)odom.pose.pose.orientation.w
            );
            Vector3 eulerAngles = q.eulerAngles;
            
            orientationText.text = $"Orientation: R={eulerAngles.x:F1}°, P={eulerAngles.y:F1}°, Y={eulerAngles.z:F1}°";
        }
        
        // Update velocity display
        if (velocityText != null)
        {
            velocityText.text = $"Velocity: Lin X={(float)odom.twist.twist.linear.x:F2}, Ang Z={(float)odom.twist.twist.angular.z:F2}";
        }
    }
    
    void JointStateCallback(JointStateMsg jointState)
    {
        if (jointStateText != null)
        {
            string jointInfo = "Joints:\n";
            for (int i = 0; i < Mathf.Min(5, jointState.name.Array.Length); i++) // Show first 5 joints
            {
                jointInfo += $"{jointState.name.Array[i]}: {(float)jointState.position.Array[i]:F3} rad\n";
            }
            jointStateText.text = jointInfo;
        }
    }
    
    void LaserScanCallback(LaserScanMsg scan)
    {
        if (sensorDataText != null)
        {
            int validRanges = 0;
            float minDistance = float.MaxValue;
            
            foreach (var range in scan.ranges)
            {
                if (scan.range_min <= range && range <= scan.range_max)
                {
                    validRanges++;
                    if (range < minDistance)
                    {
                        minDistance = range;
                    }
                }
            }
            
            sensorDataText.text = $"Scan: {validRanges}/{scan.ranges.Length} valid, Min dist: {minDistance:F2}m";
        }
    }
}
```

## Advanced Rendering Features

### Implementing Environment Effects

1. **Realistic Lighting**:
   - Add a Directional Light to represent the sun
   - Adjust the light's properties to match the Gazebo environment
   - Consider using Realtime Global Illumination for dynamic lighting

2. **Post-Processing Effects**:
   - Add a Post-process Volume to the main camera
   - Include effects like Bloom, Color Grading, and Ambient Occlusion
   - Balance visual quality with performance

### Sensor Visualization

Create visualizations for different sensor types:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SensorVisualizer : MonoBehaviour
{
    [SerializeField] private GameObject robotBase;
    [SerializeField] private GameObject lidarVisualization;
    [SerializeField] private float lidarRange = 30f;
    [SerializeField] private int lidarResolution = 360;
    
    private List<GameObject> lidarPoints = new List<GameObject>();
    
    void Start()
    {
        InitializeLidarVisualization();
    }
    
    void InitializeLidarVisualization()
    {
        // Create visualization points for lidar
        for (int i = 0; i < lidarResolution; i++)
        {
            GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            point.transform.SetParent(lidarVisualization.transform);
            point.transform.localScale = Vector3.one * 0.05f;
            point.GetComponent<Renderer>().material.color = Color.red;
            point.SetActive(false);
            lidarPoints.Add(point);
        }
    }
    
    // Method to update lidar visualization based on scan data
    public void UpdateLidarVisualization(float[] ranges, float angleMin, float angleIncrement)
    {
        for (int i = 0; i < Mathf.Min(ranges.Length, lidarPoints.Count); i++)
        {
            float range = ranges[i];
            
            if (range >= 0.1f && range <= lidarRange) // Valid range
            {
                Vector3 position = new Vector3(
                    range * Mathf.Cos(angleMin + i * angleIncrement),
                    0.1f, // Slightly above ground
                    range * Mathf.Sin(angleMin + i * angleIncrement)
                );
                
                lidarPoints[i].transform.position = robotBase.transform.position + position;
                lidarPoints[i].SetActive(true);
            }
            else
            {
                lidarPoints[i].SetActive(false);
            }
        }
    }
    
    // Method to clear all sensor visualizations
    public void ClearVisualizations()
    {
        foreach (var point in lidarPoints)
        {
            point.SetActive(false);
        }
    }
}
```

## Performance Optimization

### Level of Detail (LOD)

Implement LODs for complex robot models:

```csharp
using UnityEngine;

public class RobotLODController : MonoBehaviour
{
    [Header("LOD Settings")]
    [Range(0.1f, 100f)]
    public float lodDistance = 10f;
    
    [Header("LOD Models")]
    public GameObject detailedModel;
    public GameObject simpleModel;
    
    [Header("Camera Reference")]
    public Camera mainCamera;
    
    void Start()
    {
        if (mainCamera == null)
            mainCamera = Camera.main;
    }
    
    void Update()
    {
        if (mainCamera != null)
        {
            float distance = Vector3.Distance(transform.position, mainCamera.transform.position);
            
            detailedModel.SetActive(distance <= lodDistance);
            simpleModel.SetActive(distance > lodDistance);
        }
    }
}
```

### Occlusion Culling

Set up occlusion culling for the scene:

1. Identify static objects (environment, walls, etc.)
2. Mark them as "Static" in the inspector
3. Enable occlusion culling in the lighting settings (Window > Rendering > Lighting Settings)

## Scene Optimization Tips

1. **Use Static Batching**: Mark static environment objects as static to enable static batching
2. **Use Dynamic Batching**: Keep meshes under 900 vertices for dynamic batching
3. **Optimize Draw Calls**:尽量减少材质数量，合并相同材质的物体
4. **Use Texture Atlases**: Combine multiple textures into a single atlas
5. **Implement Frustum Culling**: Only render objects within the camera's view

## Creating Interactive Elements

Add scripts for basic user interaction:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class SceneInteractionController : MonoBehaviour
{
    [Header("Interactive Elements")]
    public GameObject robot;
    public Button resetRobotButton;
    
    [Header("Reset Position")]
    public Vector3 resetPosition = Vector3.zero;
    public Quaternion resetRotation = Quaternion.identity;
    
    void Start()
    {
        if (resetRobotButton != null)
        {
            resetRobotButton.onClick.AddListener(ResetRobotPosition);
        }
    }
    
    void Update()
    {
        // Additional interaction controls can be added here
        // e.g., keyboard controls to move the robot
        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetRobotPosition();
        }
    }
    
    public void ResetRobotPosition()
    {
        if (robot != null)
        {
            robot.transform.position = resetPosition;
            robot.transform.rotation = resetRotation;
        }
    }
}
```

## Next Steps

After implementing the visualization features, your Unity scene should provide a comprehensive and interactive view of your digital twin system. The visualization should include:

1. Multiple camera views for different perspectives
2. Real-time telemetry displays
3. Optimized rendering for smooth performance
4. Interactive elements for user control

In Week 3, you'll build on this visualization by adding perception capabilities that process the simulated sensor data.