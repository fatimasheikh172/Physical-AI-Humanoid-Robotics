using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections;

public class TelemetryDisplay : MonoBehaviour
{
    [Header("UI Elements")]
    public Text positionText;
    public Text orientationText;
    public Text lidarText;
    public Text imuText;
    public Text velocityText;
    
    [Header("ROS Connection")]
    public ROSConnectionManager rosConnection;
    
    // Store the latest sensor data
    private RobotStateData latestRobotState;
    private JointStateData latestJointState;
    private bool hasNewData = false;
    
    void Start()
    {
        // Subscribe to robot state updates
        if (rosConnection != null)
        {
            rosConnection.OnRobotPoseReceived += UpdatePositionOrientation;
        }
        
        InitializeUI();
    }
    
    void InitializeUI()
    {
        if (positionText == null) 
        {
            GameObject posObj = new GameObject("PositionText");
            posObj.transform.SetParent(transform, false);
            positionText = posObj.AddComponent<Text>();
            positionText.text = "Position: N/A";
            positionText.fontSize = 14;
            positionText.color = Color.white;
            // We need to set up a proper UI parent with Canvas for this to work in practice
        }
        
        if (orientationText == null)
        {
            GameObject orientObj = new GameObject("OrientationText");
            orientObj.transform.SetParent(transform, false);
            orientationText = orientObj.AddComponent<Text>();
            orientationText.text = "Orientation: N/A";
            orientationText.fontSize = 14;
            orientationText.color = Color.white;
        }
        
        if (lidarText == null)
        {
            GameObject lidarObj = new GameObject("LidarText");
            lidarObj.transform.SetParent(transform, false);
            lidarText = lidarObj.AddComponent<Text>();
            lidarText.text = "Lidar: N/A";
            lidarText.fontSize = 14;
            lidarText.color = Color.white;
        }
        
        if (imuText == null)
        {
            GameObject imuObj = new GameObject("IMUText");
            imuObj.transform.SetParent(transform, false);
            imuText = imuObj.AddComponent<Text>();
            imuText.text = "IMU: N/A";
            imuText.fontSize = 14;
            imuText.color = Color.white;
        }
        
        if (velocityText == null)
        {
            GameObject velObj = new GameObject("VelocityText");
            velObj.transform.SetParent(transform, false);
            velocityText = velObj.AddComponent<Text>();
            velocityText.text = "Velocity: N/A";
            velocityText.fontSize = 14;
            velocityText.color = Color.white;
        }
    }
    
    void Update()
    {
        if (hasNewData)
        {
            UpdateUI();
            hasNewData = false;
        }
    }
    
    private void UpdatePositionOrientation(RosMessageTypes.Geometry.PoseMessage pose)
    {
        // Update position and orientation text
        if (positionText != null)
        {
            positionText.text = $"Position: X:{pose.position.x:F2}, Y:{pose.position.y:F2}, Z:{pose.position.z:F2}";
        }
        
        if (orientationText != null)
        {
            orientationText.text = $"Orientation: X:{pose.orientation.x:F2}, Y:{pose.orientation.y:F2}, Z:{pose.orientation.z:F2}, W:{pose.orientation.w:F2}";
        }
        
        hasNewData = true;
    }
    
    // Method to update with joint state data
    public void UpdateWithJointState(string jointStateData)
    {
        if (velocityText != null)
        {
            // In a real implementation, this would parse the joint state data
            // and compute the robot's velocity from wheel joint positions
            velocityText.text = "Velocity: Computing...";
        }
    }
    
    // Method to update with LiDAR data
    public void UpdateWithLidarData(string lidarData)
    {
        if (lidarText != null)
        {
            // For now, just show that we received data
            lidarText.text = "Lidar: Data received";
        }
    }
    
    // Method to update with IMU data
    public void UpdateWithIMUData(string imuData)
    {
        if (imuText != null)
        {
            // For now, just show that we received data
            imuText.text = "IMU: Data received";
        }
    }
    
    private void UpdateUI()
    {
        // Update all UI elements with the latest data
        // This is handled in the callbacks for real-time updates
    }
    
    void OnDestroy()
    {
        if (rosConnection != null)
        {
            rosConnection.OnRobotPoseReceived -= UpdatePositionOrientation;
        }
    }
}