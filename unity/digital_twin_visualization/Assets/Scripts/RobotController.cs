using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class RobotController : MonoBehaviour
{
    public ROSConnectionManager rosConnection;
    public Slider linearVelocitySlider;
    public Slider angularVelocitySlider;
    public Button sendCommandButton;
    
    private float linearVelocity = 0.0f;
    private float angularVelocity = 0.0f;
    
    void Start()
    {
        if (sendCommandButton != null)
        {
            sendCommandButton.onClick.AddListener(SendCommand);
        }
        
        UpdateSliders();
    }
    
    void UpdateSliders()
    {
        if (linearVelocitySlider != null)
        {
            linearVelocitySlider.onValueChanged.AddListener(OnLinearVelocityChanged);
        }
        
        if (angularVelocitySlider != null)
        {
            angularVelocitySlider.onValueChanged.AddListener(OnAngularVelocityChanged);
        }
    }
    
    void OnLinearVelocityChanged(float value)
    {
        linearVelocity = value;
    }
    
    void OnAngularVelocityChanged(float value)
    {
        angularVelocity = value;
    }
    
    void SendCommand()
    {
        if (rosConnection != null)
        {
            rosConnection.SendVelocityCommand(linearVelocity, angularVelocity);
            Debug.Log($"Sending command: linear={linearVelocity}, angular={angularVelocity}");
        }
        else
        {
            Debug.LogWarning("ROS Connection Manager is not assigned!");
        }
    }
    
    // Update is called once per frame
    void Update()
    {
        // You can also send commands based on user input (e.g., keyboard)
        if (Input.GetKey(KeyCode.UpArrow))
        {
            rosConnection.SendVelocityCommand(1.0f, 0.0f);
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            rosConnection.SendVelocityCommand(-1.0f, 0.0f);
        }
        else if (Input.GetKey(KeyCode.LeftArrow))
        {
            rosConnection.SendVelocityCommand(0.0f, 1.0f);
        }
        else if (Input.GetKey(KeyCode.RightArrow))
        {
            rosConnection.SendVelocityCommand(0.0f, -1.0f);
        }
        else if (Input.anyKey == false)
        {
            // Stop the robot when no keys are pressed
            rosConnection.SendVelocityCommand(0.0f, 0.0f);
        }
    }
}