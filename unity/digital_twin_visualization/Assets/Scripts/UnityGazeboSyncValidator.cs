using UnityEngine;

public class UnityGazeboSyncValidator : MonoBehaviour
{
    [Header("Robot References")]
    public Transform gazeboRobotTransform;  // The transform representing the robot from Gazebo
    public Transform unityRobotTransform;   // The transform representing the robot in Unity
    
    [Header("Validation Settings")]
    public float positionTolerance = 0.1f;  // Maximum position difference allowed (meters)
    public float rotationTolerance = 5.0f;  // Maximum rotation difference allowed (degrees)
    public float maxUpdateDelay = 0.5f;     // Maximum time difference allowed (seconds)
    
    [Header("Output")]
    public bool isSynchronized = true;
    public float lastSyncTime = 0f;
    
    private float lastCheckTime = 0f;
    private float checkInterval = 1.0f;  // Check sync every second
    
    void Start()
    {
        if (unityRobotTransform == null)
        {
            unityRobotTransform = transform;  // Use this object's transform as default
        }
    }
    
    void Update()
    {
        // Perform validation check periodically
        if (Time.time - lastCheckTime >= checkInterval)
        {
            ValidateSynchronization();
            lastCheckTime = Time.time;
        }
    }
    
    void ValidateSynchronization()
    {
        if (gazeboRobotTransform == null)
        {
            Debug.LogWarning("Gazebo robot transform is not assigned for synchronization validation");
            isSynchronized = false;
            return;
        }
        
        // Calculate position difference
        float positionDiff = Vector3.Distance(gazeboRobotTransform.position, unityRobotTransform.position);
        
        // Calculate rotation difference
        float rotationDiff = Quaternion.Angle(gazeboRobotTransform.rotation, unityRobotTransform.rotation);
        
        // Update synchronization status
        isSynchronized = (positionDiff <= positionTolerance) && (rotationDiff <= rotationTolerance);
        
        // Record sync time
        lastSyncTime = Time.time;
        
        // Log validation results
        if (isSynchronized)
        {
            Debug.Log($"Synchronization VALID: Position diff: {positionDiff:F3}m, Rotation diff: {rotationDiff:F2}°");
        }
        else
        {
            Debug.LogWarning($"Synchronization INVALID: Position diff: {positionDiff:F3}m (tol: {positionTolerance}m), " +
                           $"Rotation diff: {rotationDiff:F2}° (tol: {rotationTolerance}°)");
        }
    }
    
    // Method to be called when new robot state is received from Gazebo
    public void OnGazeboRobotStateReceived(Vector3 gazeboPosition, Quaternion gazeboRotation)
    {
        // Update the gazebo robot transform
        if (gazeboRobotTransform != null)
        {
            gazeboRobotTransform.position = gazeboPosition;
            gazeboRobotTransform.rotation = gazeboRotation;
        }
        
        // Validate synchronization immediately
        ValidateSynchronization();
    }
    
    // Visualization in the editor
    void OnValidate()
    {
        if (positionTolerance < 0) positionTolerance = 0;
        if (rotationTolerance < 0) rotationTolerance = 0;
        if (maxUpdateDelay < 0) maxUpdateDelay = 0;
        if (checkInterval <= 0) checkInterval = 0.1f;
    }
    
#if UNITY_EDITOR
    // Draw gizmos for position tolerance visualization
    void OnDrawGizmosSelected()
    {
        if (unityRobotTransform != null)
        {
            // Draw a sphere showing the position tolerance
            Gizmos.color = isSynchronized ? Color.green : Color.red;
            Gizmos.DrawWireSphere(unityRobotTransform.position, positionTolerance);
        }
    }
#endif
}