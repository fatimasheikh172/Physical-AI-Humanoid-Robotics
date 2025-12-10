using UnityEngine;

public class CameraController : MonoBehaviour
{
    public Transform target; // The object to follow
    public float smoothSpeed = 0.125f;
    public Vector3 offset = new Vector3(0f, 2f, -5f);
    
    // For manual camera control
    public float rotationSpeed = 100.0f;
    public float zoomSpeed = 10.0f;
    public float moveSpeed = 10.0f;
    
    private Vector3 velocity = Vector3.zero;
    private bool manualControl = false;
    private Vector3 dragOrigin;

    void Start()
    {
        if (target != null)
        {
            transform.position = target.position + offset;
        }
    }

    void LateUpdate()
    {
        if (target != null && !manualControl)
        {
            // Follow the target with smooth movement
            Vector3 desiredPosition = target.position + offset;
            Vector3 smoothedPosition = Vector3.SmoothDamp(transform.position, desiredPosition, ref velocity, smoothSpeed);
            transform.position = smoothedPosition;
            
            // Always look at the target
            transform.LookAt(target);
        }
        else if (manualControl)
        {
            // Handle manual camera controls
            HandleManualControls();
        }
    }

    void HandleManualControls()
    {
        // Zoom in/out with scroll wheel or specific keys
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        transform.Translate(Vector3.forward * scroll * zoomSpeed * 100f);
        
        // Rotate camera with right mouse button
        if (Input.GetMouseButton(1)) // Right mouse button
        {
            float rotateX = Input.GetAxis("Mouse X") * rotationSpeed * 0.02f;
            float rotateY = Input.GetAxis("Mouse Y") * rotationSpeed * 0.02f;
            
            transform.Rotate(-rotateY, rotateX, 0, Space.Self);
        }
        
        // Move camera with middle mouse button (pan)
        if (Input.GetMouseButton(2)) // Middle mouse button
        {
            float moveX = Input.GetAxis("Mouse X") * moveSpeed * 0.05f;
            float moveY = Input.GetAxis("Mouse Y") * moveSpeed * 0.05f;
            
            transform.Translate(-moveX, -moveY, 0);
        }
        
        // Toggle between follow mode and manual mode
        if (Input.GetKeyDown(KeyCode.C))
        {
            manualControl = !manualControl;
        }
    }

    void Update()
    {
        // Toggle camera control mode
        if (Input.GetKeyDown(KeyCode.C))
        {
            manualControl = !manualControl;
            
            if (manualControl)
            {
                Debug.Log("Camera: Manual Control Mode");
            }
            else
            {
                Debug.Log("Camera: Follow Mode");
            }
        }
    }
}