# Week 3: Perception Pipeline

## Overview

In Week 3, you'll implement perception algorithms that process the simulated sensor data from your digital twin system. This includes building processing pipelines for LiDAR, camera, and IMU data to enable the robot to understand its environment and make intelligent decisions.

## Learning Objectives

By the end of this week, you will be able to:

- Process LiDAR point cloud data for obstacle detection
- Implement computer vision algorithms for camera data
- Fuse multiple sensor streams for improved perception
- Build perception modules that work with simulated data

## Perception Pipeline Architecture

### Overview of Perception Components

The perception pipeline consists of several processing modules that work together:

1. **Preprocessing**: Clean and format raw sensor data
2. **Feature Extraction**: Identify relevant features in the data
3. **Object Detection**: Recognize and classify objects
4. **Environmental Understanding**: Build a model of the surroundings
5. **Output Generation**: Format results for downstream systems

### Data Flow

```
Raw Sensor Data → Preprocessing → Feature Extraction → Object Detection → Environmental Understanding → Output
     ↓              ↓                   ↓                 ↓                     ↓                  ↓
   LiDAR →    → Point Cloud     → Obstacle Map    → Object List      → Semantic Map    → Action Commands
   Camera →   → Image Processing → Feature Points → Classification   → Scene Understanding
   IMU →      → Inertial Data   → Motion Data    → Movement Info    → Kinematic Model
```

## LiDAR Perception

### Point Cloud Processing

Create a script to process LiDAR data in Unity:

```csharp
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LidarPerception : MonoBehaviour
{
    [Header("ROS Settings")]
    public string lidarTopic = "/scan";
    
    [Header("Perception Settings")]
    public float obstacleThreshold = 1.0f; // Distance threshold for obstacles
    public float minRange = 0.1f;
    public float maxRange = 30f;
    
    [Header("Visualization")]
    public GameObject obstaclePrefab;
    private List<GameObject> obstacleIndicators = new List<GameObject>();
    
    private ROSConnection ros;
    private float[] ranges;
    private float angleMin;
    private float angleIncrement;
    
    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<LaserScanMsg>(lidarTopic, LaserScanCallback);
    }
    
    void LaserScanCallback(LaserScanMsg scan)
    {
        // Store scan data
        this.ranges = new float[scan.ranges.Length];
        for (int i = 0; i < scan.ranges.Length; i++)
        {
            this.ranges[i] = (float)scan.ranges[i];
        }
        
        angleMin = (float)scan.angle_min;
        angleIncrement = (float)scan.angle_increment;
        
        // Process the LiDAR data
        ProcessLidarData();
    }
    
    void ProcessLidarData()
    {
        // Clear previous obstacle indicators
        foreach (var indicator in obstacleIndicators)
        {
            if (indicator != null)
                Destroy(indicator);
        }
        obstacleIndicators.Clear();
        
        // Detect obstacles
        List<Vector3> obstacles = new List<Vector3>();
        
        for (int i = 0; i < ranges.Length; i++)
        {
            float range = ranges[i];
            
            if (range > minRange && range < maxRange && range < obstacleThreshold)
            {
                // Convert polar coordinates to Cartesian
                float angle = angleMin + i * angleIncrement;
                
                // Convert to Unity coordinates (X-right, Y-up, Z-forward)
                Vector3 obstaclePos = new Vector3(
                    range * Mathf.Sin(angle), // Right
                    0.1f,                   // Slightly above ground
                    range * Mathf.Cos(angle)  // Forward
                );
                
                obstacles.Add(obstaclePos);
            }
        }
        
        // Create visual indicators for obstacles
        foreach (var obstaclePos in obstacles)
        {
            Vector3 worldPos = transform.position + obstaclePos;
            GameObject indicator = Instantiate(obstaclePrefab, worldPos, Quaternion.identity);
            obstacleIndicators.Add(indicator);
        }
        
        // Use the detected obstacles for navigation planning
        OnObstaclesDetected(obstacles);
    }
    
    void OnObstaclesDetected(List<Vector3> obstacles)
    {
        // Send obstacle information to navigation system
        Debug.Log($"Detected {obstacles.Count} obstacles");
        // Here you would typically send this information to a navigation planner
    }
}
```

### Occupancy Grid Mapping

Create an occupancy grid from LiDAR data:

```csharp
using System.Collections;
using UnityEngine;

public class OccupancyGridMapper : MonoBehaviour
{
    [Header("Grid Settings")]
    public int gridSize = 100;           // Grid size in each dimension (100x100)
    public float cellSize = 0.5f;        // Size of each cell in meters
    public float robotRadius = 0.5f;     // Robot radius for safety margin
    
    [Header("Visualization")]
    public Material occupiedMaterial;
    public Material freeMaterial;
    public Material unknownMaterial;
    
    private float[,] occupancyGrid;      // Grid: -1 = unknown, 0 = free, 1 = occupied
    private GameObject[,] gridCells;     // Visual representation of each cell
    
    void Start()
    {
        InitializeGrid();
    }
    
    void InitializeGrid()
    {
        occupancyGrid = new float[gridSize, gridSize];
        
        // Initialize all cells as unknown
        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                occupancyGrid[x, y] = -1.0f;
            }
        }
        
        // Create visual representations of cells
        gridCells = new GameObject[gridSize, gridSize];
        
        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                GameObject cell = GameObject.CreatePrimitive(PrimitiveType.Quad);
                cell.transform.parent = transform;
                cell.transform.position = new Vector3(
                    (x - gridSize / 2) * cellSize,
                    0.01f,  // Slightly above ground to be visible
                    (y - gridSize / 2) * cellSize
                );
                cell.transform.rotation = Quaternion.Euler(90, 0, 0);  // Face upwards
                cell.transform.localScale = Vector3.one * cellSize * 0.9f;  // Slightly smaller to show grid lines
                
                var renderer = cell.GetComponent<Renderer>();
                renderer.material = unknownMaterial;
                
                gridCells[x, y] = cell;
            }
        }
    }
    
    public void UpdateGridWithScan(Vector3[] laserPoints, Vector3 robotPosition)
    {
        // Process each laser point
        foreach (var point in laserPoints)
        {
            // Convert world coordinates to grid coordinates
            int gridX = Mathf.RoundToInt((point.x - robotPosition.x) / cellSize + gridSize / 2);
            int gridY = Mathf.RoundToInt((point.z - robotPosition.z) / cellSize + gridSize / 2);
            
            // Check if the point is within the grid bounds
            if (gridX >= 0 && gridX < gridSize && gridY >= 0 && gridY < gridSize)
            {
                // Mark the cell as occupied
                occupancyGrid[gridX, gridY] = 1.0f;
                
                // Update visual representation
                if (gridCells[gridX, gridY] != null)
                {
                    gridCells[gridX, gridY].GetComponent<Renderer>().material = occupiedMaterial;
                }
            }
        }
        
        // Perform ray tracing from robot position to each end point to mark free spaces
        Vector3Int robotGridPos = new Vector3Int(
            Mathf.RoundToInt(robotPosition.x / cellSize + gridSize / 2),
            0,
            Mathf.RoundToInt(robotPosition.z / cellSize + gridSize / 2)
        );
        
        // Mark free space along rays (simplified implementation)
        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                if (occupancyGrid[x, y] == -1.0f)  // Still unknown
                {
                    // This is a simplified approach - in a real implementation,
                    // you'd perform proper ray tracing from robot position to this cell
                    float distance = Vector2.Distance(
                        new Vector2(x, y),
                        new Vector2(robotGridPos.x, robotGridPos.z)
                    );
                    
                    if (distance < 10)  // Mark nearby unknown cells as free
                    {
                        occupancyGrid[x, y] = 0.0f;
                        if (gridCells[x, y] != null)
                        {
                            gridCells[x, y].GetComponent<Renderer>().material = freeMaterial;
                        }
                    }
                }
            }
        }
    }
    
    public bool IsCellOccupied(int x, int y)
    {
        if (x >= 0 && x < gridSize && y >= 0 && y < gridSize)
        {
            return occupancyGrid[x, y] > 0.5f;
        }
        return true; // Treat out-of-bounds as occupied
    }
    
    public Vector3Int WorldToGrid(Vector3 worldPos, Vector3 robotPos)
    {
        int x = Mathf.RoundToInt((worldPos.x - robotPos.x) / cellSize + gridSize / 2);
        int y = Mathf.RoundToInt((worldPos.z - robotPos.z) / cellSize + gridSize / 2);
        return new Vector3Int(x, 0, y);
    }
}
```

## Camera Perception

### Basic Image Processing

Create a script to process camera data (in simulation, this would use Unity's camera):

```csharp
using System.Collections.Generic;
using UnityEngine;

public class CameraPerception : MonoBehaviour
{
    [Header("Camera Settings")]
    public Camera cameraComponent;
    public int processingWidth = 320;
    public int processingHeight = 240;
    
    [Header("Detection Settings")]
    public Color targetColor = Color.red;
    public float colorThreshold = 0.2f;
    
    private RenderTexture renderTexture;
    private Texture2D processedTexture;
    private float lastProcessTime = 0f;
    private float processInterval = 0.1f; // Process every 100ms
    
    void Start()
    {
        if (cameraComponent == null)
            cameraComponent = GetComponent<Camera>();
            
        // Create render texture for processing
        renderTexture = new RenderTexture(processingWidth, processingHeight, 24);
        processedTexture = new Texture2D(processingWidth, processingHeight, TextureFormat.RGB24, false);
        
        cameraComponent.targetTexture = renderTexture;
    }
    
    void Update()
    {
        // Process image periodically to avoid performance issues
        if (Time.time - lastProcessTime > processInterval)
        {
            ProcessImage();
            lastProcessTime = Time.time;
        }
    }
    
    void ProcessImage()
    {
        // Read pixels from the render texture
        RenderTexture.active = renderTexture;
        processedTexture.ReadPixels(new Rect(0, 0, processingWidth, processingHeight), 0, 0);
        processedTexture.Apply();
        RenderTexture.active = null;
        
        // Detect objects by color (simplified example)
        List<Vector2> detectedObjects = FindColorRegions(targetColor);
        
        foreach (Vector2 center in detectedObjects)
        {
            Debug.Log($"Detected {targetColor} object at image coordinates: {center}");
        }
        
        // In a real implementation, you would use more sophisticated computer vision techniques
        OnImageProcessed(detectedObjects);
    }
    
    List<Vector2> FindColorRegions(Color targetColor)
    {
        List<Vector2> centers = new List<Vector2>();
        int regionCount = 0;
        
        // This is a simplified color detection algorithm
        // In a real implementation, use more sophisticated techniques like OpenCV
        for (int x = 0; x < processingWidth; x += 10) // Sample every 10th pixel for performance
        {
            for (int y = 0; y < processingHeight; y += 10)
            {
                Color pixelColor = processedTexture.GetPixel(x, y);
                
                // Compare colors using a threshold
                float colorDistance = Mathf.Abs(pixelColor.r - targetColor.r) +
                                    Mathf.Abs(pixelColor.g - targetColor.g) +
                                    Mathf.Abs(pixelColor.b - targetColor.b);
                
                if (colorDistance < colorThreshold)
                {
                    // Found a potential object, calculate center of region
                    Vector2 regionCenter = FindRegionCenter(x, y, targetColor);
                    if (!centers.Contains(regionCenter))
                    {
                        centers.Add(regionCenter);
                        regionCount++;
                    }
                }
            }
        }
        
        return centers;
    }
    
    Vector2 FindRegionCenter(int startX, int startY, Color targetColor)
    {
        // Simple flood-fill-like approach to find region center
        int totalX = 0, totalY = 0, count = 0;
        
        // For performance, only check a small area around the start point
        int regionSize = 20;
        int startXRegion = Mathf.Max(0, startX - regionSize/2);
        int endXRegion = Mathf.Min(processingWidth, startX + regionSize/2);
        int startYRegion = Mathf.Max(0, startY - regionSize/2);
        int endYRegion = Mathf.Min(processingHeight, startY + regionSize/2);
        
        for (int x = startXRegion; x < endXRegion; x++)
        {
            for (int y = startYRegion; y < endYRegion; y++)
            {
                Color pixelColor = processedTexture.GetPixel(x, y);
                
                float colorDistance = Mathf.Abs(pixelColor.r - targetColor.r) +
                                    Mathf.Abs(pixelColor.g - targetColor.g) +
                                    Mathf.Abs(pixelColor.b - targetColor.b);
                
                if (colorDistance < colorThreshold)
                {
                    totalX += x;
                    totalY += y;
                    count++;
                }
            }
        }
        
        if (count > 0)
        {
            return new Vector2(totalX / count, totalY / count);
        }
        
        return new Vector2(startX, startY);
    }
    
    void OnImageProcessed(List<Vector2> detectedObjects)
    {
        // Process the detected objects
        // This is where you would integrate with other systems
        Debug.Log($"Processed image, found {detectedObjects.Count} objects");
    }
    
    void OnDestroy()
    {
        if (renderTexture != null)
            renderTexture.Release();
        Destroy(processedTexture);
    }
}
```

## Sensor Fusion

### Combining Data from Multiple Sensors

Create a sensor fusion module that combines LiDAR and camera data:

```csharp
using System.Collections.Generic;
using UnityEngine;

public class SensorFusion : MonoBehaviour
{
    [Header("Fusion Settings")]
    [Range(0.0f, 1.0f)]
    public float lidarConfidence = 0.8f;
    [Range(0.0f, 1.0f)]
    public float cameraConfidence = 0.6f;
    
    [Header("Input Data")]
    public Transform robotTransform;
    public List<Vector3> lidarObstacles = new List<Vector3>();
    public List<Vector2> cameraObjects = new List<Vector2>();
    
    [Header("Fused Output")]
    public List<FusedObject> fusedObjects = new List<FusedObject>();
    
    [System.Serializable]
    public class FusedObject
    {
        public Vector3 position;      // World position
        public Vector2 imageCoords;   // Image coordinates
        public float confidence;      // Combined confidence value
        public ObjectType type;       // Type of object
        public float timestamp;       // Time of detection
    }
    
    public enum ObjectType
    {
        Unknown,
        Obstacle,
        Landmark,
        Target
    }
    
    void Update()
    {
        // This would typically be called when new sensor data is available
        PerformFusion();
    }
    
    public void UpdateLidarData(List<Vector3> obstacles)
    {
        lidarObstacles = obstacles;
    }
    
    public void UpdateCameraData(List<Vector2> objects)
    {
        cameraObjects = objects;
    }
    
    void PerformFusion()
    {
        fusedObjects.Clear();
        
        // Convert camera objects to world space
        // This requires a rough depth estimate since we only have 2D image coordinates
        foreach (var cameraObj in cameraObjects)
        {
            // Estimate depth based on LiDAR data or assumptions
            // For simplicity, assume all detected objects are at 2 meters
            float estimatedDepth = EstimateDepth(cameraObj);
            
            // Convert image coordinates to world coordinates
            Vector3 worldPos = CameraToUnityWorld(cameraObj, estimatedDepth);
            
            // Find corresponding LiDAR detection
            Vector3 closestLidarPoint = FindClosestLidarPoint(worldPos);
            float distanceToLidar = Vector3.Distance(worldPos, closestLidarPoint);
            
            // Create fused object
            FusedObject fusedObj = new FusedObject();
            fusedObj.position = worldPos;
            fusedObj.imageCoords = cameraObj;
            
            // Calculate confidence based on sensor agreement
            if (distanceToLidar < 0.5f) // Close to a LiDAR point
            {
                fusedObj.confidence = (lidarConfidence + cameraConfidence) / 2f;
                fusedObj.type = ObjectType.Obstacle;
            }
            else // Only camera detection
            {
                fusedObj.confidence = cameraConfidence * 0.7f; // Lower confidence
                fusedObj.type = ObjectType.Unknown;
            }
            
            fusedObj.timestamp = Time.time;
            fusedObjects.Add(fusedObj);
        }
        
        // Process fused objects
        ProcessFusedResults();
    }
    
    float EstimateDepth(Vector2 imageCoords)
    {
        // Simplified depth estimation
        // In a real system, this would use stereo vision, depth maps, or other methods
        return 2.0f; // Assume 2 meters for demonstration
    }
    
    Vector3 CameraToUnityWorld(Vector2 imageCoords, float depth)
    {
        // Convert image coordinates to Unity world coordinates
        // This requires information about the Unity camera's FOV and position
        Camera unityCamera = GetComponent<Camera>();
        if (unityCamera == null)
        {
            unityCamera = Camera.main;
        }
        
        if (unityCamera == null)
            return Vector3.zero;
        
        // Convert image coordinates (0-1) to normalized device coordinates (-1 to 1)
        Vector3 ndc = new Vector3(
            (imageCoords.x / Screen.width) * 2 - 1,
            (imageCoords.y / Screen.height) * 2 - 1,
            depth
        );
        
        // Convert NDC to world coordinates
        Vector3 worldPos = unityCamera.ViewportToWorldPoint(new Vector3(
            (imageCoords.x / Screen.width),
            (imageCoords.y / Screen.height),
            depth
        ));
        
        return worldPos;
    }
    
    Vector3 FindClosestLidarPoint(Vector3 worldPos)
    {
        Vector3 closest = Vector3.zero;
        float minDist = float.MaxValue;
        
        foreach (var lidarPoint in lidarObstacles)
        {
            float dist = Vector3.Distance(worldPos, lidarPoint);
            if (dist < minDist)
            {
                minDist = dist;
                closest = lidarPoint;
            }
        }
        
        return minDist < float.MaxValue ? closest : worldPos;
    }
    
    void ProcessFusedResults()
    {
        // Here you would send the fused objects to navigation, planning, or other systems
        foreach (var obj in fusedObjects)
        {
            Debug.Log($"Fused object: pos={obj.position}, confidence={obj.confidence:F2}, type={obj.type}");
            
            // Example: If high confidence obstacle, trigger avoidance behavior
            if (obj.type == ObjectType.Obstacle && obj.confidence > 0.7f)
            {
                TriggerObstacleAvoidance(obj.position);
            }
        }
    }
    
    void TriggerObstacleAvoidance(Vector3 obstaclePosition)
    {
        // In a real system, this would send commands to the navigation system
        Debug.Log($"Avoiding obstacle at {obstaclePosition}");
    }
}
```

## Perception Pipeline Integration

### Connecting Perception to Other Systems

Create a perception manager that coordinates different perception modules:

```csharp
using System.Collections.Generic;
using UnityEngine;

public class PerceptionManager : MonoBehaviour
{
    [Header("Perception Modules")]
    public LidarPerception lidarPerception;
    public CameraPerception cameraPerception;
    public SensorFusion sensorFusion;
    public OccupancyGridMapper gridMapper;
    
    [Header("Performance Settings")]
    public float perceptionInterval = 0.1f;
    
    private float lastPerceptionUpdate = 0f;
    
    void Start()
    {
        // Initialize perception modules
        if (lidarPerception == null)
            lidarPerception = GetComponent<LidarPerception>();
        if (cameraPerception == null)
            cameraPerception = GetComponent<CameraPerception>();
        if (sensorFusion == null)
            sensorFusion = GetComponent<SensorFusion>();
        if (gridMapper == null)
            gridMapper = GetComponent<OccupancyGridMapper>();
    }
    
    void Update()
    {
        // Run perception pipeline at specified intervals
        if (Time.time - lastPerceptionUpdate > perceptionInterval)
        {
            RunPerceptionPipeline();
            lastPerceptionUpdate = Time.time;
        }
    }
    
    void RunPerceptionPipeline()
    {
        // The perception pipeline is driven by incoming ROS messages,
        // but here we synchronize the processing of collected data
        
        // Process LiDAR data
        if (lidarPerception != null)
        {
            // This would be called when new LiDAR data arrives
        }
        
        // Process camera data
        if (cameraPerception != null)
        {
            // This runs continuously, but results are only used periodically
        }
        
        // Update occupancy grid
        if (gridMapper != null && lidarPerception != null)
        {
            // Update grid with latest LiDAR data
            // This needs to be implemented based on actual LiDAR data
        }
        
        // Perform sensor fusion
        if (sensorFusion != null)
        {
            sensorFusion.PerformFusion();
        }
        
        // Process perception results
        ProcessPerceptionResults();
    }
    
    void ProcessPerceptionResults()
    {
        // Use the fused perception results for navigation, planning, etc.
        if (sensorFusion != null)
        {
            foreach (var obj in sensorFusion.fusedObjects)
            {
                // Send object to navigation planner
                SendToObjectTracker(obj);
                
                // If it's a significant obstacle, update path planning
                if (obj.type == SensorFusion.ObjectType.Obstacle && obj.confidence > 0.7f)
                {
                    UpdatePathPlanning(obj.position);
                }
            }
        }
    }
    
    void SendToObjectTracker(SensorFusion.FusedObject obj)
    {
        // In a real system, this would add objects to a tracking system
        Debug.Log($"Tracking object at {obj.position}");
    }
    
    void UpdatePathPlanning(Vector3 obstaclePos)
    {
        // In a real system, this would update a path planner
        Debug.Log($"Replanning path to avoid obstacle at {obstaclePos}");
    }
}
```

## Next Steps

After implementing the perception pipeline, your digital twin system will be able to process simulated sensor data much like a real robot would. The perception system will:

1. Process LiDAR data to detect obstacles and build occupancy maps
2. Process camera data to identify visual features and landmarks
3. Fuse information from multiple sensors for more reliable perception
4. Provide processed information to downstream systems

In the final part of Week 3, you'll look at simulation fidelity and how to validate that your perception system is working correctly with simulated data.