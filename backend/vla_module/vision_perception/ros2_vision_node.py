"""
ROS 2 Node for Vision Perception in the Vision-Language-Action (VLA) module.

This module implements a ROS 2 node that handles computer vision tasks including
object detection, scene understanding, and visual processing for the VLA system.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import asyncio
import threading
from typing import Optional, List, Dict, Any

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import VisionObservation, DetectedObject
from ..core.data_models import VisionObservationModel, DetectedObjectModel
from .computer_vision_processor import VisionProcessor, get_vision_processor


class VisionPerceptionNode(Node):
    """
    ROS 2 node that handles computer vision tasks for the VLA system.
    """
    
    def __init__(self, node_name: str = "vla_vision_perception_node"):
        super().__init__(node_name)
        
        self.config = get_config()
        self.logger = setup_logger(node_name)
        
        # Initialize vision processor
        self.vision_processor = get_vision_processor()
        
        # Setup CV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Setup ROS 2 publishers and subscribers
        self.setup_ros2_interfaces()
        
        # Vision processing state
        self.is_processing = False
        self.last_image_timestamp = 0.0
        
        # Threading for async processing
        self.async_loop = None
        self.async_thread = None
        self._setup_async_loop()
        
        self.logger.info(f"{node_name} initialized successfully")
    
    def setup_ros2_interfaces(self):
        """Set up publishers and subscribers for ROS 2 communication."""
        # Publishers
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/vla/detected_objects',
            qos_profile=qos_profile_sensor_data
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/vla/vision_status',
            qos_profile=qos_profile_sensor_data
        )
        
        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        # Set up timer for periodic processing if needed
        self.processing_timer = self.create_timer(
            1.0 / self.config.get('vision_processing_frequency', 10.0),  # Default 10Hz
            self.periodic_processing_callback
        )
        
        self.logger.info("ROS 2 interfaces set up for vision perception")
    
    def _setup_async_loop(self):
        """Set up the asyncio event loop in a separate thread."""
        self.async_loop = asyncio.new_event_loop()
        
        def run_loop():
            asyncio.set_event_loop(self.async_loop)
            self.async_loop.run_forever()
        
        self.async_thread = threading.Thread(target=run_loop, daemon=True)
        self.async_thread.start()
    
    def _teardown_async_loop(self):
        """Teardown the asyncio event loop."""
        if self.async_loop:
            self.async_loop.call_soon_threadsafe(self.async_loop.stop)
            if self.async_thread:
                self.async_thread.join()
    
    def image_callback(self, msg: Image):
        """Callback to handle incoming image messages."""
        try:
            self.logger.debug(f"Received image with timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
            
            # Convert ROS Image message to CV2 format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Convert to bytes for processing (this is a simplification)
            # In a real implementation, we'd use the actual image data
            success, encoded_image = cv2.imencode('.jpg', cv_image)
            if not success:
                self.logger.error("Failed to encode image for processing")
                return
            
            image_bytes = encoded_image.tobytes()
            
            # Process image asynchronously
            future = asyncio.run_coroutine_threadsafe(
                self.process_image_async(image_bytes),
                self.async_loop
            )
            
            # Publish status
            status_msg = String()
            status_msg.data = "PROCESSING"
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.logger.error(f"Error in image callback: {e}")
            status_msg = String()
            status_msg.data = f"ERROR: {str(e)}"
            self.status_publisher.publish(status_msg)
    
    def camera_info_callback(self, msg: CameraInfo):
        """Callback to handle camera info messages."""
        try:
            self.logger.debug(f"Received camera info with resolution: {msg.width}x{msg.height}")
            
            # Store camera info for future use
            self.camera_info = {
                'width': msg.width,
                'height': msg.height,
                'intrinsics': msg.k,  # Camera intrinsics
                'distortion_model': msg.distortion_model,
                'distortion_coefficients': msg.d
            }
            
        except Exception as e:
            self.logger.error(f"Error in camera info callback: {e}")
    
    async def process_image_async(self, image_bytes: bytes) -> Optional[VisionObservationModel]:
        """
        Process image asynchronously using the vision processor.
        
        Args:
            image_bytes: Image data as bytes
            
        Returns:
            VisionObservationModel with results or None on failure
        """
        try:
            start_time = time.time()
            
            # Process image using vision processor
            perception_result = await self.vision_processor.process_image(image_bytes)
            
            # Convert results to VisionObservationModel
            observation = VisionObservationModel.create(
                observation_id=f"vis_obs_{int(time.time()*1000)}",
                state_id="current_state",  # In a real system, this would be tied to robot state
                timestamp=time.time(),
                objects_detected=perception_result.detected_objects,
                image_timestamp=self.last_image_timestamp
            )
            
            # Add camera info if available
            if hasattr(self, 'camera_info'):
                observation.camera_info = self.camera_info
            
            # Publish detections to ROS
            await self.publish_detections(observation)
            
            processing_time = time.time() - start_time
            self.logger.info(f"Image processing completed in {processing_time:.3f}s: {len(observation.objects_detected)} objects detected")
            
            # Publish completion status
            status_msg = String()
            status_msg.data = f"COMPLETED: {len(observation.objects_detected)} objects in {processing_time:.3f}s"
            self.status_publisher.publish(status_msg)
            
            return observation
            
        except Exception as e:
            self.logger.error(f"Error in async image processing: {e}")
            
            # Publish error status
            status_msg = String()
            status_msg.data = f"ERROR: {str(e)}"
            self.status_publisher.publish(status_msg)
            
            raise VLAException(
                f"Async image processing error: {str(e)}", 
                VLAErrorType.VISION_ERROR,
                e
            )
    
    async def publish_detections(self, observation: VisionObservationModel):
        """
        Publish object detections to the appropriate ROS topic.
        
        Args:
            observation: VisionObservationModel with detection results
        """
        try:
            # Create Detection2DArray message
            detections_msg = Detection2DArray()
            detections_msg.header.stamp = self.get_clock().now().to_msg()
            detections_msg.header.frame_id = "camera_link"  # In a real system, use proper frame
            
            for detected_obj in observation.objects_detected:
                detection = Detection2D()
                detection.header.stamp = self.get_clock().now().to_msg()
                detection.header.frame_id = "camera_link"
                
                # Set up the bounding box
                bbox = detected_obj.bounding_box
                detection.bbox.center.x = (bbox['x_min'] + bbox['x_max']) / 2
                detection.bbox.center.y = (bbox['y_min'] + bbox['y_max']) / 2
                detection.bbox.size_x = bbox['x_max'] - bbox['x_min']
                detection.bbox.size_y = bbox['y_max'] - bbox['y_min']
                
                # Set up hypotheses (classification results)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = detected_obj.name
                hypothesis.score = detected_obj.confidence
                # Note: pose not set in this example, would require 3D position
                
                detection.results.append(hypothesis)
            
            # Publish the detections
            self.detection_publisher.publish(detections_msg)
            
            self.logger.debug(f"Published {len(detections_msg.detections)} detections to /vla/detected_objects")
            
        except Exception as e:
            self.logger.error(f"Error publishing detections: {e}")
            raise VLAException(
                f"Detections publishing error: {str(e)}", 
                VLAErrorType.ROSCOMM_ERROR,
                e
            )
    
    def periodic_processing_callback(self):
        """Timer callback for periodic vision processing."""
        # This would be used for continuous processing in a real system
        # For now, we'll just log that the timer is running
        self.logger.debug("Vision perception timer callback executed")
    
    @log_exception()
    async def detect_specific_object_request(self, target_object: str) -> Optional[DetectedObjectModel]:
        """
        Process a request to detect a specific object.
        
        Args:
            target_object: Name of the object to detect
            
        Returns:
            DetectedObjectModel if found, None otherwise
        """
        try:
            self.logger.info(f"Processing specific object detection request for: {target_object}")
            
            # For this implementation, we'll return the last detected object of the requested type
            # In a real system, this might trigger a fresh image capture and processing
            last_observation = getattr(self, 'last_observation', None)
            if last_observation:
                for obj in last_observation.objects_detected:
                    if target_object.lower() in obj.name.lower():
                        self.logger.info(f"Found {target_object} in last observation")
                        return obj
            
            self.logger.info(f"Object {target_object} not found in recent observations")
            return None
            
        except Exception as e:
            self.logger.error(f"Error in specific object detection request: {e}")
            raise VLAException(
                f"Specific object detection request error: {str(e)}", 
                VLAErrorType.VISION_ERROR,
                e
            )
    
    def get_vision_status(self) -> Dict[str, Any]:
        """
        Get the current status of the vision system.
        
        Returns:
            Dictionary with vision system status
        """
        return {
            'is_processing': self.is_processing,
            'last_image_timestamp': self.last_image_timestamp,
            'detection_publisher_active': True,  # Check if publisher has connections
            'image_subscriber_active': len(self.image_subscriber.event_handlers) > 0,
            'camera_info_available': hasattr(self, 'camera_info'),
            'node_uptime': self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        }
    
    def destroy_node(self):
        """Cleanup when the node is destroyed."""
        self.logger.info("Destroying VisionPerceptionNode")
        
        # Stop the async loop
        self._teardown_async_loop()
        
        # Call parent destroy
        super().destroy_node()


def main(args=None):
    """Main function to run the vision perception node."""
    rclpy.init(args=args)
    
    node = VisionPerceptionNode()
    node.start_time = node.get_clock().now().seconds_nanoseconds()[0]
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


# Global vision node instance
_vision_node = None


def get_vision_node(node_name: str = "vla_vision_perception_node") -> VisionPerceptionNode:
    """Get the global vision perception node instance."""
    global _vision_node
    if _vision_node is None:
        # Note: We can't create a ROS2 node here without ROS2 being initialized
        # This would need to be called from within a ROS2 context
        pass
    
    return _vision_node


async def process_image_with_node(image_bytes: bytes) -> Optional[VisionObservationModel]:
    """Convenience function to process an image via the vision node."""
    node = get_vision_node()
    if node:
        return await node.process_image_async(image_bytes)
    else:
        # Fallback to direct vision processor if node not available
        processor = get_vision_processor()
        return await processor.process_image(image_bytes)