#!/usr/bin/env python3

"""
Synthetic Dataset Generator for Isaac Sim
Generates photorealistic synthetic datasets with ground truth annotations
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
import numpy as np
import cv2
from cv_bridge import CvBridge
import json
import os
from datetime import datetime
import random


class SyntheticDatasetGenerator(Node):
    def __init__(self):
        super().__init__('synthetic_dataset_generator')
        
        # Declare parameters
        self.declare_parameter('scene_path', '/default/scene.usd')
        self.declare_parameter('output_path', '/tmp/synthetic_dataset')
        self.declare_parameter('image_count', 1000)
        self.declare_parameter('domain_randomization', True)
        self.declare_parameter('lighting_variation', True)
        self.declare_parameter('texture_variation', True)
        self.declare_parameter('physics_enabled', True)
        self.declare_parameter('annotations', ['bounding_box', 'segmentation', 'depth'])
        
        # Get parameters
        self.scene_path = self.get_parameter('scene_path').get_parameter_value().string_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.image_count = self.get_parameter('image_count').get_parameter_value().integer_value
        self.domain_randomization = self.get_parameter('domain_randomization').get_parameter_value().bool_value
        self.lighting_variation = self.get_parameter('lighting_variation').get_parameter_value().bool_value
        self.texture_variation = self.get_parameter('texture_variation').get_parameter_value().bool_value
        self.physics_enabled = self.get_parameter('physics_enabled').get_parameter_value().bool_value
        self.annotations = self.get_parameter('annotations').get_parameter_value().string_array_value

        # Setup CvBridge
        self.cv_bridge = CvBridge()

        # Publishers and subscribers
        self.dataset_status_publisher = self.create_publisher(String, '/synthetic/generator_status', 10)
        self.dataset_complete_publisher = self.create_publisher(Bool, '/synthetic/generation_complete', 10)
        
        # Service for triggering generation
        self.generate_service = self.create_service(
            String, 
            '/synthetic/generate', 
            self.generate_dataset_callback
        )
        
        # Storage for scene data
        self.generated_images = []
        self.generation_stats = {
            'total_requested': 0,
            'total_generated': 0,
            'start_time': None,
            'end_time': None
        }

        # Create output directories
        self.setup_output_directories()
        
        self.get_logger().info('Synthetic Dataset Generator initialized')

    def setup_output_directories(self):
        """Create necessary output directories"""
        os.makedirs(self.output_path, exist_ok=True)
        os.makedirs(os.path.join(self.output_path, 'images'), exist_ok=True)
        os.makedirs(os.path.join(self.output_path, 'annotations'), exist_ok=True)
        os.makedirs(os.path.join(self.output_path, 'metadata'), exist_ok=True)

    def generate_dataset_callback(self, request, response):
        """Service callback to trigger dataset generation"""
        try:
            # Parse request parameters
            params = json.loads(request.data) if isinstance(request.data, str) else {}
            
            # Override parameters if provided in request
            scene_path = params.get('scene_path', self.scene_path)
            output_path = params.get('output_path', self.output_path)
            image_count = params.get('image_count', self.image_count)
            domain_randomization = params.get('domain_randomization', self.domain_randomization)
            annotations = params.get('annotations', self.annotations)
            
            # Start generation
            self.get_logger().info(f'Starting synthetic dataset generation with {image_count} images')
            self.generation_stats['total_requested'] = image_count
            self.generation_stats['start_time'] = datetime.now()
            
            # Status update
            status_msg = String()
            status_msg.data = f'Started_generation: {image_count} images requested'
            self.dataset_status_publisher.publish(status_msg)
            
            # Generate the dataset
            self.generate_images(scene_path, output_path, image_count, domain_randomization, annotations)
            
            # Update stats
            self.generation_stats['end_time'] = datetime.now()
            self.generation_stats['total_generated'] = len(self.generated_images)
            
            # Complete message
            complete_msg = Bool()
            complete_msg.data = True
            self.dataset_complete_publisher.publish(complete_msg)
            
            # Status completion
            status_msg.data = f'completed: generated {len(self.generated_images)}/{image_count} images'
            self.dataset_status_publisher.publish(status_msg)
            
            # Save generation metadata
            self.save_metadata()
            
            response.data = f'Successfully generated {len(self.generated_images)} images to {output_path}'
        except Exception as e:
            self.get_logger().error(f'Error generating dataset: {e}')
            response.data = f'Error: {str(e)}'
        
        return response

    def generate_images(self, scene_path, output_path, image_count, domain_randomization, annotations):
        """Generate synthetic images with annotations"""
        try:
            # Simulate loading scene data (in real implementation, this would interface with Isaac Sim)
            self.get_logger().info(f'Loading scene from: {scene_path}')
            
            for i in range(image_count):
                # Apply domain randomization if enabled
                if domain_randomization:
                    self.apply_domain_randomization()
                
                # Generate an image with random content for simulation
                image_data = self.generate_synthetic_image()
                
                # Generate annotations based on the content
                annotations_data = self.generate_annotations(image_data, annotations)
                
                # Save image and annotations
                image_filename = f"synthetic_{i:05d}.png"
                annotation_filename = f"synthetic_{i:05d}_annotations.json"
                
                image_path = os.path.join(output_path, 'images', image_filename)
                annotation_path = os.path.join(output_path, 'annotations', annotation_filename)
                
                cv2.imwrite(image_path, image_data)
                with open(annotation_path, 'w') as f:
                    json.dump(annotations_data, f, indent=2)
                
                # Store record of generated image
                self.generated_images.append({
                    'image_path': image_path,
                    'annotation_path': annotation_path,
                    'index': i
                })
                
                # Log progress
                if i % 100 == 0:
                    self.get_logger().info(f'Generated {i+1}/{image_count} synthetic images')
                
                # Simulate processing time
                # In real implementation, this would actually render in Isaac Sim
                
        except Exception as e:
            self.get_logger().error(f'Error in image generation loop: {e}')
            raise

    def apply_domain_randomization(self):
        """Apply domain randomization techniques to vary scene appearance"""
        # In real Isaac Sim implementation, this would:
        # - Change lighting conditions
        # - Modify material properties
        # - Vary object poses
        # - Adjust camera parameters
        # For simulation, we'll just log the action
        
        if self.lighting_variation:
            # Simulate lighting changes
            lighting_params = {
                'intensity': random.uniform(0.5, 2.0),
                'temperature': random.uniform(3000, 8000),  # Kelvin
                'direction': [random.uniform(-1, 1) for _ in range(3)]
            }
        
        if self.texture_variation:
            # Simulate texture changes
            texture_params = {
                'roughness': random.uniform(0.0, 1.0),
                'metallic': random.uniform(0.0, 1.0),
                'color_shift': [random.uniform(0.8, 1.2) for _ in range(3)]  # RGB
            }

    def generate_synthetic_image(self):
        """Generate a synthetic image with objects"""
        # Create a random synthetic image with some geometric shapes
        height, width = 480, 640
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Random background color (to simulate domain randomization)
        bg_color = [random.randint(50, 200) for _ in range(3)]
        image[:] = bg_color
        
        # Add random objects (for simulation purposes)
        num_objects = random.randint(1, 5)
        objects = []
        
        for _ in range(num_objects):
            # Random object type
            obj_type = random.choice(['rectangle', 'circle', 'triangle'])
            
            # Random position and size
            x = random.randint(50, width-50)
            y = random.randint(50, height-50)
            width_obj = random.randint(20, 100)
            height_obj = random.randint(20, 100)
            
            # Random color
            color = [random.randint(0, 255) for _ in range(3)]
            
            if obj_type == 'rectangle':
                pt1 = (x - width_obj//2, y - height_obj//2)
                pt2 = (x + width_obj//2, y + height_obj//2)
                cv2.rectangle(image, pt1, pt2, color, -1)
            elif obj_type == 'circle':
                radius = min(width_obj, height_obj) // 2
                center = (x, y)
                cv2.circle(image, center, radius, color, -1)
            elif obj_type == 'triangle':
                # Create a simple triangle
                pts = np.array([
                    [x, y - height_obj//2],
                    [x - width_obj//2, y + height_obj//2],
                    [x + width_obj//2, y + height_obj//2]
                ], np.int32)
                cv2.fillPoly(image, [pts], color)
        
        return image

    def generate_annotations(self, image_data, annotation_types):
        """Generate annotations for the synthetic image"""
        annotations = {
            'image_shape': list(image_data.shape),
            'timestamp': datetime.now().isoformat()
        }
        
        if 'bounding_box' in annotation_types:
            annotations['bounding_boxes'] = self.create_bounding_box_annotations(image_data)
        
        if 'segmentation' in annotation_types:
            annotations['segmentation_masks'] = self.create_segmentation_annotations(image_data)
        
        if 'depth' in annotation_types:
            annotations['depth_maps'] = self.create_depth_annotations(image_data)
        
        if 'pose' in annotation_types:
            annotations['poses'] = self.create_pose_annotations()
        
        return annotations

    def create_bounding_box_annotations(self, image_data):
        """Create bounding box annotations for objects in image"""
        # This is a simplified example - in real implementation, 
        # this would come from Isaac Replicator's ground truth
        height, width = image_data.shape[:2]
        boxes = []
        
        # For simulation, detect colored regions in image
        # In real Isaac Sim, this would be ground truth object positions
        for color_val in [50, 100, 150, 200, 250]:  # Some example colors
            # Create mask for this color (with tolerance)
            lower = np.array([max(0, color_val-10)]*3)
            upper = np.array([min(255, color_val+10)]*3)
            
            mask = cv2.inRange(image_data, lower, upper)
            
            # Find contours in mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                # Filter small contours
                if w > 10 and h > 10:
                    box = {
                        'label': f'object_{len(boxes)+1}',
                        'bbox': [x, y, x+w, y+h],  # [x_min, y_min, x_max, y_max]
                        'confidence': 1.0,  # Ground truth has perfect confidence
                        'area': w * h
                    }
                    boxes.append(box)
        
        return boxes

    def create_segmentation_annotations(self, image_data):
        """Create segmentation mask annotations"""
        # In a real implementation, this would be a pixel-wise annotation
        # from Isaac Replicator. For simulation, we'll create a simple mask.
        height, width = image_data.shape[:2]
        segmentation = np.zeros((height, width), dtype=np.uint8)
        
        # In real Isaac Sim, this would be the semantic segmentation ground truth
        # For now, we'll just return the shape
        return {
            'shape': [height, width],
            'dtype': str(segmentation.dtype),
            'has_ground_truth': True
        }

    def create_depth_annotations(self, image_data):
        """Create depth map annotations"""
        # In real implementation, this would be the ground truth depth
        # from Isaac Sim's depth sensor
        height, width = image_data.shape[:2]
        depth_map = np.random.uniform(0.1, 10.0, (height, width)).astype(np.float32)
        
        return {
            'shape': [height, width],
            'dtype': str(depth_map.dtype),
            'min_depth': float(np.min(depth_map)),
            'max_depth': float(np.max(depth_map)),
            'has_ground_truth': True
        }

    def create_pose_annotations(self):
        """Create pose annotations for objects"""
        # In real implementation, this would be ground truth poses
        # from Isaac Sim's USD scene graph
        return {
            'reference_frame': 'world',
            'has_ground_truth': True
        }

    def save_metadata(self):
        """Save generation metadata and statistics"""
        metadata = {
            'generation_stats': self.generation_stats,
            'parameters': {
                'scene_path': self.scene_path,
                'output_path': self.output_path,
                'image_count': self.image_count,
                'domain_randomization': self.domain_randomization,
                'lighting_variation': self.lighting_variation,
                'texture_variation': self.texture_variation,
                'physics_enabled': self.physics_enabled,
                'annotations': self.annotations
            },
            'generated_images_count': len(self.generated_images),
            'annotations_types': self.annotations,
            'generation_duration_seconds': (
                self.generation_stats['end_time'] - self.generation_stats['start_time']
            ).total_seconds() if self.generation_stats['start_time'] and self.generation_stats['end_time'] else 0
        }
        
        metadata_path = os.path.join(self.output_path, 'metadata', 'generation_metadata.json')
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2, default=str)
        
        self.get_logger().info(f'Metadata saved to: {metadata_path}')


def main(args=None):
    rclpy.init(args=args)
    
    generator = SyntheticDatasetGenerator()
    
    try:
        rclpy.spin(generator)
    except KeyboardInterrupt:
        pass
    finally:
        generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()