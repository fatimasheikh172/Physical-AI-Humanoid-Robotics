#!/usr/bin/env python3

"""
Perception Training Script for Isaac Sim Synthetic Data
Trains perception models using synthetic datasets generated from Isaac Sim
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import json
import os
import sys
import argparse
from datetime import datetime
import subprocess
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from PIL import Image as PILImage


class IsaacSyntheticDataset(Dataset):
    """PyTorch Dataset for Isaac Sim synthetic datasets"""
    
    def __init__(self, dataset_path, transform=None, task='object_detection'):
        """
        Args:
            dataset_path (str): Path to the synthetic dataset directory
            transform (callable, optional): Optional transform to be applied on images
            task (str): Task type ('object_detection', 'segmentation', 'classification')
        """
        self.dataset_path = dataset_path
        self.transform = transform
        self.task = task
        self.image_paths = []
        self.annotations = []
        
        # Load dataset metadata
        self._load_dataset()
        
    def _load_dataset(self):
        """Load image paths and annotations from the dataset directory"""
        images_dir = os.path.join(self.dataset_path, 'images')
        annotations_dir = os.path.join(self.dataset_path, 'annotations')
        
        # Get all image files
        for img_file in os.listdir(images_dir):
            if img_file.lower().endswith(('.png', '.jpg', '.jpeg')):
                img_path = os.path.join(images_dir, img_file)
                annotation_file = img_file.replace('.png', '_annotations.json').replace('.jpg', '_annotations.json').replace('.jpeg', '_annotations.json')
                annotation_path = os.path.join(annotations_dir, annotation_file)
                
                if os.path.exists(annotation_path):
                    self.image_paths.append(img_path)
                    self.annotations.append(annotation_path)
    
    def __len__(self):
        return len(self.image_paths)
    
    def __getitem__(self, idx):
        # Load image
        img_path = self.image_paths[idx]
        image = PILImage.open(img_path).convert('RGB')
        
        # Load annotations
        annotation_path = self.annotations[idx]
        with open(annotation_path, 'r') as f:
            annotation_data = json.load(f)
        
        # Apply transforms if specified
        if self.transform:
            image = self.transform(image)
        
        if self.task == 'classification':
            # Return image and class label
            label = annotation_data.get('class_label', 0)
            return image, label
        elif self.task == 'object_detection':
            # Return image and bounding box annotations
            bboxes = annotation_data.get('bounding_boxes', [])
            labels = [bbox.get('label', 'unknown') for bbox in bboxes]
            return image, {'boxes': bboxes, 'labels': labels}
        elif self.task == 'segmentation':
            # Return image and segmentation mask
            seg_mask_path = annotation_data.get('segmentation_mask_path', None)
            if seg_mask_path:
                seg_mask = PILImage.open(seg_mask_path)
                if self.transform:
                    seg_mask = self.transform(seg_mask)
                return image, seg_mask
            else:
                # Generate a simple segmentation mask (in real case, this would be from annotations)
                width, height = image.size
                seg_mask = torch.zeros(height, width, dtype=torch.long)
                return image, seg_mask
        else:
            # Default case, just return image
            return image, annotation_data


class PerceptionTrainingNode(Node):
    def __init__(self):
        super().__init__('perception_training_node')
        
        # Declare parameters
        self.declare_parameter('dataset_path', '/path/to/synthetic/dataset')
        self.declare_parameter('model_architecture', 'yolo')
        self.declare_parameter('model_output_path', '/path/to/trained/model')
        self.declare_parameter('epochs', 100)
        self.declare_parameter('batch_size', 16)
        self.declare_parameter('learning_rate', 0.001)
        self.declare_parameter('task', 'object_detection')
        self.declare_parameter('validation_split', 0.2)
        self.declare_parameter('save_frequency', 10)
        
        # Get parameters
        self.dataset_path = self.get_parameter('dataset_path').value
        self.model_architecture = self.get_parameter('model_architecture').value
        self.model_output_path = self.get_parameter('model_output_path').value
        self.epochs = self.get_parameter('epochs').value
        self.batch_size = self.get_parameter('batch_size').value
        self.learning_rate = self.get_parameter('learning_rate').value
        self.task = self.get_parameter('task').value
        self.validation_split = self.get_parameter('validation_split').value
        self.save_frequency = self.get_parameter('save_frequency').value
        
        # Initialize training components
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = None
        self.optimizer = None
        self.criterion = None
        self.train_loader = None
        self.val_loader = None
        
        # Publishers
        self.status_publisher = self.create_publisher(String, '/training/status', 10)
        self.metrics_publisher = self.create_publisher(String, '/training/metrics', 10)
        
        # Timer for periodic updates
        self.training_timer = self.create_timer(10.0, self.publish_training_status)
        
        # Initialize training components
        self.setup_training()
        
        self.get_logger().info('Perception Training Node initialized')
    
    def setup_training(self):
        """Setup the training components based on parameters"""
        try:
            self.get_logger().info(f'Setting up training for task: {self.task}')
            
            # Setup transforms
            transform = transforms.Compose([
                transforms.Resize((416, 416)),  # Resize to standard input size
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                                   std=[0.229, 0.224, 0.225])  # ImageNet normalization
            ])
            
            # Load dataset
            self.get_logger().info(f'Loading dataset from: {self.dataset_path}')
            dataset = IsaacSyntheticDataset(
                self.dataset_path, 
                transform=transform, 
                task=self.task
            )
            
            # Split dataset into train and validation
            dataset_size = len(dataset)
            val_size = int(self.validation_split * dataset_size)
            train_size = dataset_size - val_size
            
            train_dataset, val_dataset = torch.utils.data.random_split(
                dataset, 
                [train_size, val_size]
            )
            
            # Create data loaders
            self.train_loader = DataLoader(
                train_dataset, 
                batch_size=self.batch_size, 
                shuffle=True,
                num_workers=4
            )
            self.val_loader = DataLoader(
                val_dataset, 
                batch_size=self.batch_size, 
                shuffle=False,
                num_workers=4
            )
            
            self.get_logger().info(f'Training samples: {len(train_dataset)}')
            self.get_logger().info(f'Validation samples: {len(val_dataset)}')
            
            # Load or create model based on architecture
            self.create_model()
            
            # Setup optimizer and criterion
            if self.task == 'classification':
                self.criterion = nn.CrossEntropyLoss()
            elif self.task == 'object_detection':
                # For object detection, we'll use a combination of losses
                # This is a simplified version - in real implementation, would use appropriate detection loss
                self.criterion = nn.MSELoss()  # Simplified for demo
            elif self.task == 'segmentation':
                self.criterion = nn.CrossEntropyLoss()
            else:
                self.criterion = nn.MSELoss()
            
            self.optimizer = optim.Adam(
                self.model.parameters(), 
                lr=self.learning_rate
            )
            
            self.get_logger().info('Training components setup completed')
            
        except Exception as e:
            self.get_logger().error(f'Error in setup_training: {e}')
            self.get_logger().error(traceback.format_exc())
    
    def create_model(self):
        """Create or load the model based on architecture"""
        try:
            self.get_logger().info(f'Creating model: {self.model_architecture}')
            
            if self.model_architecture.lower() == 'yolo':
                # Placeholder - in real implementation, would use a YOLO variant
                self.model = self.create_yolo_model()
            elif self.model_architecture.lower() == 'resnet':
                # Use a ResNet-based model for classification
                self.model = self.create_resnet_model()
            elif self.model_architecture.lower() == 'unet':
                # Use U-Net for segmentation
                self.model = self.create_unet_model()
            else:
                # Default to a custom CNN
                self.get_logger().warn(f'Unknown architecture: {self.model_architecture}, using default CNN')
                self.model = self.create_default_cnn()
            
            # Move model to device
            self.model = self.model.to(self.device)
            
            # Print model summary
            self.get_logger().info(f'Model created with {sum(p.numel() for p in self.model.parameters()):,} parameters')
            
        except Exception as e:
            self.get_logger().error(f'Error creating model: {e}')
            self.get_logger().error(traceback.format_exc())
            raise
    
    def create_yolo_model(self):
        """Create a YOLO-inspired model for object detection"""
        # This is a simplified version - in real implementation would use actual YOLO architecture
        class YoloLikeModel(nn.Module):
            def __init__(self, num_classes=80, num_anchors=3):
                super(YoloLikeModel, self).__init__()
                
                # Feature extraction backbone (simplified)
                self.backbone = nn.Sequential(
                    # Conv Block 1
                    nn.Conv2d(3, 32, kernel_size=3, padding=1),
                    nn.BatchNorm2d(32),
                    nn.ReLU(inplace=True),
                    nn.MaxPool2d(kernel_size=2, stride=2),
                    
                    # Conv Block 2
                    nn.Conv2d(32, 64, kernel_size=3, padding=1),
                    nn.BatchNorm2d(64),
                    nn.ReLU(inplace=True),
                    nn.MaxPool2d(kernel_size=2, stride=2),
                    
                    # Conv Block 3
                    nn.Conv2d(64, 128, kernel_size=3, padding=1),
                    nn.BatchNorm2d(128),
                    nn.ReLU(inplace=True),
                    nn.MaxPool2d(kernel_size=2, stride=2),
                    
                    # Conv Block 4
                    nn.Conv2d(128, 256, kernel_size=3, padding=1),
                    nn.BatchNorm2d(256),
                    nn.ReLU(inplace=True),
                    nn.MaxPool2d(kernel_size=2, stride=2),
                    
                    # Conv Block 5
                    nn.Conv2d(256, 512, kernel_size=3, padding=1),
                    nn.BatchNorm2d(512),
                    nn.ReLU(inplace=True),
                    nn.MaxPool2d(kernel_size=2, stride=2),
                )
                
                # Detection head
                self.detection_head = nn.Conv2d(512, num_anchors * (5 + num_classes), kernel_size=1)
                
            def forward(self, x):
                # Extract features
                features = self.backbone(x)
                
                # Generate detection outputs
                detections = self.detection_head(features)
                
                return detections
        
        return YoloLikeModel(num_classes=10)  # Simplified for demo
    
    def create_resnet_model(self):
        """Create a ResNet-based model for classification"""
        # In a real implementation, we would use torchvision.models.resnet
        # For this demo, we'll create a simple CNN that mimics ResNet's approach
        class SimpleResBlock(nn.Module):
            def __init__(self, in_channels, out_channels, stride=1):
                super(SimpleResBlock, self).__init__()
                
                self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1, bias=False)
                self.bn1 = nn.BatchNorm2d(out_channels)
                self.relu = nn.ReLU(inplace=True)
                self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, stride=1, padding=1, bias=False)
                self.bn2 = nn.BatchNorm2d(out_channels)
                
                self.shortcut = nn.Sequential()
                if stride != 1 or in_channels != out_channels:
                    self.shortcut = nn.Sequential(
                        nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=stride, bias=False),
                        nn.BatchNorm2d(out_channels)
                    )
                    
            def forward(self, x):
                identity = self.shortcut(x)
                
                out = self.conv1(x)
                out = self.bn1(out)
                out = self.relu(out)
                
                out = self.conv2(out)
                out = self.bn2(out)
                
                out += identity
                out = self.relu(out)
                
                return out
        
        class SimpleResNet(nn.Module):
            def __init__(self, num_classes=10, in_channels=3):
                super(SimpleResNet, self).__init__()
                
                self.in_channels = 64
                
                # Initial conv layer
                self.conv1 = nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)
                self.bn1 = nn.BatchNorm2d(64)
                self.relu = nn.ReLU(inplace=True)
                self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
                
                # Residual blocks
                self.layer1 = self._make_layer(64, 2, stride=1)
                self.layer2 = self._make_layer(128, 2, stride=2)
                self.layer3 = self._make_layer(256, 2, stride=2)
                self.layer4 = self._make_layer(512, 2, stride=2)
                
                # Global average pooling and classifier
                self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
                self.fc = nn.Linear(512, num_classes)
                
            def _make_layer(self, out_channels, num_blocks, stride):
                layers = []
                layers.append(SimpleResBlock(self.in_channels, out_channels, stride))
                self.in_channels = out_channels
                for _ in range(1, num_blocks):
                    layers.append(SimpleResBlock(self.in_channels, out_channels))
                return nn.Sequential(*layers)
                
            def forward(self, x):
                x = self.conv1(x)
                x = self.bn1(x)
                x = self.relu(x)
                x = self.maxpool(x)
                
                x = self.layer1(x)
                x = self.layer2(x)
                x = self.layer3(x)
                x = self.layer4(x)
                
                x = self.avgpool(x)
                x = torch.flatten(x, 1)
                x = self.fc(x)
                
                return x
        
        return SimpleResNet(num_classes=10)  # Simplified for demo
    
    def create_unet_model(self):
        """Create a U-Net model for segmentation"""
        class UNetDoubleConv(nn.Module):
            def __init__(self, in_channels, out_channels):
                super(UNetDoubleConv, self).__init__()
                
                self.conv = nn.Sequential(
                    nn.Conv2d(in_channels, out_channels, 3, padding=1),
                    nn.BatchNorm2d(out_channels),
                    nn.ReLU(inplace=True),
                    nn.Conv2d(out_channels, out_channels, 3, padding=1),
                    nn.BatchNorm2d(out_channels),
                    nn.ReLU(inplace=True)
                )
            
            def forward(self, x):
                return self.conv(x)
        
        class UNetUp(nn.Module):
            def __init__(self, in_channels, out_channels):
                super(UNetUp, self).__init__()
                
                self.up = nn.ConvTranspose2d(in_channels, out_channels, 2, stride=2)
                self.conv = UNetDoubleConv(out_channels * 2, out_channels)
                
            def forward(self, x1, x2):
                x1 = self.up(x1)
                
                # Pad x1 to match dimensions of x2
                diffY = x2.size()[2] - x1.size()[2]
                diffX = x2.size()[3] - x1.size()[3]
                
                x1 = F.pad(x1, [diffX // 2, diffX - diffX // 2,
                                diffY // 2, diffY - diffY // 2])
                
                x = torch.cat([x2, x1], dim=1)
                return self.conv(x)
        
        class UNet(nn.Module):
            def __init__(self, in_channels=3, num_classes=21):
                super(UNet, self).__init__()
                
                self.in_channels = in_channels
                self.num_classes = num_classes
                
                # Encoder
                self.inc = UNetDoubleConv(in_channels, 64)
                self.down1 = nn.Conv2d(64, 128, 2, stride=2)
                self.conv1 = UNetDoubleConv(128, 128)
                self.down2 = nn.Conv2d(128, 256, 2, stride=2)
                self.conv2 = UNetDoubleConv(256, 256)
                self.down3 = nn.Conv2d(256, 512, 2, stride=2)
                self.conv3 = UNetDoubleConv(512, 512)
                
                # Bottleneck
                self.bottom = UNetDoubleConv(512, 1024)
                
                # Decoder
                self.up1 = UNetUp(1024, 512)
                self.up2 = UNetUp(512, 256)
                self.up3 = UNetUp(256, 128)
                self.outc = nn.Conv2d(128, num_classes, 1)
                
            def forward(self, x):
                # Encoder
                x1 = self.inc(x)
                x2 = self.conv1(self.down1(x1))
                x3 = self.conv2(self.down2(x2))
                x4 = self.conv3(self.down3(x3))
                
                # Bottleneck
                x5 = self.bottom(x4)
                
                # Decoder
                x = self.up1(x5, x4)
                x = self.up2(x, x3)
                x = self.up3(x, x2)
                x = self.outc(x)
                
                return x
        
        return UNet(in_channels=3, num_classes=10)  # Simplified for demo
    
    def create_default_cnn(self):
        """Create a default CNN model"""
        class DefaultCNN(nn.Module):
            def __init__(self, num_classes=10):
                super(DefaultCNN, self).__init__()
                
                self.features = nn.Sequential(
                    nn.Conv2d(3, 32, 3, padding=1),
                    nn.ReLU(inplace=True),
                    nn.MaxPool2d(2, 2),
                    nn.Conv2d(32, 64, 3, padding=1),
                    nn.ReLU(inplace=True),
                    nn.MaxPool2d(2, 2),
                    nn.Conv2d(64, 128, 3, padding=1),
                    nn.ReLU(inplace=True),
                    nn.MaxPool2d(2, 2),
                )
                
                self.classifier = nn.Sequential(
                    nn.Dropout(),
                    nn.Linear(128 * 52 * 52, 512),  # Assuming input is 416x416 -> 52x52 after pooling
                    nn.ReLU(inplace=True),
                    nn.Dropout(),
                    nn.Linear(512, num_classes),
                )
                
            def forward(self, x):
                x = self.features(x)
                x = torch.flatten(x, 1)
                x = self.classifier(x)
                return x
        
        return DefaultCNN(num_classes=10)
    
    def train_model(self):
        """Train the perception model"""
        try:
            self.get_logger().info('Starting training process')
            
            # Training metrics
            train_losses = []
            val_losses = []
            train_accuracies = []
            val_accuracies = []
            
            for epoch in range(self.epochs):
                self.get_logger().info(f'Starting Epoch {epoch+1}/{self.epochs}')
                
                # Training phase
                self.model.train()
                running_train_loss = 0.0
                correct_train = 0
                total_train = 0
                
                for batch_idx, (data, targets) in enumerate(self.train_loader):
                    data, targets = data.to(self.device), targets.to(self.device)
                    
                    # Zero gradients
                    self.optimizer.zero_grad()
                    
                    # Forward pass
                    outputs = self.model(data)
                    
                    # Compute loss
                    if self.task == 'classification':
                        # For classification, targets should be class indices
                        if isinstance(targets, dict):
                            # Use the first sample's class as target for this demo
                            target_class = torch.randint(0, 10, (data.size(0),)).to(self.device)
                        else:
                            target_class = targets.squeeze().long()
                        
                        loss = self.criterion(outputs, target_class)
                        
                        # Compute accuracy
                        _, predicted = torch.max(outputs.data, 1)
                        total_train += target_class.size(0)
                        correct_train += (predicted == target_class).sum().item()
                    else:
                        # For other tasks, use MSE as a simplified loss
                        if isinstance(targets, dict):
                            # For object detection, we might have bounding boxes
                            # For this demo, we'll create a simple target
                            target_tensor = torch.randn_like(outputs)
                        else:
                            target_tensor = targets.float()
                        
                        loss = self.criterion(outputs, target_tensor)
                    
                    # Backward pass
                    loss.backward()
                    self.optimizer.step()
                    
                    running_train_loss += loss.item()
                    
                    if batch_idx % 10 == 0:  # Print every 10 batches
                        self.get_logger().info(f'Epoch [{epoch+1}/{self.epochs}], Batch [{batch_idx+1}/{len(self.train_loader)}], Loss: {loss.item():.4f}')
                
                # Validation phase
                self.model.eval()
                running_val_loss = 0.0
                correct_val = 0
                total_val = 0
                
                with torch.no_grad():
                    for data, targets in self.val_loader:
                        data, targets = data.to(self.device), targets.to(self.device)
                        
                        outputs = self.model(data)
                        
                        if self.task == 'classification':
                            if isinstance(targets, dict):
                                target_class = torch.randint(0, 10, (data.size(0),)).to(self.device)
                            else:
                                target_class = targets.squeeze().long()
                            
                            loss = self.criterion(outputs, target_class)
                            
                            _, predicted = torch.max(outputs.data, 1)
                            total_val += target_class.size(0)
                            correct_val += (predicted == target_class).sum().item()
                        else:
                            if isinstance(targets, dict):
                                target_tensor = torch.randn_like(outputs)
                            else:
                                target_tensor = targets.float()
                            
                            loss = self.criterion(outputs, target_tensor)
                        
                        running_val_loss += loss.item()
                
                # Calculate averages
                avg_train_loss = running_train_loss / len(self.train_loader)
                avg_val_loss = running_val_loss / len(self.val_loader)
                
                if self.task == 'classification':
                    train_acc = 100 * correct_train / total_train if total_train > 0 else 0
                    val_acc = 100 * correct_val / total_val if total_val > 0 else 0
                else:
                    train_acc = 0
                    val_acc = 0
                
                # Store metrics
                train_losses.append(avg_train_loss)
                val_losses.append(avg_val_loss)
                train_accuracies.append(train_acc)
                val_accuracies.append(val_acc)
                
                # Print epoch results
                self.get_logger().info(f'Epoch {epoch+1}/{self.epochs}:')
                self.get_logger().info(f'  Train Loss: {avg_train_loss:.4f}, Train Acc: {train_acc:.2f}%')
                self.get_logger().info(f'  Val Loss: {avg_val_loss:.4f}, Val Acc: {val_acc:.2f}%')
                
                # Publish training status
                status_msg = String()
                status_msg.data = f'Epoch {epoch+1}/{self.epochs} - Train Loss: {avg_train_loss:.4f}, Val Loss: {avg_val_loss:.4f}'
                self.status_publisher.publish(status_msg)
                
                # Save model periodically
                if (epoch + 1) % self.save_frequency == 0:
                    self.save_model(epoch)
                
                # Early stopping check (simplified)
                if len(val_losses) > 5 and all(val_losses[-1] >= loss for loss in val_losses[-5:-1]):
                    self.get_logger().warn('Validation loss has plateaued for 5 epochs - consider early stopping')
            
            # Training complete
            self.get_logger().info('Training completed')
            
            # Save final model
            self.save_model('final')
            
            # Save metrics
            self.save_metrics(train_losses, val_losses, train_accuracies, val_accuracies)
            
        except Exception as e:
            self.get_logger().error(f'Error during training: {e}')
            self.get_logger().error(traceback.format_exc())
    
    def save_model(self, epoch):
        """Save the trained model"""
        try:
            # Create output directory if it doesn't exist
            os.makedirs(self.model_output_path, exist_ok=True)
            
            # Define model path
            model_path = os.path.join(self.model_output_path, f'model_epoch_{epoch}.pth')
            
            # Save model
            torch.save({
                'epoch': epoch if isinstance(epoch, int) else 0,
                'model_state_dict': self.model.state_dict(),
                'optimizer_state_dict': self.optimizer.state_dict(),
                'loss': self.criterion,
                'architecture': self.model_architecture,
                'task': self.task,
                'dataset_path': self.dataset_path
            }, model_path)
            
            self.get_logger().info(f'Model saved to: {model_path}')
            
            # Also save as ONNX for Isaac ROS deployment
            self.export_to_tensorrt(model_path)
            
        except Exception as e:
            self.get_logger().error(f'Error saving model: {e}')
            self.get_logger().error(traceback.format_exc())
    
    def export_to_tensorrt(self, model_path):
        """Export the model to TensorRT format for Isaac ROS deployment"""
        try:
            onnx_path = model_path.replace('.pth', '.onnx')
            
            # Create a dummy input to trace the model
            dummy_input = torch.randn(1, 3, 416, 416).to(self.device)
            
            # Export to ONNX
            torch.onnx.export(
                self.model,
                dummy_input,
                onnx_path,
                export_params=True,
                opset_version=11,
                do_constant_folding=True,
                input_names=['input'],
                output_names=['output'],
                dynamic_axes={
                    'input': {0: 'batch_size'},
                    'output': {0: 'batch_size'}
                }
            )
            
            self.get_logger().info(f'Model exported to ONNX: {onnx_path}')
            
            # For Isaac ROS, we would convert ONNX to TensorRT plan file
            # This would typically be done via Isaac ROS tools
            trt_path = onnx_path.replace('.onnx', '.plan')
            
            # In a real implementation, use TensorRT Python API or tensorrt executable
            # For now, we'll simulate this conversion
            self.get_logger().info(f'Model prepared for TensorRT: {trt_path}')
            
            # Create a simple TensorRT conversion script
            trt_convert_script = os.path.join(self.model_output_path, f'trt_convert_{os.path.basename(model_path)}.py')
            with open(trt_convert_script, 'w') as f:
                f.write(f'''#!/usr/bin/env python3
"""
TensorRT Conversion Script for Isaac ROS Deployment
Generated for model: {model_path}

This script converts the PyTorch model to TensorRT format for hardware acceleration.
"""
import tensorrt as trt
import numpy as np
import onnx
from polygraphy import util as pg_util
from polygraphy.backend.common import bytes_from_path
from polygraphy.backend.trt import (
    CreateConfig,
    engine_from_bytes,
    engine_from_network,
    network_from_onnx_path,
    save_engine,
)

def convert_onnx_to_tensorrt(onnx_path, trt_path, precision="fp16"):
    """
    Convert ONNX model to TensorRT engine
    """
    logger = trt.Logger(trt.Logger.WARNING)
    
    # Load ONNX model
    onnx_model_path = "{onnx_path}"
    
    # Create TensorRT builder
    builder = trt.Builder(logger)
    config = builder.create_builder_config()
    
    # Create network
    EXPLICIT_BATCH = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(EXPLICIT_BATCH)
    parser = trt.OnnxParser(network, logger)
    
    # Parse ONNX file
    with open(onnx_model_path, 'rb') as model:
        if not parser.parse(model.read()):
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            raise RuntimeError("Could not parse ONNX")

    # Configure optimization profiles
    profile = builder.create_optimization_profile()
    profile.set_shape("input", min=(1, 3, 416, 416), opt=(1, 3, 416, 416), max=(8, 3, 416, 416))
    config.add_optimization_profile(profile)
    
    # Set precision based on parameter
    if precision == "fp16":
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
        else:
            print("FP16 not supported on this platform, using FP32")
    
    # Build engine
    serialized_engine = builder.build_serialized_network(network, config)
    
    # Save engine
    with open(trt_path, "wb") as f:
        f.write(serialized_engine)
    
    print(f"TensorRT engine saved to: {{trt_path}}")

if __name__ == "__main__":
    import sys
    onnx_file = "{onnx_path}"
    trt_file = "{trt_path}"
    precision = "fp16" if len(sys.argv) < 2 else sys.argv[1]
    
    convert_onnx_to_tensorrt(onnx_file, trt_file, precision)
''')
            
            self.get_logger().info(f'TensorRT conversion script created: {trt_convert_script}')
            
        except Exception as e:
            self.get_logger().error(f'Error exporting model to TensorRT: {e}')
            self.get_logger().error(traceback.format_exc())
    
    def save_metrics(self, train_losses, val_losses, train_accuracies, val_accuracies):
        """Save training metrics to file"""
        try:
            metrics_path = os.path.join(self.model_output_path, 'training_metrics.json')
            
            metrics = {
                'train_losses': train_losses,
                'val_losses': val_losses,
                'train_accuracies': train_accuracies,
                'val_accuracies': val_accuracies,
                'final_train_loss': train_losses[-1] if train_losses else None,
                'final_val_loss': val_losses[-1] if val_losses else None,
                'final_train_accuracy': train_accuracies[-1] if train_accuracies else None,
                'final_val_accuracy': val_accuracies[-1] if val_accuracies else None,
                'epochs_trained': len(train_losses),
                'hyperparameters': {
                    'learning_rate': self.learning_rate,
                    'batch_size': self.batch_size,
                    'epochs': self.epochs,
                    'model_architecture': self.model_architecture,
                    'task': self.task
                }
            }
            
            with open(metrics_path, 'w') as f:
                json.dump(metrics, f, indent=2)
            
            self.get_logger().info(f'Training metrics saved to: {metrics_path}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving metrics: {e}')
            self.get_logger().error(traceback.format_exc())
    
    def publish_training_status(self):
        """Publish training status periodically"""
        try:
            status_msg = String()
            status_msg.data = 'Training node active'
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing training status: {e}')
    
    def start_training_process(self):
        """Start the training process as a background task"""
        self.training_thread = threading.Thread(target=self.train_model)
        self.training_thread.start()
        self.get_logger().info('Training process started in background thread')


def main(args=None):
    rclpy.init(args=args)
    
    training_node = PerceptionTrainingNode()
    
    # Start the training process
    training_node.start_training_process()
    
    try:
        rclpy.spin(training_node)
    except KeyboardInterrupt:
        print('Training interrupted by user')
    finally:
        training_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()