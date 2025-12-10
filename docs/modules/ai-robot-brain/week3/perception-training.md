# Week 3: Perception Model Training with Synthetic Data

## Learning Objectives

By the end of this section, you will be able to:
- Train perception models using synthetic datasets generated from Isaac Sim
- Implement domain randomization techniques for improved sim-to-real transfer
- Optimize models for deployment on the Isaac ROS framework
- Evaluate perception performance in both simulated and real environments
- Integrate trained perception models with the navigation pipeline

## Overview

This week focuses on leveraging the synthetic datasets generated in Week 1 to train perception models that can be deployed in both simulated and real environments. You'll learn to use domain randomization techniques to improve the sim-to-real transfer of these models and optimize them for hardware acceleration using TensorRT.

## 1. Synthetic Dataset Preparation

### Dataset Quality Assessment

Before training, it's important to assess the quality of your synthetic datasets:

```python
# dataset_assessment.py
import numpy as np
import cv2
from PIL import Image
import json
import os
from pathlib import Path
import matplotlib.pyplot as plt

class SyntheticDatasetAssessor:
    def __init__(self, dataset_path):
        self.dataset_path = Path(dataset_path)
        self.stats = {}
    
    def assess_dataset_quality(self):
        """Assess the quality and completeness of synthetic dataset"""
        annotations_file = self.dataset_path / 'annotations.json'
        
        if not annotations_file.exists():
            print(f"No annotations file found at {annotations_file}")
            return {}
        
        with open(annotations_file, 'r') as f:
            annotations = json.load(f)
        
        # Analyze annotation statistics
        total_annotations = 0
        annotation_types = {}
        image_quality_scores = []
        
        for annotation in annotations:
            # Count total annotations
            if 'annotations' in annotation:
                total_annotations += len(annotation['annotations'])
                
                # Count annotation types
                for ann in annotation['annotations']:
                    ann_type = ann.get('type', 'unknown')
                    annotation_types[ann_type] = annotation_types.get(ann_type, 0) + 1
            
            # Assess image quality if image path exists
            image_path = self.dataset_path / annotation.get('filename', '')
            if image_path.exists():
                quality_score = self.assess_image_quality(str(image_path))
                image_quality_scores.append(quality_score)
        
        # Calculate statistics
        avg_image_quality = np.mean(image_quality_scores) if image_quality_scores else 0.0
        std_image_quality = np.std(image_quality_scores) if image_quality_scores else 0.0
        
        self.stats = {
            'total_annotations': total_annotations,
            'annotation_types': annotation_types,
            'image_quality_avg': avg_image_quality,
            'image_quality_std': std_image_quality,
            'total_images': len(annotations),
            'annotation_density': total_annotations / max(1, len(annotations)) if annotations else 0
        }
        
        return self.stats
    
    def assess_image_quality(self, image_path):
        """Assess image quality using various metrics"""
        try:
            # Load image
            img = cv2.imread(image_path)
            if img is None:
                return 0.0
            
            # Convert to grayscale for some metrics
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Calculate quality metrics
            # 1. Sharpness (Laplacian variance)
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            
            # 2. Contrast (standard deviation)
            contrast = np.std(gray)
            
            # 3. Brightness (mean value)
            brightness = np.mean(gray)
            
            # 4. Entropy for information content
            hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
            hist = hist / hist.sum()  # Normalize
            entropy = -np.sum(hist * np.log2(hist + 1e-10))  # Avoid log(0)
            
            # Combine metrics into a single quality score (0-1 scale)
            # Normalize based on expected ranges
            sharpness_score = min(1.0, laplacian_var / 10000.0)
            contrast_score = min(1.0, contrast / 128.0)
            brightness_score = 1.0 - abs(brightness - 128.0) / 128.0  # 0-255 range centered at 128
            entropy_score = entropy / 8.0  # Max entropy for 8-bit is 8
            
            # Weighted average of quality metrics
            quality_score = (0.3 * sharpness_score + 
                           0.2 * contrast_score + 
                           0.2 * brightness_score + 
                           0.3 * entropy_score)
            
            return quality_score
        except Exception as e:
            print(f"Error assessing image quality for {image_path}: {e}")
            return 0.0  # Return lowest quality score on error

    def generate_quality_report(self):
        """Generate a comprehensive quality report"""
        stats = self.assess_dataset_quality()
        
        report = f"""
Synthetic Dataset Quality Report
===============================

Dataset Path: {self.dataset_path}
Total Images: {stats.get('total_images', 0)}
Total Annotations: {stats.get('total_annotations', 0)}
Average Annotations per Image: {stats.get('annotation_density', 0):.2f}

Quality Metrics:
- Average Image Quality Score: {stats.get('image_quality_avg', 0):.3f}
- Image Quality Std Deviation: {stats.get('image_quality_std', 0):.3f}
- Quality Range: {max(0, stats.get('image_quality_avg', 0) - stats.get('image_quality_std', 0)):.3f} - {min(1, stats.get('image_quality_avg', 0) + stats.get('image_quality_std', 0)):.3f}

Annotation Distribution:
"""
        for ann_type, count in stats.get('annotation_types', {}).items():
            report += f"  - {ann_type}: {count}\n"
        
        # Recommendation based on quality scores
        avg_quality = stats.get('image_quality_avg', 0)
        if avg_quality > 0.7:
            report += "\nRecommendation: Dataset quality is excellent for training.\n"
        elif avg_quality > 0.5:
            report += "\nRecommendation: Dataset quality is adequate for training.\n"
        else:
            report += "\nRecommendation: Dataset quality is low - consider regenerating with better parameters.\n"
        
        return report