"""
AI Tutor for Introduction to Physical AI Concepts
Implements the AI tutor functionality for introduction module concepts
"""
from typing import Dict, Any
import json


class IntroductionTutor:
    """
    AI Tutor class to provide explanations for Physical AI and Humanoid Robotics concepts
    """
    
    def __init__(self):
        # Knowledge base for introduction module concepts
        self.knowledge_base = {
            "physical_ai": {
                "explanation": "Physical AI extends traditional AI by integrating it with the physical world. While traditional AI operates primarily in digital spaces, Physical AI operates in and interacts with the real, physical environment.",
                "components": [
                    "Sensing the physical world through various sensors",
                    "Making decisions based on physical constraints and affordances",
                    "Acting upon the physical environment through motors and actuators",
                    "Learning from physical interactions and experiences"
                ],
                "examples": [
                    "A robot that can navigate a real room",
                    "A robotic arm that can pick up physical objects",
                    "A humanoid that can assist in a hospital setting"
                ]
            },
            "embodied_intelligence": {
                "explanation": "Embodied Intelligence is the theory that intelligence is not just a property of the brain, but emerges from the interaction between brain, body, and environment.",
                "principles": [
                    "Physical embodiment shapes cognitive processes",
                    "Intelligence develops through interaction with the environment",
                    "The body's form and sensors influence how information is processed",
                    "Learning happens through physical experience"
                ],
                "robotics_implications": [
                    "Robot's physical form influences its cognitive processes",
                    "Sensory-motor interactions are crucial for intelligence",
                    "Real-world interaction leads to better problem solving"
                ]
            },
            "humanoid_robots": {
                "explanation": "A humanoid robot is a robot with physical features that closely resemble those of a human, including bipedal locomotion, torso, arms, and head.",
                "characteristics": [
                    "Bipedal locomotion with two legs for walking",
                    "Torso that supports arms and head",
                    "Two arms with full range of motion",
                    "Head with sensory systems positioned appropriately"
                ],
                "advantages": [
                    "Can operate in human-centered environments",
                    "Can use spaces and tools designed for humans",
                    "Facilitate natural human-robot interaction"
                ]
            },
            "sensors_actuators": {
                "explanation": "Robots use sensors to perceive their environment and actuators to perform actions.",
                "sensor_types": {
                    "cameras": "Visual perception and object recognition",
                    "lidar": "Distance measurement and 3D mapping",
                    "imu": "Balance, orientation, and motion detection",
                    "force_torque": "Feedback for manipulation and safety",
                    "microphones": "Audio input for communication"
                },
                "actuator_types": {
                    "servo_motors": "Precise position and movement control",
                    "stepper_motors": "Precise discrete position control",
                    "pneumatic": "Simple on/off actuation for gripping"
                }
            },
            "sensor_fusion": {
                "explanation": "Sensor fusion is the combination of data from multiple sensors to create a more complete and accurate understanding than any single sensor could provide.",
                "benefits": [
                    "More reliable perception than single sensors",
                    "Redundancy in case of sensor failure",
                    "Complementary information from different sensors"
                ],
                "examples": [
                    "Combining camera and LiDAR for better object detection",
                    "Using IMU with cameras for stable visual tracking",
                    "Fusing force sensors with vision for safe manipulation"
                ]
            }
        }
    
    async def explain_concept(self, concept: str) -> str:
        """
        Provide an explanation for a Physical AI concept in beginner-friendly language
        """
        concept = concept.lower().replace(" ", "_").replace("-", "_")
        
        if concept in self.knowledge_base:
            concept_data = self.knowledge_base[concept]
            explanation = f"## {concept.replace('_', ' ').title()}\n\n"
            explanation += f"{concept_data['explanation']}\n\n"
            
            if 'components' in concept_data:
                explanation += "### Key Components:\n"
                for component in concept_data['components']:
                    explanation += f"- {component}\n"
            
            if 'principles' in concept_data:
                explanation += "### Key Principles:\n"
                for principle in concept_data['principles']:
                    explanation += f"- {principle}\n"
            
            if 'characteristics' in concept_data:
                explanation += "### Characteristics:\n"
                for characteristic in concept_data['characteristics']:
                    explanation += f"- {characteristic}\n"
            
            if 'advantages' in concept_data:
                explanation += "### Advantages:\n"
                for advantage in concept_data['advantages']:
                    explanation += f"- {advantage}\n"
            
            if 'examples' in concept_data:
                explanation += "### Examples:\n"
                for example in concept_data['examples']:
                    explanation += f"- {example}\n"
            
            if 'sensor_types' in concept_data:
                explanation += "### Sensor Types:\n"
                for sensor, purpose in concept_data['sensor_types'].items():
                    explanation += f"- **{sensor.replace('_', ' ').title()}**: {purpose}\n"
            
            if 'actuator_types' in concept_data:
                explanation += "### Actuator Types:\n"
                for actuator, purpose in concept_data['actuator_types'].items():
                    explanation += f"- **{actuator.replace('_', ' ').title()}**: {purpose}\n"
            
            if 'benefits' in concept_data:
                explanation += "### Benefits:\n"
                for benefit in concept_data['benefits']:
                    explanation += f"- {benefit}\n"
            
            if 'robotics_implications' in concept_data:
                explanation += "### Robotics Implications:\n"
                for implication in concept_data['robotics_implications']:
                    explanation += f"- {implication}\n"
            
            return explanation
        else:
            return f"I don't have detailed information about '{concept.replace('_', ' ')}'. Please ask about concepts like 'Physical AI', 'Embodied Intelligence', 'Humanoid Robots', 'Sensors and Actuators', or 'Sensor Fusion'."
    
    async def compare_sensors(self, sensor1: str, sensor2: str) -> str:
        """
        Compare two sensor types based on their characteristics and use cases
        """
        sensors_info = self.knowledge_base['sensors_actuators']['sensor_types']
        
        s1 = sensor1.lower().replace(" ", "_")
        s2 = sensor2.lower().replace(" ", "_")
        
        if s1 in sensors_info and s2 in sensors_info:
            comparison = f"## Comparison: {s1.replace('_', ' ').title()} vs {s2.replace('_', ' ').title()}\n\n"
            
            comparison += f"### {s1.replace('_', ' ').title()}:\n"
            comparison += f"- Purpose: {sensors_info[s1]}\n"
            
            comparison += f"\n### {s2.replace('_', ' ').title()}:\n"
            comparison += f"- Purpose: {sensors_info[s2]}\n"
            
            # Add comparison details based on common attributes
            comparison += f"\n### When to Use Each:\n"
            if s1 == "cameras" and s2 == "lidar":
                comparison += f"- **Cameras**: Best for visual recognition, color detection, and detailed scene analysis\n"
                comparison += f"- **LiDAR**: Best for accurate distance measurement, 3D mapping, and performance in various lighting conditions\n"
            elif s1 == "lidar" and s2 == "cameras":
                comparison += f"- **LiDAR**: Best for accurate distance measurement, 3D mapping, and performance in various lighting conditions\n"
                comparison += f"- **Cameras**: Best for visual recognition, color detection, and detailed scene analysis\n"
            elif s1 == "imu" and s2 == "cameras":
                comparison += f"- **IMU**: Best for orientation, balance, and motion detection\n"
                comparison += f"- **Cameras**: Best for visual scene understanding and object recognition\n"
            elif s1 == "cameras" and s2 == "imu":
                comparison += f"- **Cameras**: Best for visual scene understanding and object recognition\n"
                comparison += f"- **IMU**: Best for orientation, balance, and motion detection\n"
            else:
                comparison += f"- Each sensor type serves different purposes in robot perception\n"
                comparison += f"- They are often used together in sensor fusion systems\n"
            
            return comparison
        else:
            return f"I can compare sensor types like: {', '.join(sensors_info.keys())}. Please specify two sensors to compare."
    
    async def design_sensor_layout(self, application: str) -> str:
        """
        Provide guidance on designing a sensor layout for a specific application
        """
        application = application.lower()
        
        # Default sensor layout for humanoid robots
        layout = "## Sensor Layout for Humanoid Robots\n\n"
        layout += "For a general-purpose humanoid robot, consider the following sensor layout:\n\n"
        
        layout += "### Head Region:\n"
        layout += "- **Stereo cameras**: For depth perception and visual recognition\n"
        layout += "- **Microphones**: For audio input and speech recognition\n"
        layout += "- **Speaker**: For audio output and speech\n\n"
        
        layout += "### Torso Region:\n"
        layout += "- **IMU**: For balance and orientation\n"
        layout += "- **LiDAR** (optional): For 360° environmental awareness\n\n"
        
        layout += "### Arm Region:\n"
        layout += "- **Joint encoders**: For precise position feedback\n"
        layout += "- **Force/torque sensors**: For safe and controlled manipulation\n\n"
        
        layout += "### Hand Region:\n"
        layout += "- **Tactile sensors**: For object manipulation feedback\n"
        layout += "- **Force sensors**: For grip control\n\n"
        
        layout += "### Leg Region:\n"
        layout += "- **Joint encoders**: For precise movement control\n"
        layout += "- **Force/torque sensors**: For balance and ground contact feedback\n\n"
        
        # If the application is for a specific task, add specialized sensors
        if "navigation" in application:
            layout += "### Additional for Navigation:\n"
            layout += "- **Additional LiDAR**: For enhanced obstacle detection\n"
            layout += "- **Ultrasonic sensors**: For close-proximity obstacle detection\n\n"
        elif "manipulation" in application:
            layout += "### Additional for Manipulation:\n"
            layout += "- **Vision in hands**: Cameras in palms for precise manipulation\n"
            layout += "- **High-resolution tactile sensors**: For fine manipulation control\n\n"
        elif "interaction" in application:
            layout += "### Additional for Human Interaction:\n"
            layout += "- **Depth camera**: For gesture recognition\n"
            layout += "- **More microphones**: For voice localization\n\n"
        
        layout += "### Design Considerations:\n"
        layout += "- Redundancy for safety and reliability\n"
        layout += "- Protection of sensors from damage\n"
        layout += "- Calibration access for maintenance\n"
        layout += "- Power and data connectivity planning\n"
        
        return layout


# Global instance of the tutor
intro_tutor = IntroductionTutor()