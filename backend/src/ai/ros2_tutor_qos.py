"""
AI Tutor for ROS 2 Communication Patterns (Week 2)
Enhances the AI tutor with QoS concepts for ROS 2 communication patterns
"""
from typing import Dict, Any
import json
from .ros2_tutor import ROS2Tutor


class Week2ROS2Tutor(ROS2Tutor):
    """
    Extended AI Tutor class to provide explanations for ROS 2 communication patterns (Week 2 concepts)
    """
    
    def __init__(self):
        super().__init__()
        
        # Extend knowledge base with Week 2 concepts
        self.knowledge_base.update({
            "topics_services_actions_comparison": {
                "explanation": "ROS 2 provides three main communication patterns: Topics (asynchronous, many-to-many), Services (synchronous, one-to-one), Actions (asynchronous with feedback for long-running tasks).",
                "patterns": {
                    "topics": {
                        "characteristics": [
                            "Asynchronous communication",
                            "Many-to-many (multiple publishers/subscribers)",
                            "Publish/subscribe pattern",
                            "Used for continuous data streams like sensor readings"
                        ],
                        "best_for": [
                            "Sensor data broadcasting",
                            "Robot state information",
                            "Continuous status updates"
                        ]
                    },
                    "services": {
                        "characteristics": [
                            "Synchronous communication",
                            "One-to-one (client-server)",
                            "Request/Response pattern",
                            "Used for immediate results"
                        ],
                        "best_for": [
                            "Configuration changes",
                            "Map lookup requests",
                            "Immediate computations"
                        ]
                    },
                    "actions": {
                        "characteristics": [
                            "Asynchronous communication with feedback",
                            "One-to-one (client-server)", 
                            "Goal/Result/Feedback pattern",
                            "Used for long-running tasks with progress updates"
                        ],
                        "best_for": [
                            "Navigation goals",
                            "Manipulation tasks",
                            "Calibration procedures"
                        ]
                    }
                }
            },
            "quality_of_service": {
                "explanation": "Quality of Service (QoS) profiles in ROS 2 determine how messages are delivered between nodes, including reliability, durability, history, and other aspects.",
                "policies": {
                    "reliability": {
                        "reliable": "Guarantees all messages are delivered, may block if delivery fails",
                        "best_effort": "Messages may be lost, no blocking occurs"
                    },
                    "durability": {
                        "volatile": "New subscribers don't receive historical messages",
                        "transient_local": "New subscribers receive the last value for each topic"
                    },
                    "history": {
                        "keep_last": "Store a fixed number of the most recent messages",
                        "keep_all": "Store all messages (resource constraints apply)"
                    }
                },
                "common_use_cases": {
                    "reliable_keep_last": "Most sensor data requiring guaranteed delivery",
                    "best_effort_keep_last": "Video streams where frame loss is acceptable",
                    "reliable_transient_local": "Maps and parameters that new nodes should receive",
                    "best_effort_volatile": "Fast-changing sensor data where only newest values matter"
                }
            }
        })
    
    async def compare_policies(self, policy1: str, policy2: str) -> str:
        """
        Compare two QoS policies based on their characteristics and use cases
        """
        qos_info = self.knowledge_base.get("quality_of_service", {})
        
        if policy1.lower() == "reliable" and policy2.lower() == "best_effort":
            comparison = (
                "## Reliability Policy Comparison\n\n"
                "### RELIABLE:\n"
                "- Guarantees all messages are delivered\n"
                "- May block if delivery fails\n"
                "- Uses more resources for reliable delivery\n"
                "- **Best for**: Critical data that must not be lost (robot control commands, safety messages)\n\n"
                
                "### BEST_EFFORT:\n"
                "- Messages may be lost\n"
                "- No blocking occurs\n"
                "- More resource efficient\n"
                "- **Best for**: Data where occasional loss is acceptable (video streams, some sensor data)\n\n"
                
                "### When to Use Each:\n"
                "- **RELIABLE**: When message loss is not acceptable\n"
                "- **BEST_EFFORT**: When timeliness is more important than delivery guarantee\n"
            )
            return comparison
        
        elif policy1.lower() == "transient_local" and policy2.lower() == "volatile":
            comparison = (
                "## Durability Policy Comparison\n\n"
                "### TRANSIENT_LOCAL:\n"
                "- New subscribers receive historical messages\n"
                "- Maintains the last message for each topic\n"
                "- **Best for**: Maps, parameters, configuration data that new nodes should receive\n\n"
                
                "### VOLATILE:\n"
                "- New subscribers don't receive historical messages\n"
                "- Only current messages are available\n"
                "- **Best for**: Real-time sensor data where only current values matter\n\n"
                
                "### When to Use Each:\n"
                "- **TRANSIENT_LOCAL**: When late-joining nodes need initial state\n"
                "- **VOLATILE**: When only real-time data is important\n"
            )
            return comparison
        
        else:
            return "I can compare these QoS policies: reliable vs best_effort, transient_local vs volatile. Please specify two policies to compare."
    
    async def suggest_qos(self, use_case: str) -> str:
        """
        Suggest appropriate QoS settings based on the robot use case
        """
        use_case_lower = use_case.lower()
        
        if any(term in use_case_lower for term in ["control", "safety", "critical", "command"]):
            suggestion = (
                "## QoS Suggestion for Safety/Critical Control\n\n"
                "For safety-critical or control commands, I recommend:\n\n"
                
                "**Reliability**: RELIABLE\n"
                "- Ensures all control commands are delivered\n\n"
                
                "**Durability**: VOLATILE\n"
                "- Only current commands matter for safety\n\n"
                
                "**History**: KEEP_LAST with depth 1\n"
                "- Only store the most recent command\n\n"
                
                "This ensures that control commands are always delivered while maintaining efficiency."
            )
        
        elif any(term in use_case_lower for term in ["video", "stream", "camera", "image"]):
            suggestion = (
                "## QoS Suggestion for Video Streams\n\n"
                "For video or image streams, I recommend:\n\n"
                
                "**Reliability**: BEST_EFFORT\n"
                "- Allows for frame loss without blocking\n\n"
                
                "**Durability**: VOLATILE\n"
                "- Only current frames are relevant\n\n"
                
                "**History**: KEEP_LAST with depth 1\n"
                "- Only store the most recent frame\n\n"
                
                "This ensures smooth video streaming where occasional frame loss is acceptable."
            )
        
        elif any(term in use_case_lower for term in ["map", "parameter", "configuration", "initial"]):
            suggestion = (
                "## QoS Suggestion for Map/Parameter Data\n\n"
                "For maps or configuration data, I recommend:\n\n"
                
                "**Reliability**: RELIABLE\n"
                "- Ensures complete map/parameter delivery\n\n"
                
                "**Durability**: TRANSIENT_LOCAL\n"
                "- New nodes get the last value immediately\n\n"
                
                "**History**: KEEP_ALL or KEEP_LAST with appropriate depth\n"
                "- Depends on how many values you want to maintain\n\n"
                
                "This ensures that late-joining nodes get the necessary configuration data."
            )
        
        elif any(term in use_case_lower for term in ["sensor", "lidar", "range"]):
            suggestion = (
                "## QoS Suggestion for Sensor Data\n\n"
                "For sensor data like LIDAR or range sensors:\n\n"
                
                "**Reliability**: BEST_EFFORT\n"
                "- Allows for occasional sensor readings to be dropped\n\n"
                
                "**Durability**: VOLATILE\n"
                "- Only current sensor readings are relevant\n\n"
                
                "**History**: KEEP_LAST with depth depending on application (usually 1-10)\n"
                "- Store enough samples for filtering or buffering if needed\n\n"
                
                "This balances between responsiveness and resource usage for sensor data."
            )
        
        else:
            suggestion = (
                "## General QoS Guidance\n\n"
                "For selecting QoS policies, consider:\n\n"
                
                "1. **Reliability**:\n"
                "   - Use RELIABLE for critical data that must be delivered\n"
                "   - Use BEST_EFFORT for data where occasional loss is acceptable\n\n"
                
                "2. **Durability**:\n"
                "   - Use TRANSIENT_LOCAL for data new nodes should receive immediately\n"
                "   - Use VOLATILE for real-time only data\n\n"
                
                "3. **History**:\n"
                "   - Use KEEP_LAST for recent values only\n"
                "   - Use KEEP_ALL for complete history (careful with resource usage)\n\n"
                
                "Could you provide more specific details about your use case for a more targeted recommendation?"
            )
        
        return suggestion


# Global instance of the extended tutor
week2_tutor = Week2ROS2Tutor()