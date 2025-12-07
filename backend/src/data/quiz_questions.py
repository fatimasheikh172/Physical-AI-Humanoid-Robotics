# Week 1 Quiz Questions

# This file contains quiz questions related to ROS 2 fundamentals

quiz_questions = [
    {
        "question": "What does ROS stand for?",
        "options": [
            "Robot Operating System",
            "Remote Operating Service",
            "Robotic Operations Suite",
            "Runtime Operating System"
        ],
        "correct_answer": "Robot Operating System",
        "explanation": "ROS stands for Robot Operating System, though it's not a traditional operating system but rather a middleware framework for robotics development."
    },
    {
        "question": "What does DDS stand for in the context of ROS 2?",
        "options": [
            "Data Distribution Service",
            "Distributed Development System",
            "Dynamic Data Sharing",
            "Digital Data Standard"
        ],
        "correct_answer": "Data Distribution Service",
        "explanation": "DDS (Data Distribution Service) is the middleware standard that ROS 2 uses for communication between nodes."
    },
    {
        "question": "Which ROS 2 distribution is LTS (Long Term Support)?",
        "options": [
            "Rolling",
            "Iron",
            "Humble",
            "Galactic"
        ],
        "correct_answer": "Humble",
        "explanation": "ROS 2 Humble Hawksbill is the LTS version with support until May 2027, making it ideal for educational and production use."
    },
    {
        "question": "What are the three main communication patterns in ROS 2?",
        "options": [
            "Topics, Services, Actions",
            "Publishers, Subscribers, Clients",
            "Messages, Services, Commands",
            "TCP, UDP, DDS"
        ],
        "correct_answer": "Topics, Services, Actions",
        "explanation": "ROS 2 uses three main communication patterns: Topics (asynchronous pub/sub), Services (synchronous request/response), and Actions (asynchronous with feedback for long-running tasks)."
    },
    {
        "question": "What is the primary purpose of a ROS 2 workspace?",
        "options": [
            "A directory where you develop and build ROS 2 packages",
            "A GUI interface for ROS 2",
            "A simulation environment",
            "A cloud service for robotics"
        ],
        "correct_answer": "A directory where you develop and build ROS 2 packages",
        "explanation": "A ROS 2 workspace is a directory structure that organizes your ROS 2 source code, builds, and installations."
    }
]

def get_week1_quiz():
    """
    Returns the quiz questions for Week 1
    """
    return quiz_questions