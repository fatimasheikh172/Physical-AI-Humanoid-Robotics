"""
Student Project Submission
Student's submission for ROS 2 mini-projects or final project
"""
from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, UUID4


class SubmissionStatus(Enum):
    SUBMITTED = "submitted"
    EVALUATED = "evaluated"
    REVISION_REQUESTED = "revision_requested"


class StudentProjectSubmission(BaseModel):
    """
    Student's submission for ROS 2 mini-projects or final project
    Fields:
    - id (UUID, Primary Key)
    - student_id (UUID, Foreign Key to User, Required)
    - project_id (UUID, Foreign Key to ROS2MiniProject, Required)
    - submission_content (Text, Required)
    - file_attachments (JSON Array, Optional - {filename: string, path: string})
    - submitted_at (DateTime, Required)
    - evaluated_at (DateTime, Optional)
    - evaluator_id (UUID, Foreign Key to User, Optional)
    - score (Integer, 0-100, Optional)
    - feedback (Text, Optional)
    - status (Enum: 'submitted', 'evaluated', 'revision_requested', Required, Default: 'submitted')
    """
    id: UUID4
    student_id: UUID4
    project_id: UUID4
    submission_content: str
    file_attachments: Optional[List[dict]] = []
    submitted_at: datetime
    evaluated_at: Optional[datetime] = None
    evaluator_id: Optional[UUID4] = None
    score: Optional[int] = None
    feedback: Optional[str] = None
    status: SubmissionStatus = SubmissionStatus.SUBMITTED
    
    class Config:
        arbitrary_types_allowed = True