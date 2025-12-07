import React, { useState } from 'react';
import { Project } from '../../types/Project';
import { Chapter } from '../../types/Chapter';
import { Simulation } from '../../types/Simulation';
import SimulationViewer from '../simulations/SimulationViewer';

interface CapstoneProjectTemplateProps {
  project: Project;
  chapter?: Chapter; // The chapter this project belongs to
  simulation?: Simulation; // Associated simulation if applicable
}

const CapstoneProjectTemplate: React.FC<CapstoneProjectTemplateProps> = ({ project, chapter, simulation }) => {
  const [currentStep, setCurrentStep] = useState(0);
  const [projectResources, setProjectResources] = useState<string[]>(project.resources);
  const [projectSubmission, setProjectSubmission] = useState('');

  // Mock steps for the capstone project
  const projectSteps = [
    {
      title: 'Project Overview',
      description: project.description,
      content: project.requirements,
    },
    {
      title: 'Project Resources',
      description: 'Download and review the resources needed for this project',
      content: project.resources.map((resource, idx) => (
        <a 
          key={idx} 
          href={resource} 
          className="block p-2 bg-blue-50 rounded-md mb-2 text-blue-600 hover:underline"
          target="_blank"
          rel="noopener noreferrer"
        >
          Resource {idx + 1}: {resource}
        </a>
      )).map(item => <div key={item.key}>{item}</div>),
    },
    {
      title: 'Implementation Guidelines',
      description: 'Follow these steps to implement your solution',
      content: 'Detailed implementation steps would go here...',
    },
    {
      title: 'Testing & Validation',
      description: 'Test your solution using the provided simulation',
      content: simulation ? (
        <SimulationViewer simulation={simulation} />
      ) : 'Simulation environment will be available here once configured.',
    },
    {
      title: 'Submission',
      description: 'Submit your completed project for evaluation',
      content: (
        <div>
          <textarea
            value={projectSubmission}
            onChange={(e) => setProjectSubmission(e.target.value)}
            placeholder="Provide a description of your implementation, challenges faced, and how you solved them..."
            className="w-full p-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
            rows={6}
          />
          <div className="mt-4">
            <label className="block text-sm font-medium text-gray-700 mb-2">
              Upload your solution (if applicable)
            </label>
            <input 
              type="file" 
              className="block w-full text-sm text-gray-500
                file:mr-4 file:py-2 file:px-4
                file:rounded-md file:border-0
                file:text-sm file:font-semibold
                file:bg-blue-600 file:text-white
                hover:file:bg-blue-700"
            />
          </div>
        </div>
      ),
    },
    {
      title: 'Evaluation',
      description: 'Review the criteria for project evaluation',
      content: (
        <div>
          <h4 className="font-medium mb-2">Evaluation Criteria:</h4>
          <ul className="list-disc pl-5 space-y-1">
            {project.evaluationCriteria.map((criterion, idx) => (
              <li key={idx}>{criterion}</li>
            ))}
          </ul>
        </div>
      ),
    },
  ];

  const nextStep = () => {
    if (currentStep < projectSteps.length - 1) {
      setCurrentStep(currentStep + 1);
    }
  };

  const prevStep = () => {
    if (currentStep > 0) {
      setCurrentStep(currentStep - 1);
    }
  };

  const submitProject = () => {
    // In a real implementation, this would submit the project to the backend
    console.log('Project submitted:', {
      projectId: project.id,
      submission: projectSubmission,
      step: currentStep,
    });
    alert('Project submitted successfully! In a real implementation, this would be sent to the server.');
  };

  return (
    <div className="bg-white rounded-lg shadow-lg overflow-hidden">
      <div className="p-6">
        <div className="mb-6">
          <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-purple-100 text-purple-800 mb-2">
            Capstone Project
          </span>
          <h1 className="text-2xl font-bold text-gray-900">{project.title}</h1>
          {chapter && (
            <p className="text-gray-600 mt-1">Part of: {chapter.title}</p>
          )}
        </div>

        {/* Progress indicator */}
        <div className="mb-6">
          <div className="flex justify-between text-sm text-gray-600 mb-1">
            <span>Step {currentStep + 1} of {projectSteps.length}</span>
            <span>{Math.round(((currentStep + 1) / projectSteps.length) * 100)}% Complete</span>
          </div>
          <div className="w-full bg-gray-200 rounded-full h-2.5">
            <div 
              className="bg-purple-600 h-2.5 rounded-full transition-all duration-300" 
              style={{ width: `${((currentStep + 1) / projectSteps.length) * 100}%` }}
            ></div>
          </div>
        </div>

        {/* Project step content */}
        <div className="mb-6">
          <h2 className="text-xl font-semibold text-gray-800 mb-2">
            {projectSteps[currentStep].title}
          </h2>
          <p className="text-gray-600 mb-4">
            {projectSteps[currentStep].description}
          </p>
          <div className="bg-gray-50 p-4 rounded-md border">
            {typeof projectSteps[currentStep].content === 'string' 
              ? <div dangerouslySetInnerHTML={{ __html: projectSteps[currentStep].content.replace(/\n/g, '<br />') }} />
              : projectSteps[currentStep].content}
          </div>
        </div>

        {/* Navigation */}
        <div className="flex justify-between pt-4 border-t">
          <button
            onClick={prevStep}
            disabled={currentStep === 0}
            className={`px-4 py-2 rounded-md ${
              currentStep === 0 
                ? 'bg-gray-200 text-gray-500 cursor-not-allowed' 
                : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
            }`}
          >
            Previous
          </button>

          {currentStep < projectSteps.length - 1 ? (
            <button
              onClick={nextStep}
              className="px-4 py-2 bg-purple-600 text-white rounded-md hover:bg-purple-700"
            >
              Next
            </button>
          ) : (
            <button
              onClick={submitProject}
              className="px-4 py-2 bg-green-600 text-white rounded-md hover:bg-green-700"
            >
              Submit Project
            </button>
          )}
        </div>
      </div>
    </div>
  );
};

export default CapstoneProjectTemplate;