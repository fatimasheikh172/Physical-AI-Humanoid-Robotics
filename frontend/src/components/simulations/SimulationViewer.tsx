import React, { Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, useGLTF } from '@react-three/drei';
import { Simulation } from '../../types/Simulation';
import { LoadingSpinner } from '../common/LoadingError';

interface SimulationViewerProps {
  simulation: Simulation;
}

const Model: React.FC<{ modelPath: string }> = ({ modelPath }) => {
  const { scene } = useGLTF(modelPath);
  return <primitive object={scene} />;
};

const SimulationViewer: React.FC<SimulationViewerProps> = ({ simulation }) => {
  return (
    <div className="border rounded-lg overflow-hidden bg-gray-900">
      <div className="p-3 bg-gray-800 text-white flex justify-between items-center">
        <h3 className="font-medium">{simulation.title}</h3>
        <div className="flex space-x-2">
          <button className="text-xs bg-gray-700 hover:bg-gray-600 px-2 py-1 rounded">
            Reset
          </button>
          <button className="text-xs bg-blue-600 hover:bg-blue-500 px-2 py-1 rounded">
            Run
          </button>
        </div>
      </div>
      
      <div className="relative h-96 w-full">
        <Suspense fallback={<LoadingSpinner message="Loading simulation..." />}>
          <Canvas
            camera={{ position: [0, 2, 5], fov: 50 }}
            gl={{ antialias: true }}
          >
            <ambientLight intensity={0.5} />
            <spotLight position={[10, 10, 10]} angle={0.15} penumbra={1} />
            <pointLight position={[-10, -10, -10]} />
            
            {/* Add the 3D model if model path is available */}
            {simulation.model3D && <Model modelPath={simulation.model3D} />}
            
            {/* Add orbit controls for camera manipulation */}
            <OrbitControls 
              enablePan={true}
              enableZoom={true}
              enableRotate={true}
            />
          </Canvas>
        </Suspense>
      </div>
      
      <div className="p-3 bg-gray-100 border-t">
        <p className="text-sm text-gray-600">{simulation.description}</p>
        
        <div className="mt-2 flex flex-wrap gap-2">
          <div className="text-xs px-2 py-1 bg-gray-200 rounded">
            Complexity: {simulation.complexity}
          </div>
          <div className="text-xs px-2 py-1 bg-gray-200 rounded">
            Interactivity: {simulation.interactivityLevel}/10
          </div>
        </div>
      </div>
    </div>
  );
};

export default SimulationViewer;