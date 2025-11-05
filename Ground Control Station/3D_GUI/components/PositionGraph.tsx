// components/PositionGraph.tsx
"use client";
import React from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";

interface PositionGraphProps {
  position: { x: number; y: number; z: number };
}

const PositionGraph: React.FC<PositionGraphProps> = ({ position }) => {
  return (
    <Canvas camera={{ position: [5, 5, 5] }}>
      <ambientLight intensity={0.5} />
      <pointLight position={[10, 10, 10]} />
      <OrbitControls />

      {/* Axes */}
      <line>
        <bufferGeometry>
          <bufferAttribute
            attach="attributes-position"
            count={2}
            array={new Float32Array([0, 0, 0, 10, 0, 0])}
            itemSize={3}
          />
        </bufferGeometry>
        <lineBasicMaterial color="red" />
      </line>
      <line>
        <bufferGeometry>
          <bufferAttribute
            attach="attributes-position"
            count={2}
            array={new Float32Array([0, 0, 0, 0, 10, 0])}
            itemSize={3}
          />
        </bufferGeometry>
        <lineBasicMaterial color="green" />
      </line>
      <line>
        <bufferGeometry>
          <bufferAttribute
            attach="attributes-position"
            count={2}
            array={new Float32Array([0, 0, 0, 0, 0, 10])}
            itemSize={3}
          />
        </bufferGeometry>
        <lineBasicMaterial color="blue" />
      </line>

      {/* Launch Station at Origin */}
      <mesh position={[0, 0, 0]}>
        <sphereGeometry args={[0.2, 16, 16]} />
        <meshStandardMaterial color="white" />
      </mesh>

      {/* Line tracing from origin to current position */}
      <line>
        <bufferGeometry>
          <bufferAttribute
            attach="attributes-position"
            count={2}
            // Create an array from [0,0,0] to the current position
            array={new Float32Array([0, 0, 0, position.x, position.y, position.z])}
            itemSize={3}
          />
        </bufferGeometry>
        <lineBasicMaterial color="yellow" linewidth={2} />
      </line>

      {/* Current Position */}
      <mesh position={[position.x, position.y, position.z]}>
        <sphereGeometry args={[0.3, 16, 16]} />
        <meshStandardMaterial color="yellow" />
      </mesh>
    </Canvas>
  );
};

export default PositionGraph;
