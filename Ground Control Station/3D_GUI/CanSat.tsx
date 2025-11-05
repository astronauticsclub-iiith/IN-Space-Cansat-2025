// CanSat.tsx
import React from "react"

// A simple red cylinder to simulate the CanSat
interface CanSatProps {
  position?: [number, number, number]
  rotation?: [number, number, number]
}

const CanSat: React.FC<CanSatProps> = ({ position = [0, 0, 0], rotation = [0, 0, 0] }) => {
  return (
    <mesh position={position} rotation={rotation}>
      {/* Cylinder: [radiusTop, radiusBottom, height, radialSegments] */}
      <cylinderGeometry args={[0.5, 0.5, 2, 32]} />
      <meshStandardMaterial color="red" />
    </mesh>
  )
}

export default CanSat
