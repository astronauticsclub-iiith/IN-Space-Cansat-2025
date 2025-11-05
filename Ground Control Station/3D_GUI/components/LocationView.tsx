import { Canvas } from "@react-three/fiber"
import { OrbitControls } from "@react-three/drei"
import CanSat from "./CanSat"

interface LocationViewProps {
  location: { x: number; y: number; z: number }
}

export default function LocationView({ location }: LocationViewProps) {
  return (
    <div className="w-full h-full">
      <h2 className="absolute top-0 left-0 m-4 text-white z-10">Location View</h2>
      <Canvas camera={{ position: [5, 5, 5] }}>
        <ambientLight intensity={0.5} />
        <pointLight position={[10, 10, 10]} />
        <OrbitControls />
        <CanSat position={[location.x, location.y, location.z]} />
        <gridHelper args={[10, 10]} />
      </Canvas>
    </div>
  )
}

