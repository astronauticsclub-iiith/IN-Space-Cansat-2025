import { Canvas } from "@react-three/fiber"
import { OrbitControls } from "@react-three/drei"
import CanSat from "./CanSat"

interface OrientationViewProps {
  orientation: { pitch: number; yaw: number; roll: number }
}

export default function OrientationView({ orientation }: OrientationViewProps) {
  return (
    <div className="w-full h-full">
      <h2 className="absolute top-0 left-0 m-4 text-white z-10">Orientation View</h2>
      <Canvas camera={{ position: [5, 5, 5] }}>
        <ambientLight intensity={0.5} />
        <pointLight position={[10, 10, 10]} />
        <OrbitControls />
        <CanSat
          rotation={[
            (orientation.pitch * Math.PI) / 180,
            (orientation.yaw * Math.PI) / 180,
            (orientation.roll * Math.PI) / 180,
          ]}
        />
      </Canvas>
    </div>
  )
}

