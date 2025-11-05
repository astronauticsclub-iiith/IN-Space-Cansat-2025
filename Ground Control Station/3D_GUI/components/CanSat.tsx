import { useRef } from "react"
import { useFrame } from "@react-three/fiber"
import { Cylinder } from "@react-three/drei"
import type * as THREE from "three"

interface CanSatProps {
  position?: [number, number, number]
  rotation?: [number, number, number]
}

export default function CanSat({ position = [0, 0, 0], rotation = [0, 0, 0] }: CanSatProps) {
  const meshRef = useRef<THREE.Mesh>(null)

  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.rotation.x = rotation[0]
      meshRef.current.rotation.y = rotation[1]
      meshRef.current.rotation.z = rotation[2]
    }
  })

  return (
    <Cylinder ref={meshRef} args={[0.2, 0.2, 1, 32]} position={position}>
      <meshStandardMaterial color="red" />
    </Cylinder>
  )
}

