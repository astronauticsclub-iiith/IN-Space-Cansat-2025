"use client"

import { useState, useEffect } from "react"
import LocationView from "./LocationView"
import OrientationView from "./OrientationView"

export default function CanSatTracker() {
  const [location, setLocation] = useState({ x: 0, y: 0, z: 0 })
  const [orientation, setOrientation] = useState({ pitch: 0, yaw: 0, roll: 0 })

  // Simulate data updates
  useEffect(() => {
    const interval = setInterval(() => {
      setLocation((prev) => ({
        x: prev.x + (Math.random() - 0.5) * 0.1,
        y: prev.y + (Math.random() - 0.5) * 0.1,
        z: prev.z + (Math.random() - 0.5) * 0.1,
      }))
      setOrientation((prev) => ({
        pitch: (prev.pitch + Math.random() * 5) % 360,
        yaw: (prev.yaw + Math.random() * 5) % 360,
        roll: (prev.roll + Math.random() * 5) % 360,
      }))
    }, 1000)

    return () => clearInterval(interval)
  }, [])

  return (
    <div className="flex flex-col md:flex-row w-full h-full">
      <div className="w-full md:w-1/2 h-1/2 md:h-full">
        <LocationView location={location} />
      </div>
      <div className="w-full md:w-1/2 h-1/2 md:h-full">
        <OrientationView orientation={orientation} />
      </div>
    </div>
  )
}

