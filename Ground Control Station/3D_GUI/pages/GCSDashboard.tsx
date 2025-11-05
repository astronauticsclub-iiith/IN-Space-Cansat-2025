// app/GCSDashboard.tsx
"use client";

import React, { useState, useEffect } from "react";
import { useTheme } from "next-themes";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import Terminal from "@/components/Terminal";
import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  ResponsiveContainer,
} from "recharts";
import CanSat from "@/CanSat"; // A simple red cylinder simulation of the CanSat
import PositionGraph from "@/components/PositionGraph";
import { Sun, Moon } from "lucide-react";

// Dummy launch station coordinates (for relative positioning)
const LAUNCH_LAT = 28.6139;
const LAUNCH_LNG = 77.209;
const LAUNCH_ALT = 1000;

// Define the TypeScript interface for the API response
interface TelemetryData {
  timestamp: number;
  altitude: number;
  temperature: number;
  pressure: number;
  humidity: number;
  battery: number;
  gnss: { lat: number; lng: number };
  gyro: { pitch: number; yaw: number; roll: number };
  acceleration: { x: number; y: number; z: number };
  airQuality: number;
  sensor_status: {
    gnss: string;
    altimetry: string;
    pressure: string;
    temperature: string;
    gyro: string;
    power: string;
    airQuality: string;
  };
  system_state: number;
};

const getColorForStatus = (status: string) => {
  if (status === "ACTIVE") return "green";
  if (status === "ERROR") return "red";
  if (status === "STANDBY") return "blue";
  return "gray";
};

const GCSDashboard: React.FC = () => {
  const { theme, setTheme } = useTheme();

  // Sensor state variables
  const [altitude, setAltitude] = useState(LAUNCH_ALT);
  const [temperature, setTemperature] = useState(25);
  const [pressure, setPressure] = useState(1013);
  const [humidity, setHumidity] = useState(50);
  const [battery, setBattery] = useState(100);
  const [gnssLat, setGnssLat] = useState(LAUNCH_LAT);
  const [gnssLng, setGnssLng] = useState(LAUNCH_LNG);
  const [gyroOrientation, setGyroOrientation] = useState({
    pitch: 0,
    yaw: 0,
    roll: 0,
  });
  const [acceleration, setAcceleration] = useState({ x: 0, y: 0, z: 0 });
  const [airQuality, setAirQuality] = useState(10);

  // Data mode state variables
  const [dataMode, setDataMode] = useState("random");
  const [csvLoaded, setCsvLoaded] = useState(false);
  const [csvRows, setCsvRows] = useState(0);

  // Chart data and relative location
  const [chartData, setChartData] = useState<
    { time: number; altitude: number; temperature: number; humidity: number; pressure: number }[]
  >([]);
  const [relativeLocation, setRelativeLocation] = useState({ x: 0, y: 0, z: 0 });
  const [thirdMetric, setThirdMetric] = useState<"humidity" | "pressure">("humidity");

  // Sensor status from API
  const [sensorStatus, setSensorStatus] = useState({
    gnss: "ACTIVE",
    altimetry: "ACTIVE",
    pressure: "ACTIVE",
    temperature: "ACTIVE",
    gyro: "ACTIVE",
    power: "ACTIVE",
    airQuality: "ACTIVE",
  });

  // Fetch telemetry data from API every second
  useEffect(() => {
    const fetchTelemetry = async () => {
      try {
        const res = await fetch("http://localhost:8002/telemetry");
        const data: TelemetryData = await res.json();
        console.log("Fetched telemetry:", data);

        // Update sensor values from API data
        setAltitude(data.altitude);
        setTemperature(data.temperature);
        setPressure(data.pressure);
        setHumidity(data.humidity);
        setBattery(data.battery);
        setGnssLat(data.gnss.lat);
        setGnssLng(data.gnss.lng);
        setGyroOrientation({
          pitch: data.gyro.pitch,
          yaw: data.gyro.yaw,
          roll: data.gyro.roll,
        });
        setAcceleration({
          x: data.acceleration.x,
          y: data.acceleration.y,
          z: data.acceleration.z,
        });
        setAirQuality(data.airQuality);

        // Update sensor status from API
        setSensorStatus({
          gnss: data.sensor_status.gnss,
          altimetry: data.sensor_status.altimetry,
          pressure: data.sensor_status.pressure,
          temperature: data.sensor_status.temperature,
          gyro: data.sensor_status.gyro,
          power: data.sensor_status.power,
          airQuality: data.sensor_status.airQuality,
        });

        // Append new data point to chart data (limit to 20 entries)
        setChartData((prevData) => [
          ...prevData.slice(-19),
          {
            time: prevData.length,
            altitude: data.altitude,
            temperature: data.temperature,
            humidity: data.humidity,
            pressure: data.pressure,
          },
        ]);

        // Update relative location (rough conversion from degrees to meters)
        setRelativeLocation({
          x: (data.gnss.lng - LAUNCH_LNG) * 111320,
          y: data.altitude - LAUNCH_ALT,
          z: (data.gnss.lat - LAUNCH_LAT) * 110540,
        });
      } catch (error) {
        console.error("Error fetching telemetry:", error);
      }
    };

    fetchTelemetry();
    const interval = setInterval(fetchTelemetry, 1000);
    return () => clearInterval(interval);
  }, []);

  const handleSendCommand = async (command: string) => {
    console.log("Command sent:", command);
    
    try {
      const response = await fetch("http://localhost:8002/command", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          command: command,
          timestamp: Date.now() / 1000, // Convert to Unix timestamp
          source: "GUI"
        }),
      });

      if (response.ok) {
        const result = await response.json();
        console.log("Command response:", result);
      } else {
        console.error("Failed to send command:", response.statusText);
      }
    } catch (error) {
      console.error("Error sending command:", error);
    }
  };

  const handleCalibrate = () => {
    // Optionally, you could send a calibration command to the API.
    setAltitude(LAUNCH_ALT);
    setTemperature(25);
    setPressure(1013);
    setHumidity(50);
    setBattery(100);
    setGnssLat(LAUNCH_LAT);
    setGnssLng(LAUNCH_LNG);
    setGyroOrientation({ pitch: 0, yaw: 0, roll: 0 });
    setAirQuality(10);
  };

  // Fetch current data mode on component mount
  useEffect(() => {
    const fetchDataMode = async () => {
      try {
        const response = await fetch("http://localhost:8002/config/data-mode");
        if (response.ok) {
          const result = await response.json();
          setDataMode(result.mode);
          setCsvLoaded(result.csv_loaded);
          setCsvRows(result.csv_rows);
          console.log("Current data mode:", result);
        }
      } catch (error) {
        console.error("Error fetching data mode:", error);
      }
    };

    fetchDataMode();
  }, []);

  const handleModeSwitch = async (newMode: string) => {
    try {
      const response = await fetch("http://localhost:8002/config/data-mode", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          mode: newMode
        }),
      });

      if (response.ok) {
        const result = await response.json();
        setDataMode(result.mode);
        console.log("Mode switched to:", result.mode);
        
        // Also refresh the data mode status
        const statusResponse = await fetch("http://localhost:8002/config/data-mode");
        if (statusResponse.ok) {
          const statusResult = await statusResponse.json();
          setCsvLoaded(statusResult.csv_loaded);
          setCsvRows(statusResult.csv_rows);
        }
      } else {
        console.error("Failed to switch mode:", response.statusText);
      }
    } catch (error) {
      console.error("Error switching mode:", error);
    }
  };

  return (
    <div className="h-screen p-2 bg-gray-100 dark:bg-gray-900 text-gray-900 dark:text-gray-100">
      <header className="mb-2 flex justify-between items-center">
        <h1 className="text-xl font-bold">GCS Dashboard</h1>
        <div className="flex items-center space-x-4">
          {/* Data Mode Controls */}
          <div className="flex items-center space-x-2">
            <span className="text-sm font-medium">
              Mode: <span className={`font-bold ${dataMode === 'csv' ? 'text-green-500' : 'text-blue-500'}`}>
                {dataMode.toUpperCase()}
              </span>
              {dataMode === 'csv' && csvLoaded && (
                <span className="text-xs text-gray-500 ml-1">({csvRows} rows)</span>
              )}
            </span>
            <button
              onClick={() => handleModeSwitch('random')}
              className={`px-3 py-1 text-xs rounded transition-colors ${
                dataMode === 'random' 
                  ? 'bg-blue-500 text-white' 
                  : 'bg-gray-200 dark:bg-gray-700 hover:bg-gray-300 dark:hover:bg-gray-600'
              }`}
            >
              Random
            </button>
            <button
              onClick={() => handleModeSwitch('csv')}
              className={`px-3 py-1 text-xs rounded transition-colors ${
                dataMode === 'csv' 
                  ? 'bg-green-500 text-white' 
                  : 'bg-gray-200 dark:bg-gray-700 hover:bg-gray-300 dark:hover:bg-gray-600'
              }`}
              disabled={!csvLoaded}
              title={!csvLoaded ? 'No CSV file loaded' : 'Switch to CSV mode'}
            >
              CSV {!csvLoaded && '(No file)'}
            </button>
          </div>

          {/* Dark Mode Controls */}
          <div className="flex items-center space-x-2">
            <span className="text-sm font-medium">Dark Mode</span>
            <button
              onClick={() => setTheme(theme === "dark" ? "light" : "dark")}
              className="flex items-center justify-center p-2 bg-gray-200 dark:bg-gray-700 rounded-full hover:bg-gray-300 dark:hover:bg-gray-600 transition-colors"
            >
              {theme === "dark" ? (
                <Sun className="h-6 w-6 text-yellow-500" />
              ) : (
                <Moon className="h-6 w-6 text-blue-500" />
              )}
            </button>
          </div>
        </div>
      </header>

      {/* Grid with auto-sized rows */}
      <div className="grid grid-cols-2 gap-1 auto-rows-auto">
        {/* Sensor Readouts */}
        <div className="overflow-auto p-1">
          <Card>
            <CardHeader>
              <CardTitle>Sensor Readouts</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="grid grid-cols-2 gap-2 text-sm">
                {/* GNSS */}
                <div>
                  <p>
                    <strong>GNSS</strong>{" "}
                    <span style={{ color: getColorForStatus(sensorStatus.gnss) }}>
                      ●
                    </span>{" "}
                    <span className="ml-1 text-xs">{sensorStatus.gnss}</span>
                  </p>
                  <p>Lat: {gnssLat.toFixed(6)}°</p>
                  <p>Lng: {gnssLng.toFixed(6)}°</p>
                </div>
                {/* Altimetry */}
                <div>
                  <p>
                    <strong>Altimetry</strong>{" "}
                    <span
                      style={{ color: getColorForStatus(sensorStatus.altimetry) }}
                    >
                      ●
                    </span>{" "}
                    <span className="ml-1 text-xs">{sensorStatus.altimetry}</span>
                  </p>
                  <p>{altitude.toFixed(2)} m</p>
                </div>
                {/* Pressure */}
                <div>
                  <p>
                    <strong>Pressure</strong>{" "}
                    <span
                      style={{ color: getColorForStatus(sensorStatus.pressure) }}
                    >
                      ●
                    </span>{" "}
                    <span className="ml-1 text-xs">{sensorStatus.pressure}</span>
                  </p>
                  <p>{pressure.toFixed(2)} hPa</p>
                </div>
                {/* Temperature */}
                <div>
                  <p>
                    <strong>Temperature</strong>{" "}
                    <span
                      style={{ color: getColorForStatus(sensorStatus.temperature) }}
                    >
                      ●
                    </span>{" "}
                    <span className="ml-1 text-xs">
                      {sensorStatus.temperature}
                    </span>
                  </p>
                  <p>{temperature.toFixed(2)} °C</p>
                </div>
                {/* Gyro / Accel */}
                <div>
                  <p>
                    <strong>Gyro / Accel</strong>{" "}
                    <span style={{ color: getColorForStatus(sensorStatus.gyro) }}>
                      ●
                    </span>{" "}
                    <span className="ml-1 text-xs">{sensorStatus.gyro}</span>
                  </p>
                  <p>Pitch: {gyroOrientation.pitch.toFixed(2)}°</p>
                  <p>Yaw: {gyroOrientation.yaw.toFixed(2)}°</p>
                  <p>Roll: {gyroOrientation.roll.toFixed(2)}°</p>
                  <p>
                    Accel: {acceleration.x.toFixed(2)}, {acceleration.y.toFixed(2)}, {acceleration.z.toFixed(2)}
                  </p>
                </div>
                {/* Power */}
                <div>
                  <p>
                    <strong>Power</strong>{" "}
                    <span style={{ color: getColorForStatus(sensorStatus.power) }}>
                      ●
                    </span>{" "}
                    <span className="ml-1 text-xs">{sensorStatus.power}</span>
                  </p>
                  <p>{battery.toFixed(2)}%</p>
                </div>
                {/* Air Quality */}
                <div>
                  <p>
                    <strong>Air Quality (PM2.5)</strong>{" "}
                    <span
                      style={{ color: getColorForStatus(sensorStatus.airQuality) }}
                    >
                      ●
                    </span>{" "}
                    <span className="ml-1 text-xs">
                      {sensorStatus.airQuality}
                    </span>
                  </p>
                  <p>{airQuality.toFixed(2)} µg/m³</p>
                </div>
              </div>
              <div className="mt-1 text-xs font-bold">
                System State: {0 /* or from API if available */}
              </div>
              <div className="mt-1">
                <Button onClick={handleCalibrate} size="sm">
                  Calibrate Sensors
                </Button>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Telemetry Charts */}
        <div className="overflow-auto p-1">
          <Card>
            <CardHeader>
              <CardTitle>Telemetry Charts</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="flex justify-end mb-2 space-x-2">
                <Button
                  size="sm"
                  variant={thirdMetric === "humidity" ? "outline" : "default"}
                  onClick={() => setThirdMetric("humidity")}
                >
                  Humidity
                </Button>
                <Button
                  size="sm"
                  variant={thirdMetric === "pressure" ? "outline" : "default"}
                  onClick={() => setThirdMetric("pressure")}
                >
                  Pressure
                </Button>
              </div>
              <div className="grid grid-cols-3 gap-2">
                {/* Altitude Graph */}
                <div className="border p-1">
                  <h2 className="text-xs font-bold text-center">
                    Altitude (m)
                  </h2>
                  <ResponsiveContainer width="100%" height={100}>
                    <LineChart data={chartData}>
                      <XAxis dataKey="time" tick={{ fontSize: 10 }} axisLine={{ stroke: "#ccc" }} tickLine={{ stroke: "#ccc" }} />
                      <YAxis tick={{ fontSize: 10 }} axisLine={{ stroke: "#ccc" }} tickLine={{ stroke: "#ccc" }} domain={[0, 1100]} />
                      <Tooltip wrapperStyle={{ fontSize: "10px" }} />
                      <Line type="monotone" dataKey="altitude" stroke="#8884d8" strokeWidth={2} dot={false} />
                    </LineChart>
                  </ResponsiveContainer>
                </div>
                {/* Temperature Graph */}
                <div className="border p-1">
                  <h2 className="text-xs font-bold text-center">
                    Temperature (°C)
                  </h2>
                  <ResponsiveContainer width="100%" height={100}>
                    <LineChart data={chartData}>
                      <XAxis dataKey="time" tick={{ fontSize: 10 }} axisLine={{ stroke: "#ccc" }} tickLine={{ stroke: "#ccc" }} />
                      <YAxis tick={{ fontSize: 10 }} axisLine={{ stroke: "#ccc" }} tickLine={{ stroke: "#ccc" }} domain={["dataMin - 5", "dataMax + 5"]} />
                      <Tooltip wrapperStyle={{ fontSize: "10px" }} />
                      <Line type="monotone" dataKey="temperature" stroke="#82ca9d" strokeWidth={2} dot={false} />
                    </LineChart>
                  </ResponsiveContainer>
                </div>
                {/* Third Graph: Humidity or Pressure */}
                <div className="border p-1">
                  <h2 className="text-xs font-bold text-center">
                    {thirdMetric === "humidity"
                      ? "Humidity (%)"
                      : "Pressure (hPa)"}
                  </h2>
                  <ResponsiveContainer width="100%" height={100}>
                    <LineChart data={chartData}>
                      <XAxis dataKey="time" tick={{ fontSize: 10 }} axisLine={{ stroke: "#ccc" }} tickLine={{ stroke: "#ccc" }} />
                      <YAxis
                        tick={{ fontSize: 10 }}
                        axisLine={{ stroke: "#ccc" }}
                        tickLine={{ stroke: "#ccc" }}
                        domain={thirdMetric === "humidity" ? [0, 100] : [1000, 1020]}
                      />
                      <Tooltip wrapperStyle={{ fontSize: "10px" }} />
                      <Line type="monotone" dataKey={thirdMetric} stroke={thirdMetric === "humidity" ? "#ffc658" : "#ff7300"} strokeWidth={2} dot={false} />
                    </LineChart>
                  </ResponsiveContainer>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* 3D Simulation */}
        <div className="overflow-auto p-1">
          <Card>
            <CardHeader>
              <CardTitle>3D Simulation</CardTitle>
            </CardHeader>
            <CardContent>
              <Canvas camera={{ position: [5, 5, 5] }}>
                <ambientLight intensity={0.5} />
                <pointLight position={[10, 10, 10]} />
                <OrbitControls />
                {/* Only show orientation; fixed position */}
                <CanSat
                  position={[0, 0, 0]}
                  rotation={[
                    (gyroOrientation.pitch * Math.PI) / 180,
                    (gyroOrientation.yaw * Math.PI) / 180,
                    (gyroOrientation.roll * Math.PI) / 180,
                  ]}
                />
                <gridHelper args={[1, 1]} />
                <axesHelper args={[2]} />
              </Canvas>
            </CardContent>
          </Card>
        </div>

        {/* 3D Position Graph */}
        <div className="overflow-auto p-1">
          <Card>
            <CardHeader>
              <CardTitle>3D Position Graph</CardTitle>
            </CardHeader>
            <CardContent>
              <PositionGraph
                position={{
                  x: relativeLocation.x / 100000,
                  y: relativeLocation.y / 100,
                  z: relativeLocation.z / 100000,
                }}
              />
            </CardContent>
          </Card>
        </div>
      </div>

      {/* Terminal Section */}
      <div className="mt-1">
        <Card>
          <CardHeader>
            <CardTitle>Terminal</CardTitle>
          </CardHeader>
          <CardContent className="overflow-auto p-1">
            <Terminal onSendCommand={handleSendCommand} />
          </CardContent>
        </Card>
      </div>
    </div>
  );
};

export default GCSDashboard;
