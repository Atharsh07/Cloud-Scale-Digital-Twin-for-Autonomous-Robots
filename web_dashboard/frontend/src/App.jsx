import React, { useEffect, useState } from 'react';
import RobotStatus from './components/RobotStatus';
import SensorData from './components/SensorData';
import TaskScheduler from './components/TaskScheduler';
import './App.css';

function App() {
  const [sensor, setSensor] = useState({});
  const [status, setStatus] = useState({});

  useEffect(() => {
    const interval = setInterval(async () => {
      const sensorRes = await fetch('http://localhost:5000/api/sensor');
      const statusRes = await fetch('http://localhost:5000/api/status');
      setSensor(await sensorRes.json());
      setStatus(await statusRes.json());
    }, 500); // update every 0.5s
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="App">
      <h1>Cloud-Scale Digital Twin Dashboard</h1>
      <RobotStatus status={status} />
      <SensorData data={sensor} />
      <TaskScheduler status={status} />
    </div>
  );
}

export default App;
