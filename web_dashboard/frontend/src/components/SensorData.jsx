import React from 'react';

export default function SensorData({ data }) {
  return (
    <div>
      <h2>Sensor Data</h2>
      <pre>{JSON.stringify(data, null, 2)}</pre>
    </div>
  );
}
