import React from 'react';

export default function RobotStatus({ status }) {
  return (
    <div>
      <h2>Robot Status</h2>
      <ul>
        {Object.entries(status).map(([robot, s]) => (
          <li key={robot}>{robot}: {s}</li>
        ))}
      </ul>
    </div>
  );
}
