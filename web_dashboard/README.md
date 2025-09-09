# Web Dashboard for Cloud Digital Twin

This folder contains the **frontend and backend** code for the web dashboard of the Cloud Digital Twin ROS2 project.

## Structure

- **backend/**: Python Flask/FastAPI backend
  - `app.py` — main backend application
  - `requirements.txt` — Python dependencies

- **frontend/**: React frontend
  - `package.json` — frontend dependencies
  - `src/` — React source code
    - `App.jsx` — main App component
    - `index.jsx` — entry point
    - `components/` — reusable components
      - `RobotStatus.jsx` — displays robot status
      - `SensorData.jsx` — displays sensor readings
      - `TaskScheduler.jsx` — task scheduling UI

## Usage

1. **Backend**
   ```bash
   cd backend
   pip install -r requirements.txt
   python app.py
