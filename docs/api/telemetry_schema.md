# Telemetry Data Schema

This document describes the telemetry data format and the SAE J670 coordinate system used.

## Coordinate System: SAE J670

The telemetry data uses the SAE J670 vehicle-fixed coordinate system:

```
        +Z (Up)
         |
         |    +X (Forward)
         |   /
         |  /
         | /
         |/_________ +Y (Left)
        O (Vehicle Center)
```

### Axis Definitions

| Axis | Direction | Positive Value |
|------|-----------|----------------|
| X | Longitudinal | Front of vehicle |
| Y | Lateral | Left side of vehicle |
| Z | Vertical | Top of vehicle |

### CARLA to SAE J670 Conversion

CARLA uses a slightly different convention (Y points right), so:

```python
SAE_X = CARLA_X      # Same direction
SAE_Y = -CARLA_Y     # Inverted
SAE_Z = CARLA_Z      # Same direction
```

## JSON Schema

### Root Object

```json
{
  "metadata": { ... },
  "frames": [ ... ]
}
```

### Metadata Object

```json
{
  "fps": 20,                              // Frames per second
  "total_frames": 600,                    // Total recorded frames
  "coordinate_system": "SAE_J670",       // Coordinate system identifier
  "coordinate_description": {
    "X": "Forward (positive = front of vehicle)",
    "Y": "Left (positive = left side of vehicle)",
    "Z": "Up (positive = top of vehicle)"
  },
  "units": {
    "position": "meters",
    "velocity": "m/s",
    "acceleration": "m/s^2",
    "angular_velocity": "deg/s",
    "angles": "degrees"
  }
}
```

### Frame Object

```json
{
  "frame": 100,           // Frame index (0-based)
  "t_sim": 5.0,           // Simulation time (frame / fps)
  "t_world": 5.123,       // World elapsed time from CARLA
  "dt": 0.05,             // Delta time since last frame
  "ego": { ... },         // Ego vehicle state
  "actors": [ ... ]       // Nearby actor states
}
```

### Ego Vehicle State

```json
{
  "position": {
    "x": 123.456,         // World X coordinate (meters)
    "y": -67.890,         // World Y coordinate (meters)
    "z": 0.500            // World Z coordinate (meters)
  },
  "velocity": {
    "vx": 12.50,          // Longitudinal velocity (m/s, + = forward)
    "vy": 0.10,           // Lateral velocity (m/s, + = left)
    "vz": 0.00            // Vertical velocity (m/s, + = up)
  },
  "acceleration": {
    "ax": -1.20,          // Longitudinal acceleration (m/s^2)
    "ay": 0.05,           // Lateral acceleration (m/s^2)
    "az": 0.00            // Vertical acceleration (m/s^2)
  },
  "angular_velocity": {
    "roll_rate": 0.00,    // Roll rate (deg/s)
    "pitch_rate": 0.00,   // Pitch rate (deg/s)
    "yaw_rate": 2.50      // Yaw rate (deg/s, + = counter-clockwise)
  },
  "orientation": {
    "roll": 0.00,         // Roll angle (degrees)
    "pitch": -0.50,       // Pitch angle (degrees)
    "yaw": 45.00          // Yaw angle (degrees, heading)
  },
  "speed": 12.50,         // Total speed magnitude (m/s)
  "control": {
    "throttle": 0.50,     // Throttle input [0, 1]
    "brake": 0.00,        // Brake input [0, 1]
    "steer": 0.10         // Steering input [-1, 1]
  }
}
```

### Actor State

```json
{
  "id": 42,                       // CARLA actor ID
  "type": "vehicle",              // Actor type (vehicle/walker/traffic_light)
  "type_id": "vehicle.tesla.model3",  // Blueprint type ID
  "role_name": "merge_vehicle",   // Assigned role name
  "position": {
    "x": 135.123,                 // World X coordinate
    "y": -65.456,                 // World Y coordinate
    "z": 0.500                    // World Z coordinate
  },
  "rotation": {
    "roll": 0.00,                 // Roll angle
    "pitch": 0.00,                // Pitch angle
    "yaw": 47.50                  // Yaw angle
  },
  "distance_to_ego": 12.34,       // Distance to ego vehicle (meters)
  "velocity": {                   // Only for vehicles and walkers
    "x": 13.20,                   // Velocity X component
    "y": -0.50,                   // Velocity Y component
    "z": 0.00                     // Velocity Z component
  },
  "speed": 13.21                  // Speed magnitude (m/s)
}
```

## CSV Format

The CSV format contains only ego vehicle state for simpler analysis:

| Column | Type | Description |
|--------|------|-------------|
| frame | int | Frame index |
| t_sim | float | Simulation time (s) |
| t_world | float | World elapsed time (s) |
| dt | float | Delta time (s) |
| world_x | float | World X position (m) |
| world_y | float | World Y position (m) |
| world_z | float | World Z position (m) |
| vx | float | Longitudinal velocity (m/s) |
| vy | float | Lateral velocity (m/s) |
| vz | float | Vertical velocity (m/s) |
| ax | float | Longitudinal acceleration (m/s^2) |
| ay | float | Lateral acceleration (m/s^2) |
| az | float | Vertical acceleration (m/s^2) |
| roll_rate | float | Roll rate (deg/s) |
| pitch_rate | float | Pitch rate (deg/s) |
| yaw_rate | float | Yaw rate (deg/s) |
| roll | float | Roll angle (deg) |
| pitch | float | Pitch angle (deg) |
| yaw | float | Yaw angle (deg) |
| speed | float | Total speed (m/s) |
| throttle | float | Throttle [0,1] |
| brake | float | Brake [0,1] |
| steer | float | Steering [-1,1] |

## Usage Examples

### Python: Load and Analyze

```python
import json
import pandas as pd

# Load JSON telemetry
with open("telemetry.json") as f:
    data = json.load(f)

# Extract ego speeds
speeds = [frame["ego"]["speed"] for frame in data["frames"]]

# Load CSV for pandas analysis
df = pd.read_csv("telemetry.csv")
print(df.describe())
```

### MATLAB: Load CSV

```matlab
data = readtable('telemetry.csv');
plot(data.t_sim, data.speed);
xlabel('Time (s)');
ylabel('Speed (m/s)');
```

## Notes on Derived Values

### Acceleration Calculation

Acceleration is computed from velocity change:

```
ax = (vx[t] - vx[t-1]) / dt
```

The first frame has zero acceleration since there's no previous velocity.

### Angular Velocity Calculation

Angular velocity is computed from rotation change:

```
yaw_rate = (yaw[t] - yaw[t-1]) / dt
```

Angle wrap-around (e.g., 179 to -179) is handled by normalizing to [-180, 180].
