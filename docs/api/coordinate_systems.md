# Coordinate Systems

This document explains the coordinate systems used in the CARLA Experiment Client.

## Overview

The system uses two coordinate frames:
1. **CARLA World Frame** - Global coordinate system for actor positions
2. **SAE J670 Vehicle Frame** - Vehicle-relative coordinate system for motion data

## CARLA World Frame

CARLA uses a left-handed coordinate system (Unreal Engine convention):

```
        +Z (Up)
         |
         |    +X (Forward)
         |   /
         |  /
         | /
         |/_________ +Y (Right)
        O (World Origin)
```

- **X-axis**: Points east (typically)
- **Y-axis**: Points south (typically, right-handed from above)
- **Z-axis**: Points up
- **Units**: Meters
- **Origin**: Map-dependent, typically at map center

### Rotation Conventions

- **Yaw**: Rotation around Z-axis (heading)
  - 0 degrees: Facing +X direction
  - 90 degrees: Facing +Y direction
  - Range: [-180, 180] degrees
- **Pitch**: Rotation around Y-axis
  - Positive: Nose up
- **Roll**: Rotation around X-axis
  - Positive: Right side down

## SAE J670 Vehicle Frame

SAE J670 defines a vehicle-fixed coordinate system used in automotive engineering:

```
        +Z (Up)
         |
         |    +X (Forward)
         |   /
         |  /
         | /
         |/_________ +Y (Left)
        O (Vehicle CG)
```

- **X-axis**: Points forward (front of vehicle is positive)
- **Y-axis**: Points left (left side of vehicle is positive)
- **Z-axis**: Points up (top of vehicle is positive)
- **Origin**: Vehicle center of gravity (approximated at geometric center)

### Key Difference from CARLA

The main difference is the **Y-axis direction**:
- CARLA: Y points **right**
- SAE J670: Y points **left**

Conversion: `SAE_Y = -CARLA_Y`

## Velocity in SAE J670

Velocity is decomposed into vehicle-relative components:

| Component | Symbol | Direction | Positive Value |
|-----------|--------|-----------|----------------|
| Longitudinal | vx | Along X-axis | Moving forward |
| Lateral | vy | Along Y-axis | Moving left |
| Vertical | vz | Along Z-axis | Moving up |

### Calculation

```python
# Get vehicle axes
forward = transform.get_forward_vector()
right = transform.get_right_vector()

# Project velocity onto vehicle axes
vx = dot(velocity, forward)    # Longitudinal
vy = -dot(velocity, right)     # Lateral (SAE convention: left is positive)
vz = velocity.z                # Vertical
```

## Acceleration in SAE J670

Acceleration follows the same convention:

| Component | Symbol | Typical Range | Interpretation |
|-----------|--------|---------------|----------------|
| ax | Longitudinal | -8 to +4 m/s^2 | Braking/Acceleration |
| ay | Lateral | -4 to +4 m/s^2 | Cornering forces |
| az | Vertical | ~0 m/s^2 | Road bumps |

### Comfort Thresholds

Typical comfort limits for human factors:
- **Longitudinal**: -3 to +2 m/s^2 (comfortable), < -6 m/s^2 (emergency)
- **Lateral**: -2 to +2 m/s^2 (comfortable)

## Angular Velocity

Angular velocity describes rotation rates:

| Component | Symbol | Positive Direction |
|-----------|--------|--------------------|
| Roll rate | p | Right side up |
| Pitch rate | q | Nose up |
| Yaw rate | r | Counter-clockwise (from above) |

### Units

- Degrees per second (deg/s) in output data
- Can convert to rad/s by multiplying by π/180

## Transform Helpers

The codebase provides helper functions for coordinate transforms:

### offset_transform

```python
def offset_transform(transform, forward=0.0, right=0.0, up=0.0):
    """Create new transform offset from original in vehicle frame."""
```

Example: Spawn an actor 10m ahead and 3m to the right of ego:
```python
new_transform = offset_transform(ego.get_transform(), forward=10.0, right=3.0)
```

### right_vector

```python
def right_vector(transform):
    """Get the right-pointing vector from a transform."""
```

## Practical Examples

### Distance to Actor

```python
ego_loc = ego.get_location()
actor_loc = actor.get_location()
distance = ego_loc.distance(actor_loc)  # 3D Euclidean distance
```

### Is Actor Ahead?

```python
to_actor = actor.get_location() - ego.get_location()
forward = ego.get_transform().get_forward_vector()
dot_product = to_actor.x * forward.x + to_actor.y * forward.y
is_ahead = dot_product > 0
```

### Relative Bearing

```python
import math

to_actor = actor.get_location() - ego.get_location()
forward = ego.get_transform().get_forward_vector()
right = ego.get_transform().get_right_vector()

# Project onto vehicle axes
x_rel = to_actor.x * forward.x + to_actor.y * forward.y  # Forward distance
y_rel = to_actor.x * right.x + to_actor.y * right.y      # Right distance (CARLA)
y_sae = -y_rel  # SAE: left is positive

# Bearing angle (0 = ahead, 90 = left in SAE)
bearing = math.degrees(math.atan2(y_sae, x_rel))
```

## References

- SAE J670 - Motor Vehicle Dimensions
- SAE J670 - Vehicle Dynamics Terminology
- CARLA Documentation - Coordinate Systems
