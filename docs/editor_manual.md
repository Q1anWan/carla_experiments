# Interactive Scene Editor User Manual

## Overview

The Interactive Scene Editor is a Matplotlib-based 2D GUI tool for creating and editing multi-actor driving scenarios. It allows you to:

- Define actor trajectories using keyframes
- Add/edit driving events
- Snap keyframes to lane centerlines
- Analyze trajectory feasibility (kinematic constraints, collisions, TTC)
- Export to `plan.json` for rendering in CARLA

## Quick Start

### Prerequisites

1. Export map assets (required once per map):
```bash
python -m carla_experiment_client.cli map --map Town05 --out data/maps
```

2. Verify map files exist:
```bash
ls data/maps/Town05/
# Should contain: lane_centerlines.geojson, junction_areas.geojson, map_graph.json
```

### Launch Editor

```bash
# GUI mode
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town05 \
    --episode-id my_scenario \
    --duration 60

# Headless mode (export only)
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town05 \
    --episode-id my_scenario \
    --headless
```

## Window Layout

```
+--------------------------------------------------+-------------------+
|                                                  | [Actor Selection] |
|                                                  |                   |
|                                                  | [Add KF] [AddEvt] |
|                                                  | [Move KF][DelEvt] |
|                                                  | [Del KF] [Undo]   |
|              Map View                            | [Snap]   [Redo]   |
|              (main editing area)                 | [Analyze][Export] |
|                                                  | [Save]   [Reset]  |
|                                                  |                   |
|                                                  | Event: [______]   |
|                                                  | Action: [______]  |
|                                                  | KF t: [______]    |
|                                                  | KF v: [______]    |
|                                                  | Actor: [______]   |
|                                                  | [Add Actor][Del]  |
+--------------------------------------------------+-------------------+
|  [====== Time Slider (t) ======]                                     |
+----------------------------------------------------------------------+
| Mode: move | Snap: OFF | Keys: a/m/d=mode s=snap space=analyze h=help|
+----------------------------------------------------------------------+
```

## Controls

### Mouse Controls

| Action | Description |
|--------|-------------|
| Left click (Add mode) | Add new keyframe at cursor position |
| Left click (Move mode) | Select nearest keyframe |
| Left click (Delete mode) | Delete nearest keyframe |
| Drag (Move mode) | Move selected keyframe |
| Scroll wheel | Zoom in/out on map |

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `a` | Switch to Add keyframe mode |
| `m` | Switch to Move keyframe mode |
| `d` | Switch to Delete keyframe mode |
| `s` | Toggle snap to lane centerline |
| `e` | Delete nearest event at current time |
| `Space` | Run analysis (conflicts, kinematic, TTC) |
| `r` | Reset view to fit all actors |
| `h` | Print help to console |
| `Delete` / `Backspace` | Delete selected keyframe |
| `Ctrl+Z` | Undo |
| `Ctrl+Y` | Redo |
| `t` | Print keyframe table to console |
| `Ctrl+S` | Save scene |
| `Ctrl+E` | Export plan.json |

### Buttons

| Button | Description |
|--------|-------------|
| **Add KF** | Switch to Add keyframe mode |
| **Move KF** | Switch to Move keyframe mode |
| **Del KF** | Switch to Delete keyframe mode |
| **Snap** | Toggle snap to lane centerlines |
| **Analyze** | Run feasibility analysis |
| **Save** | Save `scene_edit.json` |
| **AddEvt** | Add event at current time slider position |
| **DelEvt** | Delete event nearest to current time |
| **Undo** | Undo last action |
| **Redo** | Redo last undone action |
| **Export** | Export `plan.json` and `events_plan.json` |
| **Reset** | Reset map view to fit all actors |
| **Add Actor** | Add new actor (format: `id,kind,role`) |
| **Del Actor** | Delete selected actor (except ego) |

## Workflows

### Creating a New Scenario

1. Launch editor with desired episode ID:
   ```bash
   python -m carla_experiment_client.cli editor \
       --map-dir data/maps/Town05 \
       --episode-id my_new_scenario \
       --duration 60
   ```

2. The editor starts with a default ego vehicle. Click on the map to see lane centerlines.

3. **Edit ego trajectory:**
   - Press `a` or click **Add KF** to enter Add mode
   - Click on the map to add keyframes along the desired path
   - Use the time slider to set the time for each keyframe
   - Press `s` to enable snap-to-centerline for precise lane following

4. **Add NPC actors:**
   - Enter actor info in the "Actor" text box: `npc1,vehicle,npc`
   - Click **Add Actor**
   - The new actor appears in the Actor Selection panel
   - Add keyframes for the NPC

5. **Add events:**
   - Move the time slider to the event time
   - Enter event type in "Event" box (e.g., `lane_change`)
   - Enter expected action in "Action" box
   - Click **AddEvt**

6. **Analyze:**
   - Click **Analyze** or press `Space` to check feasibility
   - Review markers:
     - Red dots: Actor conflicts (too close)
     - Yellow dots: Off-lane positions
     - Purple triangles: Kinematic violations
     - Pink X: TTC warnings

7. **Save and Export:**
   - Click **Save** to save `scene_edit.json` (editable format)
   - Click **Export** to generate `plan.json` (render format)

### Loading a Pre-Built Scene Design

Pre-converted scene designs from validated telemetry are available in `outputs/scene_designs/`:

```bash
# Load lane_change_cut_in scene design
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town05 \
    --scene outputs/scene_designs/lane_change_cut_in/scene_edit.json \
    --episode-id lane_change_cut_in

# Load highway_merge scene design (Town04)
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town04 \
    --scene outputs/scene_designs/highway_merge/scene_edit.json \
    --episode-id highway_merge
```

The editor automatically resets the view to fit all actors when loading a scene, so all trajectories are visible immediately.

### Editing an Existing Scene

```bash
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town05 \
    --scene outputs/my_scenario/scene_edit.json \
    --episode-id my_scenario
```

### Snap to Lane Centerline

1. Press `s` or click **Snap** to enable snapping
2. Status bar shows "Snap: ON"
3. When adding or moving keyframes, they will automatically snap to the nearest lane centerline point
4. Useful for creating trajectories that follow road geometry

### Using Undo/Redo

The editor maintains an undo stack (up to 50 states):
- `Ctrl+Z` or **Undo** button to undo
- `Ctrl+Y` or **Redo** button to redo
- The following actions are undoable:
  - Adding/moving/deleting keyframes
  - Adding/deleting actors
  - Adding/deleting events

## Analysis Features

### Conflict Detection
- Detects when two actors are within 3m of each other
- Shown as red dots on the map

### Off-Lane Detection
- Detects positions too far from lane centerlines
- Vehicle threshold: 1.8m
- Walker threshold: 2.8m
- Shown as yellow dots

### Kinematic Constraint Checking
- Speed limit: 30 m/s (108 km/h)
- Acceleration: 8 m/s²
- Deceleration: 10 m/s²
- Lateral acceleration: 5 m/s²
- Shown as purple triangles

### Time-to-Collision (TTC)
- Warns when TTC < 3 seconds between approaching actors
- Shown as pink X markers

## Output Files

After exporting, the following files are created in `outputs/<episode_id>/`:

| File | Description |
|------|-------------|
| `scene_edit.json` | Editable scene format (keyframes, events) |
| `plan.json` | Full trajectory plan for renderer |
| `events_plan.json` | Event list with timing |

## Text Input Fields

| Field | Format | Example |
|-------|--------|---------|
| Event | Event type string | `lane_change`, `pedestrian_crossing` |
| Action | Expected action string | `lane_change`, `brake`, `yield` |
| KF t | Time in seconds | `15.5` |
| KF v | Speed in m/s (optional) | `10.0` |
| Actor | `id,kind,role` | `npc1,vehicle,npc` or `ped1,walker,vru` |

## Keyframe Table (Debug)

Press `t` to print a formatted table of all keyframes for the selected actor to the console. This is useful for verifying keyframe timing, positions, and speeds without leaving the editor.

Example output:
```
Actor: ego (4 keyframes)
  #  t(s)      x         y        v(m/s)
  0  0.00    15.000   200.000    10.0
  1 10.00    52.240   187.827    12.0
  2 20.00    82.240   187.978    10.0
  3 30.00   156.888   167.577     8.0
```

## Tips and Best Practices

1. **Start with ego trajectory**: Define the ego vehicle's path first, then add NPCs.

2. **Use snap mode**: Enable snap (`s`) for accurate lane-following trajectories.

3. **Check with Analyze**: Always run analysis before exporting to catch issues early.

4. **Set keyframe velocities**: Specify speed at keyframes using the "KF v" field for smoother trajectories.

5. **Scrub the timeline**: Use the time slider to preview actor positions at different times.

6. **Save frequently**: Use `Ctrl+S` to save your work periodically.

7. **Use keyboard shortcuts**: They're faster than clicking buttons.

## Troubleshooting

### Buttons Not Responding
- Ensure the editor window has focus (click on the window)
- Try using keyboard shortcuts instead
- Check console for error messages

### Snap Not Working
- Verify map data exists in the specified map-dir
- Check that `map_graph.json` contains lane centerline data

### Analysis Shows Many Warnings
- Review keyframe positions and adjust
- Reduce speed at sharp turns to fix lateral acceleration warnings
- Increase spacing between actors to fix conflict warnings

### Export Fails
- Check write permissions for the output directory
- Ensure all required fields are filled

## Command Reference

```bash
# Full editor options
python -m carla_experiment_client.cli editor --help

# Common usage patterns
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town05 \
    --episode-id test \
    --duration 30 \
    --dt 0.05

# Headless export
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town05 \
    --episode-id test \
    --headless

# Load existing scene
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town05 \
    --scene outputs/test/scene_edit.json \
    --episode-id test
```
