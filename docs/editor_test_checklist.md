# Interactive Editor Manual Test Checklist

## Prerequisites

Before testing, ensure:
1. Python environment is activated (`conda activate CARLA312`)
2. Map data exists: `ls data/maps/Town05/` should show geojson files
3. No CARLA server connection required for editor testing

## Test Execution

### Test 1: Launch and Basic Display

**Steps:**
```bash
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town05 \
    --episode-id manual_test \
    --duration 30
```

**Expected Results:**
- [ ] Window opens with title "Figure 1" (matplotlib default)
- [ ] Map shows blue lane centerlines
- [ ] Yellow junction areas visible
- [ ] Actor selection panel shows "ego" selected
- [ ] Buttons visible on right panel (Add KF, Move KF, Del KF, Snap, etc.)
- [ ] Text input fields visible (Event, Action, KF t, KF v, Actor)
- [ ] Time slider at bottom shows 0.0 to 30.0
- [ ] Status bar shows "Mode: move | Snap: OFF"

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 2: Button Functionality

**Steps:**
1. Click each button and observe response

**Expected Results:**
- [ ] **Add KF**: Status changes to "Mode: add"
- [ ] **Move KF**: Status changes to "Mode: move"
- [ ] **Del KF**: Status changes to "Mode: delete"
- [ ] **Snap**: Status toggles "Snap: ON" / "Snap: OFF"
- [ ] **Analyze**: Console shows "Analysis: conflicts=X, off-lane=X..."
- [ ] **Save**: Console shows "Scene saved: ..."
- [ ] **AddEvt**: Event marker appears at current time
- [ ] **DelEvt**: Event near current time is deleted
- [ ] **Undo**: Reverts last action
- [ ] **Redo**: Restores undone action
- [ ] **Export**: Console shows "Exported plan/events to ..."
- [ ] **Reset**: Map view resets to show all actors

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 3: Keyframe Operations

**Steps:**
1. Click **Add KF** button
2. Click on the map to add a keyframe
3. Move time slider to a new position
4. Click on map to add another keyframe
5. Click **Move KF** button
6. Click on a keyframe and drag it
7. Click **Del KF** button
8. Click on a keyframe to delete it

**Expected Results:**
- [ ] Add mode: Click adds keyframe, trajectory line updates
- [ ] Keyframe appears as colored dot
- [ ] Time slider position determines keyframe time
- [ ] Move mode: Keyframe follows mouse drag
- [ ] Delete mode: Click removes keyframe
- [ ] Trajectory line updates after each operation

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 4: Snap to Centerline

**Steps:**
1. Click **Snap** to enable (status shows "Snap: ON")
2. Click **Add KF**
3. Click slightly off a lane centerline
4. Observe keyframe position

**Expected Results:**
- [ ] Keyframe snaps to nearest lane centerline point
- [ ] Console shows "Snapped keyframe to centerline (offset: X.XXm)"
- [ ] Dragging keyframes also snaps when enabled

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 5: Keyboard Shortcuts

**Steps:**
1. Press each key and observe response

**Expected Results:**
- [ ] `a` → Mode changes to "add"
- [ ] `m` → Mode changes to "move"
- [ ] `d` → Mode changes to "delete"
- [ ] `s` → Snap toggles
- [ ] `Space` → Analysis runs
- [ ] `r` → View resets
- [ ] `h` → Help text prints to console
- [ ] `Ctrl+Z` → Undo (if actions exist)
- [ ] `Ctrl+Y` → Redo (if undone actions exist)
- [ ] `Ctrl+S` → Save scene
- [ ] `Ctrl+E` → Export plan
- [ ] `Delete` or `Backspace` → Delete selected keyframe

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 6: Zoom and Pan

**Steps:**
1. Scroll mouse wheel up/down over the map
2. After zooming, press `r` to reset view

**Expected Results:**
- [ ] Scroll up → Zoom in (view shrinks)
- [ ] Scroll down → Zoom out (view expands)
- [ ] Zoom centers on cursor position
- [ ] `r` key resets view to show all actors

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 7: Add/Delete Actors

**Steps:**
1. Type `npc1,vehicle,npc` in Actor field
2. Click **Add Actor**
3. Select new actor in Actor Selection panel
4. Add keyframes for the new actor
5. Click **Del Actor** to delete it

**Expected Results:**
- [ ] New actor appears in Actor Selection panel
- [ ] New actor has different color trajectory
- [ ] Can add keyframes to new actor
- [ ] Delete removes actor (ego cannot be deleted)

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 8: Event Management

**Steps:**
1. Move time slider to t=10
2. Type "lane_change" in Event field
3. Type "lane_change" in Action field
4. Click **AddEvt**
5. Move time slider to t=10
6. Click **DelEvt** or press `e`

**Expected Results:**
- [ ] Event marker appears on ego trajectory at t=10
- [ ] Console shows "Added event 'lane_change' at t=10.00s"
- [ ] Delete removes the event
- [ ] Console shows "Deleted event..."

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 9: Analysis Visualization

**Steps:**
1. Create a scenario with two actors close together:
   - Ego: keyframes at (0,0), (100,0)
   - NPC: keyframes at (10,0), (90,0)
2. Click **Analyze**
3. Observe map for markers

**Expected Results:**
- [ ] Red dots appear where actors are within 3m
- [ ] Yellow dots appear for off-lane positions
- [ ] Purple triangles for kinematic violations (if any)
- [ ] Pink X for TTC warnings (if approaching)
- [ ] Console shows analysis summary

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 10: Undo/Redo

**Steps:**
1. Add a keyframe
2. Press `Ctrl+Z` to undo
3. Press `Ctrl+Y` to redo
4. Add multiple keyframes
5. Undo multiple times
6. Redo multiple times

**Expected Results:**
- [ ] Undo removes the last added keyframe
- [ ] Redo restores the keyframe
- [ ] Multiple undos work correctly
- [ ] Redo stack clears when new action is performed

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 11: Save and Load

**Steps:**
1. Create a scenario with multiple actors and events
2. Click **Save**
3. Close the editor
4. Reopen with the saved scene:
   ```bash
   python -m carla_experiment_client.cli editor \
       --map-dir data/maps/Town05 \
       --scene outputs/manual_test/scene_edit.json \
       --episode-id manual_test
   ```

**Expected Results:**
- [ ] Scene saves to `outputs/manual_test/scene_edit.json`
- [ ] File contains actors, keyframes, events
- [ ] Loading restores all actors and keyframes
- [ ] Events are preserved

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 12: Export Plan

**Steps:**
1. Create a complete scenario
2. Click **Export**
3. Verify output files

**Expected Results:**
- [ ] `outputs/manual_test/plan.json` created
- [ ] `outputs/manual_test/events_plan.json` created
- [ ] `outputs/manual_test/scene_edit.json` created
- [ ] plan.json contains full trajectory with dt=0.05s points
- [ ] events_plan.json contains event list

**Verify with:**
```bash
ls outputs/manual_test/
cat outputs/manual_test/plan.json | head -50
```

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 13: Headless Mode

**Steps:**
```bash
python -m carla_experiment_client.cli editor \
    --map-dir data/maps/Town05 \
    --episode-id headless_test \
    --duration 10 \
    --headless
```

**Expected Results:**
- [ ] No GUI window opens
- [ ] Console shows "Exported plan/events to outputs/headless_test"
- [ ] Output files are created

**Test Result:** PASS / FAIL
**Notes:**

---

### Test 14: Automated Test Suite

**Steps:**
```bash
python -m carla_experiment_client.editor.test_editor
```

**Expected Results:**
- [ ] All 8 tests pass
- [ ] Output shows "Results: 8/8 tests passed"

**Test Result:** PASS / FAIL
**Notes:**

---

## Summary

| Test # | Test Name | Result |
|--------|-----------|--------|
| 1 | Launch and Basic Display | |
| 2 | Button Functionality | |
| 3 | Keyframe Operations | |
| 4 | Snap to Centerline | |
| 5 | Keyboard Shortcuts | |
| 6 | Zoom and Pan | |
| 7 | Add/Delete Actors | |
| 8 | Event Management | |
| 9 | Analysis Visualization | |
| 10 | Undo/Redo | |
| 11 | Save and Load | |
| 12 | Export Plan | |
| 13 | Headless Mode | |
| 14 | Automated Test Suite | |

**Total:** ___/14 Passed

**Tester:** _______________
**Date:** _______________
**Notes:**
