"""
2D Telemetry Map Visualization Tool

Generates comprehensive 2D trajectory maps showing:
- Ego vehicle trajectory with velocity/acceleration coloring
- Other actors' trajectories (vehicles, pedestrians)
- Event markers with annotations
- Velocity and acceleration time series
- Actor appearance analysis (when actors first appear)

Designed to diagnose:
1. Vehicle oscillation during driving
2. Sudden actor appearance when ego approaches
3. Unrealistic collision or pedestrian visibility issues
"""

import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
import matplotlib.cm as cm


def load_telemetry(telemetry_path: str) -> dict:
    """Load telemetry JSON file."""
    with open(telemetry_path, 'r') as f:
        return json.load(f)


def load_events(events_path: str) -> List[dict]:
    """Load events JSON file."""
    with open(events_path, 'r') as f:
        data = json.load(f)
        return data.get('events', [])


def extract_ego_trajectory(telemetry: dict) -> dict:
    """Extract ego vehicle trajectory data."""
    frames = telemetry['frames']

    data = {
        't': [],
        'x': [],
        'y': [],
        'yaw': [],
        'speed': [],
        'vx': [],
        'vy': [],
        'ax': [],
        'ay': [],
        'throttle': [],
        'brake': [],
        'steer': []
    }

    for frame in frames:
        ego = frame['ego']
        data['t'].append(frame['t_sim'])
        data['x'].append(ego['position']['x'])
        data['y'].append(ego['position']['y'])
        data['yaw'].append(ego['orientation']['yaw'])
        data['speed'].append(ego['speed'])
        data['vx'].append(ego['velocity']['vx'])
        data['vy'].append(ego['velocity']['vy'])
        data['ax'].append(ego['acceleration']['ax'])
        data['ay'].append(ego['acceleration']['ay'])
        data['throttle'].append(ego['control']['throttle'])
        data['brake'].append(ego['control']['brake'])
        data['steer'].append(ego['control']['steer'])

    return {k: np.array(v) for k, v in data.items()}


def extract_actor_trajectories(telemetry: dict) -> Dict[int, dict]:
    """Extract trajectories for all actors."""
    actors_data = {}

    for frame in telemetry['frames']:
        t = frame['t_sim']
        for actor in frame.get('actors', []):
            actor_id = actor['id']
            if actor_id not in actors_data:
                actors_data[actor_id] = {
                    'id': actor_id,
                    'type': actor['type'],
                    'type_id': actor.get('type_id', ''),
                    'role_name': actor.get('role_name', ''),
                    't': [],
                    'x': [],
                    'y': [],
                    'distance': [],
                    'speed': [],
                    'first_seen_t': t,
                    'first_seen_distance': actor.get('distance_to_ego', 0)
                }

            actors_data[actor_id]['t'].append(t)
            actors_data[actor_id]['x'].append(actor['position']['x'])
            actors_data[actor_id]['y'].append(actor['position']['y'])
            actors_data[actor_id]['distance'].append(actor.get('distance_to_ego', 0))
            actors_data[actor_id]['speed'].append(actor.get('speed', 0))

    # Convert to numpy arrays
    for actor_id in actors_data:
        for key in ['t', 'x', 'y', 'distance', 'speed']:
            actors_data[actor_id][key] = np.array(actors_data[actor_id][key])

    return actors_data


def find_event_positions(ego_data: dict, events: List[dict]) -> List[dict]:
    """Map events to ego positions."""
    event_positions = []

    for event in events:
        t_event = event['t_event']
        # Find closest time index
        idx = np.argmin(np.abs(ego_data['t'] - t_event))

        event_positions.append({
            't': t_event,
            'x': ego_data['x'][idx],
            'y': ego_data['y'][idx],
            'type': event['type'],
            'decision_text': event.get('decision_text', ''),
            'reason_text': event.get('reason_text', '')
        })

    return event_positions


def plot_trajectory_2d(
    ego_data: dict,
    actors_data: Dict[int, dict],
    events: List[dict],
    scenario_name: str,
    output_path: str,
    color_by: str = 'speed'
) -> None:
    """
    Plot 2D trajectory map with ego, actors, and events.

    Args:
        ego_data: Ego vehicle trajectory data
        actors_data: Dictionary of actor trajectories
        events: List of events
        scenario_name: Name of scenario
        output_path: Path to save figure
        color_by: 'speed' or 'acceleration' for trajectory coloring
    """
    fig, axes = plt.subplots(2, 2, figsize=(16, 14))
    fig.suptitle(f'Telemetry Analysis: {scenario_name}', fontsize=14, fontweight='bold')

    # =====================
    # Plot 1: 2D Trajectory Map (top-left)
    # =====================
    ax1 = axes[0, 0]

    # Create colored line for ego trajectory
    x, y = ego_data['x'], ego_data['y']

    if color_by == 'speed':
        colors = ego_data['speed']
        cmap = cm.viridis
        label = 'Speed (m/s)'
    else:
        colors = ego_data['ax']  # Longitudinal acceleration
        cmap = cm.coolwarm
        label = 'Accel (m/s²)'

    # Create line segments
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    norm = Normalize(vmin=colors.min(), vmax=colors.max())
    lc = LineCollection(segments, cmap=cmap, norm=norm, linewidth=3, alpha=0.8)
    lc.set_array(colors[:-1])
    ax1.add_collection(lc)

    # Plot start and end markers
    ax1.scatter(x[0], y[0], c='green', s=150, marker='o', label='Start', zorder=10, edgecolors='white')
    ax1.scatter(x[-1], y[-1], c='red', s=150, marker='s', label='End', zorder=10, edgecolors='white')

    # Plot key actors with different colors
    actor_colors = {
        'merge_vehicle': 'orange',
        'cut_in_vehicle': 'orange',
        'lead_slow': 'purple',
        'pedestrian': 'magenta',
        'cross_vehicle': 'brown',
        'ambulance': 'cyan',
        'emergency': 'cyan'
    }

    plotted_roles = set()
    for actor_id, actor in actors_data.items():
        role = actor['role_name']
        type_id = actor['type_id']

        # Determine color and label
        color = 'gray'
        alpha = 0.3
        linewidth = 1

        for key, c in actor_colors.items():
            if key in role.lower() or key in type_id.lower():
                color = c
                alpha = 0.8
                linewidth = 2
                break

        # Check for emergency vehicles
        if 'ambulance' in type_id or 'firetruck' in type_id or 'police' in type_id:
            color = 'cyan'
            alpha = 0.9
            linewidth = 2.5

        # Check for pedestrians
        if actor['type'] == 'pedestrian':
            color = 'magenta'
            alpha = 0.9
            linewidth = 2.5

        # Create label for legend (only once per role)
        actor_label = None
        if role and role not in plotted_roles and color != 'gray':
            actor_label = role
            plotted_roles.add(role)

        ax1.plot(actor['x'], actor['y'], c=color, alpha=alpha, linewidth=linewidth, label=actor_label)

        # Mark where actor first appeared
        if len(actor['x']) > 0:
            ax1.scatter(actor['x'][0], actor['y'][0], c=color, s=50, marker='^', alpha=alpha)

    # Plot events on trajectory
    event_positions = find_event_positions(ego_data, events)
    event_colors = {
        'vehicle_cut_in': 'orange',
        'avoid_pedestrian': 'magenta',
        'yield_to_emergency': 'cyan',
        'stop_for_red_light': 'red',
        'yield_left_turn': 'blue',
        'brake_hard': 'darkred',
        'slow_down': 'yellow',
        'lane_change_left': 'lightblue',
        'lane_change_right': 'lightgreen'
    }

    for ep in event_positions:
        color = event_colors.get(ep['type'], 'black')
        ax1.scatter(ep['x'], ep['y'], c=color, s=200, marker='*',
                   edgecolors='black', linewidth=1, zorder=15)
        ax1.annotate(f"{ep['type']}\n({ep['t']:.1f}s)",
                    (ep['x'], ep['y']),
                    textcoords="offset points", xytext=(10, 10),
                    fontsize=8, color=color,
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('2D Trajectory Map')
    ax1.legend(loc='upper left', fontsize=8)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax1)
    cbar.set_label(label)

    # =====================
    # Plot 2: Speed and Acceleration Time Series (top-right)
    # =====================
    ax2 = axes[0, 1]
    ax2_twin = ax2.twinx()

    t = ego_data['t']

    # Speed
    ax2.plot(t, ego_data['speed'], 'b-', linewidth=2, label='Speed (m/s)')
    ax2.set_ylabel('Speed (m/s)', color='blue')
    ax2.tick_params(axis='y', labelcolor='blue')
    ax2.set_ylim(0, max(ego_data['speed']) * 1.2)

    # Acceleration
    ax2_twin.plot(t, ego_data['ax'], 'r-', linewidth=1.5, alpha=0.7, label='Longitudinal Accel')
    ax2_twin.plot(t, ego_data['ay'], 'orange', linewidth=1.5, alpha=0.7, label='Lateral Accel')
    ax2_twin.set_ylabel('Acceleration (m/s²)', color='red')
    ax2_twin.tick_params(axis='y', labelcolor='red')

    # Mark events
    for ep in event_positions:
        ax2.axvline(x=ep['t'], color=event_colors.get(ep['type'], 'gray'),
                   linestyle='--', alpha=0.7, linewidth=1.5)
        ax2.annotate(ep['type'], (ep['t'], ax2.get_ylim()[1]*0.95),
                    rotation=90, fontsize=7, ha='right')

    ax2.set_xlabel('Time (s)')
    ax2.set_title('Speed and Acceleration over Time')
    ax2.grid(True, alpha=0.3)

    # Combined legend
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)

    # =====================
    # Plot 3: Steering and Control Analysis (bottom-left)
    # =====================
    ax3 = axes[1, 0]

    ax3.plot(t, ego_data['steer'], 'g-', linewidth=1.5, label='Steer')
    ax3.fill_between(t, ego_data['throttle'], alpha=0.3, color='blue', label='Throttle')
    ax3.fill_between(t, -ego_data['brake'], alpha=0.3, color='red', label='Brake')

    # Highlight oscillation regions (high frequency steering changes)
    steer_diff = np.abs(np.diff(ego_data['steer']))
    steer_diff = np.append(steer_diff, 0)

    # Moving average of steering change rate
    window = 10
    if len(steer_diff) >= window:
        steer_var = np.convolve(steer_diff, np.ones(window)/window, mode='same')
        oscillation_threshold = np.percentile(steer_var, 90)
        oscillation_mask = steer_var > oscillation_threshold

        # Shade oscillation regions
        ax3.fill_between(t, -1, 1, where=oscillation_mask,
                        alpha=0.2, color='yellow', label='High Oscillation')

    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Control Value')
    ax3.set_title('Control Inputs (Oscillation Analysis)')
    ax3.legend(loc='upper right', fontsize=8)
    ax3.set_ylim(-1.1, 1.1)
    ax3.grid(True, alpha=0.3)

    # Mark events
    for ep in event_positions:
        ax3.axvline(x=ep['t'], color=event_colors.get(ep['type'], 'gray'),
                   linestyle='--', alpha=0.5)

    # =====================
    # Plot 4: Actor Distance Analysis (bottom-right)
    # =====================
    ax4 = axes[1, 1]

    # Plot distance to key actors over time
    key_roles = ['merge_vehicle', 'cut_in_vehicle', 'lead_slow', 'pedestrian',
                 'cross_vehicle', 'ambulance', 'nearby_vehicle']

    for actor_id, actor in actors_data.items():
        role = actor['role_name']
        type_id = actor['type_id']

        # Check if it's a key actor
        is_key = False
        for key in key_roles:
            if key in role.lower():
                is_key = True
                break

        # Check for emergency vehicles or pedestrians
        if 'ambulance' in type_id or 'firetruck' in type_id or actor['type'] == 'pedestrian':
            is_key = True

        if is_key and len(actor['t']) > 5:
            color = 'gray'
            for key, c in actor_colors.items():
                if key in role.lower() or key in type_id.lower():
                    color = c
                    break

            if actor['type'] == 'pedestrian':
                color = 'magenta'
            if 'ambulance' in type_id or 'firetruck' in type_id:
                color = 'cyan'

            ax4.plot(actor['t'], actor['distance'], c=color, linewidth=2,
                    label=f"{role or actor['type_id'][:20]}", alpha=0.8)

            # Mark first appearance
            ax4.scatter(actor['first_seen_t'], actor['first_seen_distance'],
                       c=color, s=100, marker='v', edgecolors='black', zorder=10)
            ax4.annotate(f"First seen\n{actor['first_seen_distance']:.0f}m",
                        (actor['first_seen_t'], actor['first_seen_distance']),
                        textcoords="offset points", xytext=(5, 5),
                        fontsize=7, color=color)

    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Distance to Ego (m)')
    ax4.set_title('Actor Distance over Time (First Appearance Analysis)')
    ax4.legend(loc='upper right', fontsize=7)
    ax4.grid(True, alpha=0.3)
    ax4.set_ylim(0, None)

    # Mark events
    for ep in event_positions:
        ax4.axvline(x=ep['t'], color=event_colors.get(ep['type'], 'gray'),
                   linestyle='--', alpha=0.5)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"Saved: {output_path}")


def plot_actor_appearance_analysis(
    actors_data: Dict[int, dict],
    scenario_name: str,
    output_path: str
) -> None:
    """
    Analyze when actors first appear and at what distance.
    Helps identify sudden appearance issues.
    """
    fig, ax = plt.subplots(figsize=(12, 8))

    # Collect first appearance data
    appearances = []
    for actor_id, actor in actors_data.items():
        appearances.append({
            'role': actor['role_name'] or actor['type_id'],
            'type': actor['type'],
            'first_t': actor['first_seen_t'],
            'first_dist': actor['first_seen_distance'],
            'type_id': actor['type_id']
        })

    # Sort by first appearance time
    appearances.sort(key=lambda x: x['first_t'])

    # Color by actor type
    colors = []
    for a in appearances:
        if a['type'] == 'pedestrian':
            colors.append('magenta')
        elif 'ambulance' in a['type_id'] or 'firetruck' in a['type_id']:
            colors.append('cyan')
        elif 'merge' in a['role'].lower() or 'cut_in' in a['role'].lower():
            colors.append('orange')
        elif 'lead' in a['role'].lower():
            colors.append('purple')
        else:
            colors.append('gray')

    # Scatter plot
    times = [a['first_t'] for a in appearances]
    distances = [a['first_dist'] for a in appearances]

    scatter = ax.scatter(times, distances, c=colors, s=100, alpha=0.7, edgecolors='black')

    # Annotate key actors
    for i, a in enumerate(appearances):
        if colors[i] != 'gray':  # Only annotate key actors
            ax.annotate(a['role'][:15], (times[i], distances[i]),
                       textcoords="offset points", xytext=(5, 5),
                       fontsize=8, color=colors[i])

    # Add reference lines
    ax.axhline(y=50, color='red', linestyle='--', alpha=0.5, label='50m (typical detection range)')
    ax.axhline(y=100, color='orange', linestyle='--', alpha=0.5, label='100m (actor spawn limit)')

    ax.set_xlabel('Time First Seen (s)')
    ax.set_ylabel('Distance When First Seen (m)')
    ax.set_title(f'Actor Appearance Analysis: {scenario_name}')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"Saved: {output_path}")


def generate_scenario_report(
    telemetry_path: str,
    events_path: str,
    output_dir: str,
    scenario_name: str
) -> dict:
    """
    Generate complete visualization report for a scenario.

    Returns summary statistics for potential issues.
    """
    os.makedirs(output_dir, exist_ok=True)

    # Load data
    telemetry = load_telemetry(telemetry_path)
    events = load_events(events_path)

    # Extract trajectories
    ego_data = extract_ego_trajectory(telemetry)
    actors_data = extract_actor_trajectories(telemetry)

    # Generate main visualization
    plot_trajectory_2d(
        ego_data, actors_data, events, scenario_name,
        os.path.join(output_dir, f'{scenario_name}_trajectory.png'),
        color_by='speed'
    )

    # Generate acceleration-colored version
    plot_trajectory_2d(
        ego_data, actors_data, events, scenario_name,
        os.path.join(output_dir, f'{scenario_name}_trajectory_accel.png'),
        color_by='acceleration'
    )

    # Generate actor appearance analysis
    plot_actor_appearance_analysis(
        actors_data, scenario_name,
        os.path.join(output_dir, f'{scenario_name}_appearance.png')
    )

    # Calculate summary statistics
    summary = {
        'scenario': scenario_name,
        'duration': float(ego_data['t'][-1]),
        'total_frames': len(ego_data['t']),
        'events_count': len(events),
        'actors_count': len(actors_data),

        # Oscillation metrics
        'steer_std': float(np.std(ego_data['steer'])),
        'steer_changes': int(np.sum(np.abs(np.diff(ego_data['steer'])) > 0.01)),
        'lateral_accel_std': float(np.std(ego_data['ay'])),

        # Speed metrics
        'speed_mean': float(np.mean(ego_data['speed'])),
        'speed_std': float(np.std(ego_data['speed'])),
        'speed_max': float(np.max(ego_data['speed'])),

        # Acceleration metrics
        'accel_max': float(np.max(ego_data['ax'])),
        'accel_min': float(np.min(ego_data['ax'])),
        'brake_hard_count': int(np.sum(ego_data['ax'] < -3.5)),

        # Actor appearance
        'key_actors': []
    }

    # Key actor analysis
    for actor_id, actor in actors_data.items():
        role = actor['role_name']
        if role and 'nearby' not in role.lower():
            summary['key_actors'].append({
                'role': role,
                'type': actor['type'],
                'first_seen_t': float(actor['first_seen_t']),
                'first_seen_distance': float(actor['first_seen_distance']),
                'min_distance': float(np.min(actor['distance'])) if len(actor['distance']) > 0 else None
            })

    # Save summary
    summary_path = os.path.join(output_dir, f'{scenario_name}_summary.json')
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)

    print(f"Saved summary: {summary_path}")

    return summary


def analyze_all_scenarios(runs_dir: str, output_dir: str) -> None:
    """
    Analyze all scenarios in a runs directory.
    """
    runs_path = Path(runs_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    all_summaries = []

    # Find all scenario directories
    for scenario_dir in sorted(runs_path.iterdir()):
        if not scenario_dir.is_dir():
            continue

        telemetry_file = scenario_dir / 'telemetry.json'
        events_file = scenario_dir / 'events.json'

        if telemetry_file.exists() and events_file.exists():
            scenario_name = scenario_dir.name
            print(f"\n{'='*60}")
            print(f"Analyzing: {scenario_name}")
            print('='*60)

            try:
                summary = generate_scenario_report(
                    str(telemetry_file),
                    str(events_file),
                    str(output_path / scenario_name),
                    scenario_name
                )
                all_summaries.append(summary)
            except Exception as e:
                print(f"Error analyzing {scenario_name}: {e}")

    # Generate comparison report
    if all_summaries:
        comparison_path = output_path / 'comparison_report.json'
        with open(comparison_path, 'w') as f:
            json.dump(all_summaries, f, indent=2)
        print(f"\nSaved comparison report: {comparison_path}")

        # Print summary table
        print("\n" + "="*80)
        print("SUMMARY COMPARISON")
        print("="*80)
        print(f"{'Scenario':<25} {'Events':>8} {'Steer Std':>10} {'Accel Std':>10} {'Hard Brake':>10}")
        print("-"*80)
        for s in all_summaries:
            print(f"{s['scenario']:<25} {s['events_count']:>8} {s['steer_std']:>10.4f} {s['lateral_accel_std']:>10.4f} {s['brake_hard_count']:>10}")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Generate 2D telemetry visualizations')
    parser.add_argument('--runs', type=str, required=True, help='Path to runs directory')
    parser.add_argument('--output', type=str, default='./telemetry_analysis', help='Output directory')
    parser.add_argument('--scenario', type=str, default=None, help='Analyze specific scenario only')

    args = parser.parse_args()

    if args.scenario:
        # Analyze single scenario
        runs_path = Path(args.runs)
        scenario_dir = runs_path / args.scenario

        if not scenario_dir.exists():
            print(f"Scenario directory not found: {scenario_dir}")
            exit(1)

        generate_scenario_report(
            str(scenario_dir / 'telemetry.json'),
            str(scenario_dir / 'events.json'),
            args.output,
            args.scenario
        )
    else:
        # Analyze all scenarios
        analyze_all_scenarios(args.runs, args.output)
