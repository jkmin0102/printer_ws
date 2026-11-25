#!/usr/bin/env python3
"""Aggregate real run artifacts and plot successful, executed TF trajectories.

No simulator or ROS nodes run here. The main process uses only the standard
library; plotting uses installed system matplotlib in a fresh process with
user-site packages disabled to avoid the host's NumPy/matplotlib ABI conflict.
"""
import argparse
import csv
import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import sys


PREFERRED_FIELDS = [
    'run_id', 'record_type', 'scenario', 'stage', 'choice', 'success',
    'execution_completed', 'failure_reason', 'L_clear', 'L_detour',
    'v_eff', 'mock_remove_seconds', 'switch_seconds', 'decision_margin',
    'T_remove', 'T_detour', 'selected_execution_sim_seconds',
    'selected_execution_wall_seconds', 'total_sim_seconds', 'total_wall_seconds',
    'stop_pose', 'observed_center', 'final_pose', 'executed_route_max_y',
    'source_file',
]


def read_json(path):
    with path.open(encoding='utf-8') as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError('Expected JSON object')
    return data


def relative(path, root):
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path.resolve())


def trajectory(path):
    """Read finite, recorded map TF positions; never interpolate missing data."""
    rows = []
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            try:
                x, y = float(row['x']), float(row['y'])
            except (KeyError, ValueError, TypeError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                rows.append({**row, 'x': x, 'y': y})
    return rows


def fixture_position(run_dir):
    """Fixture location is plot annotation only, from recorded launch arguments."""
    path = run_dir / 'supervisor.json'
    if not path.exists():
        return None
    try:
        args = read_json(path).get('args', {})
        if args.get('no_obstacle') or args.get('baseline'):
            return None
        x, y = float(args['obstacle_x']), float(args['obstacle_y'])
        return [x, y] if math.isfinite(x) and math.isfinite(y) else None
    except (KeyError, ValueError, TypeError):
        return None


def selected_run(record, root):
    result_path = root / record['source_file']
    trace_path = result_path.parent / 'trajectory.csv'
    if not trace_path.exists():
        return None
    rows = trajectory(trace_path)
    if not rows:
        return None
    states = {'DETOUR', 'GOAL_AFTER_MOCK_REMOVE'}
    executed = [row for row in rows if row.get('state') in states
                and row.get('drive_enabled', '').lower() == 'true']
    if not executed:
        return None
    keys = ('run_id', 'scenario', 'choice', 'L_clear', 'L_detour', 'T_remove',
            'T_detour', 'selected_execution_sim_seconds', 'selected_execution_wall_seconds',
            'stop_pose', 'observed_center', 'final_pose', 'executed_route_max_y',
            'v_eff', 'mock_remove_seconds', 'switch_seconds', 'decision_margin')
    selected = {key: record.get(key) for key in keys}
    selected.update(
        run_directory=relative(result_path.parent, root),
        result_file=relative(result_path, root),
        trajectory_file=relative(trace_path, root),
        trajectory_sample_count=len(rows),
        trajectory_max_y=max(row['y'] for row in rows),
        selected_execution_trajectory_max_y=max(row['y'] for row in executed),
        fixture_initial_xy_for_annotation=fixture_position(result_path.parent),
    )
    return selected


def csv_value(value):
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return value


def aggregate(root, runs, output):
    output.mkdir(parents=True, exist_ok=True)
    records, warnings = [], []
    for path in sorted(runs.glob('*/*.json')):
        if path.name not in ('result.json', 'baseline.json'):
            continue
        try:
            record = read_json(path)
            record['source_file'] = relative(path, root)
            record.setdefault('run_id', path.parent.name)
            record['record_type'] = 'baseline' if path.name == 'baseline.json' else 'mission'
            if path.name == 'baseline.json':
                record.update(stage='baseline', choice='BASELINE',
                              execution_completed=bool(record.get('success')),
                              failure_reason=record.get('reason', ''))
                supervisor = path.parent / 'supervisor.json'
                if supervisor.exists():
                    record['scenario'] = read_json(supervisor).get('args', {}).get('scenario', '')
                record['total_wall_seconds'] = record.get('wall_seconds')
            records.append(record)
        except (ValueError, OSError) as exc:
            warnings.append({'source_file': relative(path, root), 'error': str(exc)})
    records.sort(key=lambda row: (row.get('run_id', ''), row['record_type']))
    all_fields = set().union(*(set(row) for row in records)) if records else set()
    fields = PREFERRED_FIELDS + sorted(all_fields - set(PREFERRED_FIELDS))
    with (output / 'results.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({key: csv_value(value) for key, value in record.items()})

    selected = {}
    for scenario, choice in (('A', 'REMOVE'), ('B', 'DETOUR')):
        selected[scenario] = None
        for record in reversed(records):
            if (record.get('record_type') == 'mission' and record.get('scenario') == scenario
                    and record.get('stage') == 'full' and record.get('success') is True
                    and record.get('execution_completed') is True and record.get('choice') == choice):
                candidate = selected_run(record, root)
                if candidate:
                    selected[scenario] = candidate
                    break
    summary = {
        'generated_at_kst': datetime.datetime.now(
            datetime.timezone(datetime.timedelta(hours=9))).isoformat(timespec='seconds'),
        'record_count': len(records),
        'baseline_record_count': sum(row['record_type'] == 'baseline' for row in records),
        'successful_record_count': sum(row.get('success') is True for row in records),
        'failed_record_count': sum(row.get('success') is False for row in records),
        'results_csv': relative(output / 'results.csv', root),
        'comparison_complete': all(selected.values()),
        'selection_rule': 'Latest successful full execution with matching action and recorded driven TF samples',
        'time_model_note': 'Configured assumptions; actual time exists only for the executed candidate',
        'trajectory_note': 'Recorded map-to-base_footprint TF; fixture coordinates annotate the plot only',
        'selected_runs': selected,
        'warnings': warnings,
    }
    target = output / 'validation_summary.json'
    target.write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    return target, summary


def fmt(value):
    return 'unrecorded' if value is None else f'{float(value):.2f}'


def render(root, summary_path, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch, Rectangle

    summary = read_json(summary_path)
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 8.2), sharex=True, sharey=True)
    colors = {'A': '#1466b8', 'B': '#d97409'}
    for ax, scenario in zip(axes, ('A', 'B')):
        geometry = read_json(root / 'namo_sim' / 'maps' / f'{scenario}_geometry.json')
        for xmin, ymin, xmax, ymax in geometry['wall_rectangles']:
            ax.add_patch(Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                                   facecolor='#d8dce1', edgecolor='#6b7280', linewidth=.6, zorder=1))
        start, goal = geometry['start'], geometry['goal']
        ax.scatter([start[0]], [start[1]], marker='^', s=65, color='#202b38', zorder=6)
        ax.annotate('S', start[:2], xytext=(-15, -3), textcoords='offset points')
        ax.scatter([goal[0]], [goal[1]], marker='*', s=145, color='#743bb8', zorder=7)
        ax.annotate('G', goal[:2], xytext=(7, -3), textcoords='offset points')
        item = summary['selected_runs'][scenario]
        if item is None:
            ax.set_title(f'Scenario {scenario}: no verified completed run')
            ax.text(.5, .5, 'No executed trajectory available', ha='center', va='center',
                    transform=ax.transAxes, bbox={'facecolor': 'white', 'alpha': .9})
        else:
            trace = trajectory(root / item['trajectory_file'])
            ax.plot([row['x'] for row in trace], [row['y'] for row in trace],
                    color=colors[scenario], linewidth=2.2, zorder=4)
            fixture = item.get('fixture_initial_xy_for_annotation')
            if fixture:
                # Fixture dimensions from the generated experiment model, not an estimate.
                ax.add_patch(Rectangle((fixture[0]-.225, fixture[1]-.5), .45, 1.0,
                                       fill=False, linestyle='--', edgecolor='#c53138', linewidth=1.4, zorder=3))
            stop = item.get('stop_pose')
            if stop:
                ax.scatter([stop[0]], [stop[1]], s=65, facecolor='#00845a',
                           edgecolor='white', linewidth=.9, zorder=8)
                ax.annotate('decision stop', stop[:2], xytext=(14, -16),
                            textcoords='offset points', fontsize=8,
                            arrowprops={'arrowstyle': '-', 'color': '#00845a'})
            ax.set_title(f"Scenario {scenario}: {item['choice']}\n"
                         f"T_remove = {fmt(item['T_remove'])} s; T_detour = {fmt(item['T_detour'])} s",
                         fontsize=10, pad=12)
            ax.text(.02, .98,
                    f"Executed time: {fmt(item['selected_execution_sim_seconds'])} sim s\n"
                    f"Route max y: {fmt(item['selected_execution_trajectory_max_y'])} m",
                    transform=ax.transAxes, va='top', fontsize=8,
                    bbox={'facecolor': 'white', 'edgecolor': '#bbbbbb', 'alpha': .92})
        ax.set_xlim(-2.8, 7.5)
        ax.set_ylim(-3.4, 6.5)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('map x [m]')
        ax.grid(True, alpha=.18)
    axes[0].set_ylabel('map y [m]')
    legend = [
        Line2D([0], [0], color=colors['A'], lw=2.2, label='A recorded TF trajectory'),
        Line2D([0], [0], color=colors['B'], lw=2.2, label='B recorded TF trajectory'),
        Patch(facecolor='#d8dce1', edgecolor='#6b7280', label='Static physical walls'),
        Patch(facecolor='none', edgecolor='#c53138', linestyle='--', label='Fixture initial footprint (annotation)'),
        Line2D([0], [0], marker='o', color='none', markerfacecolor='#00845a', label='Measured decision stop'),
        Line2D([0], [0], marker='*', color='none', markerfacecolor='#743bb8', markersize=10, label='Goal G'),
    ]
    fig.suptitle('NAMO: recorded executed routes', fontsize=15, y=.97)
    fig.legend(handles=legend, loc='lower center', ncol=3, frameon=False, fontsize=8,
               bbox_to_anchor=(.5, .075))
    fig.text(.5, .035,
             'Only executed map TF samples are plotted. Unexecuted candidate travel times are estimates.\n'
             'Identical axis scales; latest successful full A/B runs selected from saved evidence.',
             ha='center', fontsize=8, color='#374151')
    fig.subplots_adjust(left=.07, right=.98, bottom=.19, top=.84, wspace=.12)
    for extension in ('png', 'pdf'):
        fig.savefig(output / f'route_comparison.{extension}', dpi=180,
                    facecolor='white', bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--runs', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--render-only', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    root = args.root.resolve()
    output = (args.output or root / '.namo').resolve()
    if args.render_only:
        render(root, args.render_only.resolve(), output)
        return
    runs = (args.runs or root / '.namo' / 'runs').resolve()
    summary_path, summary = aggregate(root, runs, output)
    env = dict(os.environ, PYTHONNOUSERSITE='1', MPLCONFIGDIR=str(output / '.matplotlib'))
    subprocess.run([sys.executable, '-s', str(Path(__file__).resolve()), '--root', str(root),
                    '--output', str(output), '--render-only', str(summary_path)],
                   env=env, check=True)
    print(json.dumps({'records': summary['record_count'], 'comparison_complete': summary['comparison_complete'],
                      'selected_runs': {k: v['run_id'] if v else None for k, v in summary['selected_runs'].items()},
                      'csv': str(output / 'results.csv'), 'summary': str(summary_path),
                      'figure': str(output / 'route_comparison.png')}, indent=2))


if __name__ == '__main__':
    main()
