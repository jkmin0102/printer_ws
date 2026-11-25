#!/usr/bin/env python3
"""Post-run evidence only: simulator truth never enters motion or perception."""
import argparse
import json
import math
from pathlib import Path
import statistics
import numpy as np
from scipy.spatial.transform import Rotation


def validate(folder):
    result = json.loads((folder/'result.json').read_text())
    samples = [json.loads(line) for line in (folder/'navigation_probe.jsonl').read_text().splitlines()]
    can = [d['validation_only_can_world_pose'] for d in samples if 'validation_only_can_world_pose' in d]
    robot = [d['validation_only_world_pose'] for d in samples if 'validation_only_world_pose' in d]
    report = dict(run_id=result['run_id'], choice=result['choice'],
                  mission_success=result['success'], execution_completed=result['execution_completed'],
                  localized_final_error_m=result.get('final_position_error'),
                  selected_execution_sim_seconds=result.get('selected_execution_sim_seconds'),
                  perception_mode=result['perception_mode'],
                  evaluation_only=True, truth_used_for_control=False)
    if robot and result['execution_completed']:
        report['world_final_goal_error_m'] = math.dist(robot[-1][:2], (0., 3.4))
    departure_goals = [g for g in result.get('navigation_goals',[])
                       if g['state']=='GOAL_AFTER_REMOVE']
    if departure_goals:
        start = departure_goals[0]['sim']
        end = next((e['sim_seconds'] for e in
                    (json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines())
                    if e['state']=='DONE'), math.inf)
        departing = [s for s in samples if start<=s['sim']<=end]
        plans = {s['plan']['stamp']:s['plan'] for s in departing
                 if s.get('plan',{}).get('stamp',-1)>=start}
        report['departure_navigation_goal_count'] = len(departure_goals)
        report['departure_only_goal_is_G'] = all(math.dist(g['goal'][:2],(0.,3.4))<1e-8
                                                 for g in departure_goals)
        report['departure_distinct_nav2_plans'] = len(plans)
        if plans:
            report['departure_first_plan_start_error_m'] = math.dist(
                min(plans.items())[1]['first'],departure_goals[0]['start_pose'][:2])
            report['departure_plan_endpoint_max_error_m'] = max(
                math.dist(p['last'],(0.,3.4)) for p in plans.values())
        report['continuous_departure_verified'] = (
            result['execution_completed'] and len(departure_goals)==1
            and report['departure_only_goal_is_G'] and len(plans)>2
            and report.get('departure_first_plan_start_error_m',math.inf)<.10
            and report.get('departure_plan_endpoint_max_error_m',math.inf)<.05)
    observation=result.get('camera_observation')
    paired=[d for d in samples if 'validation_only_can_world_pose' in d and 'validation_only_world_pose' in d]
    if observation and paired:
        stamp=observation['stamp']['sec']+observation['stamp']['nanosec']/1e9
        sample=min(paired,key=lambda d:abs(d['sim']-stamp))
        pose=sample['validation_only_world_pose']
        # Robot entity origin is base_footprint; the fixed base_joint is +10 mm Z.
        observed_world=Rotation.from_quat(pose[3:]).apply(
            np.asarray(observation['center_xyz'])+[0.,0.,.010])+pose[:3]
        report['camera_xy_error_m_approx']=math.dist(observed_world[:2],sample['validation_only_can_world_pose'][:2])
        report['camera_validation_sample_time_offset_s']=sample['sim']-stamp
    if can:
        initial_z = statistics.median(p[2] for p in can[:20])
        report.update(can_initial_world_xyz=can[0][:3], can_final_world_xyz=can[-1][:3],
                      maximum_lift_m=max(p[2] for p in can)-initial_z)
        if result['choice'] == 'REMOVE':
            arm_path=folder/'arm_remove_result.json'
            arm = json.loads(arm_path.read_text()) if arm_path.exists() else {}
            evidence = {e['check'] for e in arm.get('evidence',[])}
            # The generated green pad is x=[-.50,-.35], y=[.81,1.01].
            # Require the entire radius, upright support height, and release.
            x, y, z = can[-1][:3]
            on_pad = -.475 <= x <= -.375 and .835 <= y <= .985 and abs(z-.0695) < .01
            report.update(can_fully_on_pad=on_pad,
                remove_operation_called=arm_path.exists(),
                contact_sequence_confirmed={'pinch', 'hold', 'release'} <= evidence,
                physical_pick_place_verified=(arm.get('state') == 'SUCCEEDED' and on_pad
                    and report['maximum_lift_m'] > .025
                    and {'pinch', 'hold', 'release'} <= evidence),
                attachment_used=arm.get('attachment_used'), teleport_used=arm.get('teleport_used'))
        elif result['choice'] == 'DETOUR':
            report['can_displacement_xy_m'] = math.dist(can[0][:2], can[-1][:2])
            report['remove_operation_called'] = (folder/'arm_remove_result.json').exists()
    (folder/'validation.json').write_text(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    print(json.dumps(validate(parser.parse_args().run), indent=2))
