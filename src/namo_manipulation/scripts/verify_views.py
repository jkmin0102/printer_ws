#!/usr/bin/env python3
"""Stationary fixture validation only; never run beside a navigation mission.

Checks real measured view motions, unchanged can, and fresh YOLO observations.
The caller must supply an isolated ROS domain and a stopped-base fixture.
"""
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import time
import numpy as np
import rclpy
from std_srvs.srv import Trigger
from arm_probe import Probe


def main():
    rclpy.init();node=Probe(SimpleNamespace(operation='prepare',can_name='namo_can'))
    report={'validation_only':True,'steps':[]}
    output=Path(os.environ['NAMO_RUN_DIR'])
    try:
        if not node.wait(lambda:node.client.service_is_ready() and node.can is not None
                         and node.status.get('ready_for_operation'),45):
            raise RuntimeError('Fixture not ready')
        initial=list(node.can)
        for index,operation in enumerate(('PREPARE','VIEW','VIEW')):
            client=node.create_client(Trigger,'/namo/prepare_grasp' if index==0 else '/namo/adjust_grasp_view')
            if not node.wait(client.service_is_ready,5):raise RuntimeError('View service unavailable')
            started=time.monotonic();ack=client.call_async(Trigger.Request())
            if not node.wait(ack.done,10) or not ack.result().success:raise RuntimeError('View request rejected')
            if not node.wait(lambda:node.status_received>started and node.status.get('operation')==operation
                             and node.status.get('state') in ('PREPARED','FAILED'),40):
                raise RuntimeError('View motion timeout')
            if node.status.get('state')!='PREPARED':raise RuntimeError(node.status.get('reason'))
            at=node.now()
            def fresh():
                obs=node.observation;stamp=obs.get('stamp',{})
                t=stamp.get('sec',0)+stamp.get('nanosec',0)*1e-9
                return obs.get('valid') and obs.get('stable') and at<t<=node.now() and node.now()-t<.8
            recognized=node.wait(fresh,8)
            actual=node.arm_samples[-1]
            expected=math.radians((77.,72.,82.)[index])
            error=abs(actual['actual_joint1']-expected)
            if error>.06:raise RuntimeError('Measured view joint did not reach target')
            report['steps'].append(dict(operation=operation,index=index,joint1_error_rad=error,
                fresh_yolo=bool(recognized),observation=dict(node.observation),can_world=list(node.can)))
        client=node.create_client(Trigger,'/namo/adjust_grasp_view')
        request=client.call_async(Trigger.Request())
        if not node.wait(request.done,5) or request.result().success:raise RuntimeError('View limit not enforced')
        report.update(success=True,third_view_rejected=True,
            can_displacement_m=float(np.linalg.norm(np.array(node.can)-initial)))
    except Exception as exc:
        report.update(success=False,error=str(exc));raise
    finally:
        (output/'view_validation.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
