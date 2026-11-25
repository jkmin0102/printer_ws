"""Human-readable status output for headless runs, using the same RViz snapshot."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String

class StatusDisplay(Node):
    def __init__(self):
        super().__init__('namo_status_display');self.last=None;self.printed_choice=None
        self.create_subscription(String,'/namo/status',self.callback,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def callback(self,msg):
        try:d=json.loads(msg.data)
        except ValueError:return
        key=(d.get('state'),d.get('arm_state'),d.get('arm_stage'),d.get('failure'),d.get('navigation_notice'))
        if key==self.last:return
        self.last=key
        choice={'REMOVE':'치우고 직진','DETOUR':'우회','CLEAR':'장애물 없이 주행'}.get(d.get('choice'),'판단 전')
        print(f"[NAMO] {d.get('label')} | 선택={choice} | 팔={d.get('arm_state_label')}/{d.get('arm_stage_label')} | 인식={d.get('perception_mode')}",flush=True)
        if d.get('choice') in ('REMOVE','DETOUR') and d.get('choice')!=self.printed_choice:
            self.printed_choice=d['choice']
            metres=lambda value:f'{value:.3f} m' if isinstance(value,(int,float)) else '후보 없음'
            print(f"  정지 위치(map): {d.get('stop_pose')} | LiDAR 관측 표면(map): {d.get('observed_surface')}",flush=True)
            print(f"  후보 길이: 제거 {metres(d.get('L_clear'))}, 우회 {metres(d.get('L_detour'))} | 공통 유효 속도: {d.get('v_eff')} m/s",flush=True)
            fmt=lambda value:f'{value:.1f}초' if isinstance(value,(int,float)) else '후보 없음'
            print(f"  판단 당시 예상: 제거 후 이동 {fmt(d.get('T_remove'))}, 우회 {fmt(d.get('T_detour'))}\n  이유: {d.get('reason')}",flush=True)
        if d.get('failure'):print('  중단 원인: '+d.get('failure_label',d['failure']),flush=True)
        if d.get('failure'):print('  마지막 카메라 상태: '+d.get('camera_reason','관측 수신 없음'),flush=True)
        if d.get('navigation_notice'):print('  주행 진단: '+d['navigation_notice'],flush=True)

def main(args=None):
    rclpy.init(args=args);n=StatusDisplay()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
