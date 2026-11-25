"""Conservative adjacent-ray geometry for costmap CLEARING ONLY.

These are derived rays, not additional measurements. Raw /scan remains the
sole obstacle marking and NAMO detection input. Shorter neighbor minus 2 cm
is used only across a continuous surface or two no-return (+inf) beams.
"""
import math


def conservative_ranges(ranges, range_min, range_max, factor=4, max_jump=.15, margin=.02):
    if not isinstance(factor, int) or factor<1 or not (0<=range_min<range_max):
        raise ValueError('Invalid resampling geometry')
    if not ranges:
        return []
    def cap(value):
        if value==math.inf:
            return range_max
        return value if math.isfinite(value) and range_min<=value<=range_max else math.nan
    def shorten(value):
        return value-margin if math.isfinite(value) and value-margin>=range_min else math.nan
    output=[]
    for i,a in enumerate(ranges):
        av=cap(a)
        output.append(shorten(av))
        if i==len(ranges)-1:
            break
        b=ranges[i+1];bv=cap(b)
        # Do not bridge depth edges, invalid beams, or finite/no-return transitions.
        continuous=(math.isfinite(av) and math.isfinite(bv)
                    and math.isinf(a)==math.isinf(b) and abs(av-bv)<=max_jump)
        value=shorten(min(av,bv)) if continuous else math.nan
        output.extend([value]*(factor-1))
    return output


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class ScanClearing(Node):
        def __init__(self):
            super().__init__('scan_clearing')
            self.pub=self.create_publisher(LaserScan,'/scan_clearing',qos_profile_sensor_data)
            self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)

        def scan(self,msg):
            factor=4
            result=LaserScan()
            result.header=msg.header
            result.angle_min=msg.angle_min
            result.angle_increment=msg.angle_increment/factor
            result.time_increment=msg.time_increment/factor
            result.scan_time=msg.scan_time
            result.range_min=msg.range_min
            result.range_max=msg.range_max
            result.ranges=conservative_ranges(msg.ranges,msg.range_min,msg.range_max,factor)
            result.angle_max=result.angle_min+max(0,len(result.ranges)-1)*result.angle_increment
            self.pub.publish(result)
    rclpy.init(args=args)
    node=ScanClearing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
