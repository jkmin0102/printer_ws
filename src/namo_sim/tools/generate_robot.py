#!/usr/bin/env python3
"""Make a private Waffle Pi SDF with LaserScan origin exactly matching its URDF.

Source: installed ROBOTIS turtlebot3_gazebo Waffle Pi model, Apache-2.0.
SDF joint poses locate joint anchors; they do not translate child links. The
installed base_scan SDF link has zero pose and its sensor has (-.064, 0, .121).
The installed URDF's base_scan TF is (-.064, 0, .131) from base_footprint because
base_joint contributes .010 m. Correct the sensor's z by .010 m in our copy.
XY was already consistent and is deliberately preserved.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    source = Path(get_package_share_directory('turtlebot3_gazebo'))
    tree = ET.parse(source / 'models' / 'turtlebot3_waffle_pi' / 'model.sdf')
    sensor = tree.getroot().find('./model/link[@name="base_scan"]/sensor/pose')
    assert sensor is not None
    sensor.text = '-0.064 0 0.131 0 0 0'
    # gazebo_ros spawn_entity parses read_text() with lxml, which rejects an
    # XML encoding declaration in an already decoded Unicode string.
    tree.write(root / 'models' / 'waffle_pi.sdf', encoding='unicode', xml_declaration=False)
