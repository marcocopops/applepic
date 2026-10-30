"""Sanity checks of the robot description (xacro -> URDF), no ROS graph needed."""
import os
import xml.etree.ElementTree as ET

import pytest

xacro = pytest.importorskip('xacro')

URDF = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'applepic.urdf.xacro')


@pytest.fixture(scope='module')
def robot():
    return ET.fromstring(xacro.process_file(URDF).toxml())


def _joint_xyz(robot, name):
    joint = robot.find(f"joint[@name='{name}']")
    return [float(v) for v in joint.find('origin').get('xyz').split()]


def test_expected_frames(robot):
    links = {link.get('name') for link in robot.findall('link')}
    assert {'base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link',
            'caster_link', 'camera_link', 'camera_optical_frame', 'gripper_link',
            'basket_link'} <= links


def test_chassis_footprint(robot):
    box = robot.find("link[@name='base_link']/collision/geometry/box")
    length, width, _ = (float(v) for v in box.get('size').split())
    assert length == pytest.approx(0.60)
    assert width == pytest.approx(0.45)


def test_wheels_touch_ground_on_rotation_axis(robot):
    base_z = _joint_xyz(robot, 'base_joint')[2]
    radius = float(robot.find("link[@name='left_wheel_link']/collision/geometry/cylinder")
                   .get('radius'))
    for side, sign in (('left', 1), ('right', -1)):
        x, y, z = _joint_xyz(robot, f'{side}_wheel_joint')
        assert x == pytest.approx(0.0)
        assert sign * y > 0
        assert base_z + z - radius == pytest.approx(0.0, abs=1e-9)


def test_chassis_clears_apples(robot):
    # Apples are 8 cm tall: a lower chassis pushes (and sometimes launches) them.
    base_z = _joint_xyz(robot, 'base_joint')[2]
    box = robot.find("link[@name='base_link']/collision/geometry/box")
    height = float(box.get('size').split()[2])
    assert base_z - height / 2 > 0.08 + 0.01


def test_gripper_in_front_and_low(robot):
    x, _, z = _joint_xyz(robot, 'gripper_joint')
    chassis_front = 0.10 + 0.30
    assert x > chassis_front
    assert 0.0 < z < 0.08


def test_gz_plugins_present(robot):
    names = {p.get('name') for p in robot.iter('plugin')}
    assert 'gz::sim::systems::DiffDrive' in names
    assert 'gz::sim::systems::OdometryPublisher' in names
    sensor = robot.find(".//sensor[@type='camera']")
    assert sensor.find('topic').text == 'camera/image_raw'


def test_bumper_covers_the_wheels_but_not_their_ground_contact(robot):
    sensors = [s.get('name') for s in robot.iter('sensor') if s.get('type') == 'contact']
    assert sensors == ['chassis_bumper']
    strip = robot.find("link[@name='base_link']/collision[@name='bumper_strip']")
    width = float(strip.find('geometry/box').get('size').split()[1])
    assert width > 0.52 + 0.05
