import math

import pytest

from applepic_mission.approach import ApproachParams, approach_command, gripper_error


def _simulate(x, y, yaw, tx, ty, params=ApproachParams(), dt=0.05, steps=2000):
    for _ in range(steps):
        lin, ang, reached = approach_command(x, y, yaw, tx, ty, params)
        if reached:
            return x, y, yaw, True
        x += lin * math.cos(yaw) * dt
        y += lin * math.sin(yaw) * dt
        yaw += ang * dt
    return x, y, yaw, False


@pytest.mark.parametrize('target', [(2.0, 0.0), (0.5, 1.5), (-1.0, -0.8), (0.3, 0.3), (0.6, 0.05)])
def test_gripper_converges_onto_target(target):
    p = ApproachParams()
    x, y, yaw, reached = _simulate(0.0, 0.0, 0.0, *target, p)
    assert reached
    assert gripper_error(x, y, yaw, *target, p.offset) < p.tolerance


def test_overshoot_backs_up():
    p = ApproachParams()
    # Target only 0.3 m ahead: the gripper (0.46 m ahead) is past it.
    lin, ang, reached = approach_command(0.0, 0.0, 0.0, 0.3, 0.0, p)
    assert not reached and lin < 0.0
