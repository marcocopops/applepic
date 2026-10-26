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


@pytest.mark.parametrize('target', [(2.0, 0.0), (0.5, 1.5), (-1.0, -0.8), (0.3, 0.3), (0.6, 0.05),
                                    (0.0, 0.3), (-0.3, -0.1)])
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


CASTER_X = 0.32   # m, front caster ahead of base_footprint (URDF caster_x)


@pytest.mark.parametrize('target', [(0.1, 0.3), (0.3, -0.25), (-0.25, 0.2), (0.0, -0.35)])
def test_target_beside_chassis_is_left_before_turning(target):
    # Turning in place toward an apple beside the base would run the front caster over it.
    p = ApproachParams()
    lin, ang, reached = approach_command(0.0, 0.0, 0.0, *target, p)
    assert not reached and ang == 0.0 and lin != 0.0
    x, y, yaw = 0.0, 0.0, 0.0
    closest = math.inf
    for _ in range(2000):
        lin, ang, reached = approach_command(x, y, yaw, *target, p)
        if reached:
            break
        x += lin * math.cos(yaw) * 0.05
        y += lin * math.sin(yaw) * 0.05
        yaw += ang * 0.05
        closest = min(closest, math.dist((x + CASTER_X * math.cos(yaw),
                                          y + CASTER_X * math.sin(yaw)), target))
    assert reached
    assert closest > 0.09   # caster (r 5 cm) never touches the apple (r 4 cm)
