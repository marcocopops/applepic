import math

import pytest

from applepic_mission.basket import Basket
from applepic_mission.picking import apple_in_reach, gripper_position


def test_basket_capacity_and_unload():
    b = Basket(capacity=2)
    assert b.add() and b.add()
    assert b.full
    assert not b.add()
    assert b.count == 2 and b.total_picked == 2
    assert b.unload() == 2
    assert b.count == 0 and not b.full and b.total_unloaded == 2
    assert b.add() and b.total_picked == 3


def test_basket_rejects_bad_capacity():
    with pytest.raises(ValueError):
        Basket(capacity=0)


def test_gripper_position_follows_heading():
    gx, gy = gripper_position(1.0, 2.0, math.pi / 2, 0.46)
    assert gx == pytest.approx(1.0)
    assert gy == pytest.approx(2.46)


def test_apple_in_reach_picks_closest_within_radius():
    apples = {'a': (1.05, 0.0), 'b': (1.02, 0.01), 'c': (3.0, 0.0)}
    assert apple_in_reach((1.0, 0.0), apples, 0.07) == 'b'
    assert apple_in_reach((2.0, 0.0), apples, 0.07) is None
    assert apple_in_reach((0.0, 0.0), {}, 0.07) is None
