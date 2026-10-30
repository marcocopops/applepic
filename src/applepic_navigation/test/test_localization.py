import math
import random

from applepic_navigation.gps import enu_to_geodetic, geodetic_to_enu, GpsErrorModel
from applepic_navigation.localization import compose, LocalizationParams, OffsetEkf, wrap

LAT0, LON0 = 49.18, -0.37


def test_enu_round_trip_and_scale():
    lat, lon = enu_to_geodetic(16.0, 10.0, LAT0, LON0)
    e, n = geodetic_to_enu(lat, lon, LAT0, LON0)
    assert abs(e - 16.0) < 1e-6 and abs(n - 10.0) < 1e-6
    # One degree of latitude is ~111.2 km at 49 deg N; a degree of longitude ~cos(lat) of that.
    assert abs(geodetic_to_enu(LAT0 + 1e-3, LON0, LAT0, LON0)[1] - 111.2) < 0.2
    assert abs(geodetic_to_enu(LAT0, LON0 + 1e-3, LAT0, LON0)[0] - 72.9) < 0.2


def test_gps_error_model_statistics():
    white = GpsErrorModel(stddev=0.5, seed=1)
    errs = [white.sample(0.1) for _ in range(4000)]
    sd = math.sqrt(sum(e * e for e, _ in errs) / len(errs))
    assert 0.45 < sd < 0.55
    # A biased model with a long correlation time: consecutive errors are strongly correlated.
    biased = GpsErrorModel(stddev=0.0, bias_stddev=0.3, tau=60.0, seed=2)
    a = biased.sample(0.1)
    b = biased.sample(0.1)
    assert math.hypot(a[0] - b[0], a[1] - b[1]) < 0.02
    # Same seed -> same sequence (reproducible e2e runs).
    assert GpsErrorModel(0.1, 0.2, 30, seed=3).sample(0.1) == GpsErrorModel(0.1, 0.2, 30,
                                                                           seed=3).sample(0.1)


def _drive(offset_true, gps, ekf, odom_scale=1.0, odom_yaw_rate_bias=0.0, steps=3000,
           init_yaw_error=0.0, seed=0):
    """Unicycle on a square-ish loop; odometry with scale error and heading drift.

    Returns the max and the mean position error of the estimate over the second half.
    """
    rng = random.Random(seed)
    dt = 0.05
    true = list(compose(offset_true, (0.0, 0.0, 0.0)))
    odom = [0.0, 0.0, 0.0]
    errs = []
    for k in range(steps):
        v = 0.5
        w = 0.4 if (k // 200) % 2 else 0.0
        true[0] += v * dt * math.cos(true[2])
        true[1] += v * dt * math.sin(true[2])
        true[2] = wrap(true[2] + w * dt)
        vo = v * odom_scale
        odom[0] += vo * dt * math.cos(odom[2])
        odom[1] += vo * dt * math.sin(odom[2])
        odom[2] = wrap(odom[2] + w * dt + odom_yaw_rate_bias * dt * rng.random())
        pose = tuple(odom)
        if k % 2 == 0:   # 10 Hz GPS
            e, n = gps.sample(0.1)
            fix = (true[0] + e, true[1] + n)
            if not ekf.initialized:
                ekf.initialize(fix, pose, true[2] + init_yaw_error)
            else:
                ekf.update(fix, pose)
        else:
            ekf.predict(pose)
        if ekf.initialized and k > steps // 2:
            x, y, _ = ekf.pose(pose)
            errs.append(math.hypot(x - true[0], y - true[1]))
    return max(errs), sum(errs) / len(errs)


def test_ekf_follows_drifting_odometry_with_noisy_gps():
    ekf = OffsetEkf(LocalizationParams(gps_stddev=0.3))
    worst, mean = _drive((3.0, -2.0, 0.6), GpsErrorModel(stddev=0.3, seed=4), ekf,
                         odom_scale=1.03, odom_yaw_rate_bias=0.02, init_yaw_error=0.05)
    # Raw odometry alone would be metres off after 75 m; raw fixes are 0.3 m (1 sigma) noisy.
    assert mean < 0.15 and worst < 0.4
    assert ekf.rejected < 0.02 * ekf.accepted


def test_ekf_recovers_initial_heading_error():
    ekf = OffsetEkf(LocalizationParams(gps_stddev=0.05, initial_yaw_stddev=0.2))
    worst, mean = _drive((0.0, 0.0, 0.0), GpsErrorModel(stddev=0.05, seed=5), ekf,
                         init_yaw_error=0.2)
    assert mean < 0.05 and worst < 0.12
    assert abs(wrap(ekf.x[2])) < 0.03


def test_ekf_rejects_outlier_fix():
    ekf = OffsetEkf(LocalizationParams(gps_stddev=0.05))
    ekf.initialize((1.0, 1.0), (0.0, 0.0, 0.0), 0.0)
    for _ in range(20):
        assert ekf.update((1.0, 1.0), (0.0, 0.0, 0.0))
    assert not ekf.update((6.0, 1.0), (0.0, 0.0, 0.0))
    x, y, _ = ekf.pose((0.0, 0.0, 0.0))
    assert math.hypot(x - 1.0, y - 1.0) < 0.01
    assert ekf.position_stddev((0.0, 0.0, 0.0)) < 0.05


def test_ekf_antenna_lever_arm():
    p = LocalizationParams(gps_stddev=0.02, antenna_x=0.3)
    ekf = OffsetEkf(p)
    # Robot at the map origin facing +y: the antenna is at (0, 0.3).
    ekf.initialize((0.0, 0.3), (0.0, 0.0, 0.0), math.pi / 2)
    x, y, yaw = ekf.pose((0.0, 0.0, 0.0))
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9


def test_pose_error_tracker_interpolates_truth():
    from applepic_navigation.localization import PoseErrorTracker
    tr = PoseErrorTracker()
    assert tr.summary() == {'samples': 0}
    tr.add_truth(0.0, (0.0, 0.0, 0.0))
    tr.add_truth(1.0, (1.0, 0.0, 0.2))
    assert tr.add_estimate(0.5, (0.5, 0.1, 0.1))
    assert not tr.add_estimate(-1.0, (0.0, 0.0, 0.0))
    assert tr.summary()['samples'] == 1
    tr.add_estimate(1.5, (1.5, 0.1, 0.2))     # newer than the truth: deferred
    assert tr.summary()['samples'] == 1
    tr.add_truth(2.0, (2.0, 0.0, 0.2))
    s = tr.summary()
    assert s['samples'] == 2
    assert abs(s['position_error_mean_m'] - 0.1) < 1e-9
    assert s['heading_error_max_rad'] < 1e-9
