"""2D localization: EKF on the map -> odom transform, fusing dead reckoning and GPS fixes.

Dead reckoning (`DeadReckoning`: wheel speed + gyro yaw rate; the wheels alone under-measure
in-place pivots by ~7 % because they slip) is smooth and locally accurate but drifts; GPS is
absolute but noisy. Rather than
filtering the robot pose, the filter estimates the (slowly varying) transform T = (tx, ty, theta)
from the odometry frame to the map (world) frame, as robot_localization's navsat setup or AMCL
do with map -> odom:
    map pose = T * odom pose.
Prediction: T is constant; odometry drift (proportional to the distance driven and the angle
turned) inflates its covariance. Update: a GPS fix measures the antenna position in the map,
h(T) = R(theta) * p_antenna_odom + t. Fixes failing a Mahalanobis gate are rejected.
The heading offset is initialised from a known start heading (robot docked at the depot) and is
then observable from the GPS track as the robot drives.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class LocalizationParams:
    gps_stddev: float = 0.05        # m, assumed per-fix horizontal error (if the fix has none)
    initial_yaw_stddev: float = 0.05   # rad, start heading uncertainty
    # Odometry drift, as the 1-sigma error accumulated over 1 m driven / 1 rad turned
    # (random walk: the variance grows linearly with distance and angle).
    drift_per_m: float = 0.02       # m
    yaw_drift_per_m: float = 0.02   # rad
    yaw_drift_per_rad: float = 0.02   # rad
    yaw_drift_per_s: float = 0.002  # rad over 1 s (gyro bias), so the heading never locks
    gate: float = 16.0              # Mahalanobis^2 threshold (2 dof), larger fixes are rejected
    max_rejections: int = 20        # consecutive rejected fixes before the filter re-opens
    reopen_stddev: float = 0.5      # m, translation std added when it re-opens
    reopen_yaw_stddev: float = 0.1  # rad
    antenna_x: float = 0.0          # m, GPS antenna ahead of base_footprint
    antenna_y: float = 0.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(offset, pose):
    """map pose of an odom-frame pose (x, y, yaw) given the offset (tx, ty, theta)."""
    tx, ty, th = offset
    x, y, yaw = pose
    c, s = math.cos(th), math.sin(th)
    return tx + c * x - s * y, ty + s * x + c * y, wrap(th + yaw)


def map_shift(old, new):
    """Function moving a map point (x, y) of the map -> odom estimate `old` to `new`.

    The point keeps its odom-frame position: used to keep odometry-anchored things (apple tracks)
    consistent with the robot when the GPS-driven estimate moves the map.
    """
    tx, ty, th = old
    c, s = math.cos(th), math.sin(th)

    def fn(p):
        dx, dy = p[0] - tx, p[1] - ty
        return compose(new, (c * dx + s * dy, -s * dx + c * dy, 0.0))[:2]
    return fn


class OffsetEkf:

    def __init__(self, params=None):
        self.p = params or LocalizationParams()
        self.x = None              # (tx, ty, theta)
        self.P = None
        self.last_odom = None
        self.last_t = None
        self.accepted = 0
        self.rejected = 0
        self.consecutive_rejections = 0
        self.reopened = 0

    @property
    def initialized(self):
        return self.x is not None

    def _antenna(self, odom_pose):
        x, y, yaw = odom_pose
        c, s = math.cos(yaw), math.sin(yaw)
        return (x + c * self.p.antenna_x - s * self.p.antenna_y,
                y + s * self.p.antenna_x + c * self.p.antenna_y)

    def initialize(self, gps_xy, odom_pose, map_yaw, gps_stddev=None):
        """First fix: theta from the known map heading, translation from the fix."""
        th = wrap(map_yaw - odom_pose[2])
        ax, ay = self._antenna(odom_pose)
        c, s = math.cos(th), math.sin(th)
        self.x = np.array([gps_xy[0] - (c * ax - s * ay), gps_xy[1] - (s * ax + c * ay), th])
        sg = gps_stddev if gps_stddev is not None else self.p.gps_stddev
        # Translation uncertainty: the fix plus the lever effect of the heading uncertainty.
        lever = math.hypot(ax, ay) * self.p.initial_yaw_stddev
        self.P = np.diag([sg ** 2 + lever ** 2, sg ** 2 + lever ** 2,
                          self.p.initial_yaw_stddev ** 2])
        self.last_odom = odom_pose

    def predict(self, odom_pose, t=None):
        """Inflate the covariance with the odometry drift since the previous odom pose / time."""
        if not self.initialized:
            return
        lx, ly, lyaw = self.last_odom
        dist = math.hypot(odom_pose[0] - lx, odom_pose[1] - ly)
        turn = abs(wrap(odom_pose[2] - lyaw))
        dt = 0.0 if t is None or self.last_t is None else max(t - self.last_t, 0.0)
        self.last_odom = odom_pose
        if t is not None:
            self.last_t = t
        # Random-walk drift: the variance grows linearly with the distance driven / angle turned.
        q_xy = self.p.drift_per_m ** 2 * dist
        q_th = (self.p.yaw_drift_per_m ** 2 * dist + self.p.yaw_drift_per_rad ** 2 * turn
                + self.p.yaw_drift_per_s ** 2 * dt)
        if q_xy == 0.0 and q_th == 0.0:
            return
        # A heading error appearing at the robot rotates the map about the robot, not about the
        # odom origin: the offset translation moves with it (G = d offset / d heading drift).
        tx, ty, th = self.x
        x, y, _ = compose(self.x, odom_pose)
        G = np.array([y - ty, -(x - tx), 1.0])
        self.P = self.P + np.diag([q_xy, q_xy, 0.0]) + q_th * np.outer(G, G)

    def update(self, gps_xy, odom_pose, gps_stddev=None):
        """Fuse one GPS fix (map x, y) taken at odom pose `odom_pose`. True if accepted.

        After `max_rejections` consecutive rejections the estimate is assumed lost (e.g. an
        unmodelled odometry error made the filter over-confident): the covariance is inflated so
        that the fixes pull it back instead of being rejected forever.
        """
        if self._update(gps_xy, odom_pose, gps_stddev):
            self.consecutive_rejections = 0
            return True
        self.consecutive_rejections += 1
        if self.consecutive_rejections < self.p.max_rejections:
            return False
        self.reopen(odom_pose)
        self.consecutive_rejections = 0
        return self._update(gps_xy, odom_pose, gps_stddev)

    def reopen(self, odom_pose):
        """Inflate the covariance (robot position and heading uncertain again)."""
        tx, ty, _ = self.x
        x, y, _ = compose(self.x, odom_pose)
        G = np.array([y - ty, -(x - tx), 1.0])   # heading uncertainty about the robot
        self.P = (self.P + np.diag([self.p.reopen_stddev ** 2] * 2 + [0.0])
                  + self.p.reopen_yaw_stddev ** 2 * np.outer(G, G))
        self.reopened += 1

    def _update(self, gps_xy, odom_pose, gps_stddev):
        tx, ty, th = self.x
        ax, ay = self._antenna(odom_pose)
        c, s = math.cos(th), math.sin(th)
        z_hat = np.array([tx + c * ax - s * ay, ty + s * ax + c * ay])
        H = np.array([[1.0, 0.0, -s * ax - c * ay],
                      [0.0, 1.0, c * ax - s * ay]])
        sg = gps_stddev if gps_stddev is not None else self.p.gps_stddev
        R = np.eye(2) * sg ** 2
        innov = np.asarray(gps_xy, dtype=float) - z_hat
        S = H @ self.P @ H.T + R
        if float(innov @ np.linalg.solve(S, innov)) > self.p.gate:
            self.rejected += 1
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.x[2] = wrap(self.x[2])
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T   # Joseph form, stays symmetric PSD
        self.accepted += 1
        return True

    def pose(self, odom_pose):
        """Estimated map pose (x, y, yaw) of the robot at odom pose `odom_pose`."""
        return compose(self.x, odom_pose)

    def position_stddev(self, odom_pose):
        """1-sigma horizontal position uncertainty (m) of the robot at `odom_pose`."""
        tx, ty, _ = self.x
        x, y, _ = compose(self.x, odom_pose)
        J = np.array([[1.0, 0.0, -(y - ty)], [0.0, 1.0, x - tx]])
        return math.sqrt(max(np.trace(J @ self.P @ J.T) / 2.0, 0.0))


class DeadReckoning:
    """Odometry pose (x, y, yaw) from the wheel forward speed and the gyro yaw rate."""

    def __init__(self):
        self.pose = (0.0, 0.0, 0.0)
        self.v = 0.0
        self.t = None

    def set_speed(self, v):
        self.v = v

    def integrate(self, t, yaw_rate):
        """Advance to time t with the current speed and `yaw_rate` (midpoint heading)."""
        if self.t is None or t <= self.t:
            self.t = t if self.t is None else self.t
            return self.pose
        dt = min(t - self.t, 0.2)   # a gap in the IMU stream must not teleport the robot
        self.t = t
        x, y, yaw = self.pose
        mid = yaw + 0.5 * yaw_rate * dt
        self.pose = (x + self.v * dt * math.cos(mid), y + self.v * dt * math.sin(mid),
                     wrap(yaw + yaw_rate * dt))
        return self.pose


class PoseErrorTracker:
    """Error of a pose estimate against the ground truth (evaluation only).

    Truth poses are interpolated at each estimate stamp; summary() gives position error
    statistics (m) and the heading error (rad).
    """

    def __init__(self, max_truth=200):
        self.truth = []
        self.max_truth = max_truth
        self.errors = []      # (t, position error, |heading error|)
        self.pending = []     # estimates newer than the latest truth sample

    def add_truth(self, t, pose):
        self.truth.append((t, pose))
        if len(self.truth) > self.max_truth:
            del self.truth[:len(self.truth) - self.max_truth]
        ready = [e for e in self.pending if e[0] <= t]
        self.pending = [e for e in self.pending if e[0] > t][-self.max_truth:]
        for te, pe in ready:
            self._record(te, pe)

    def add_estimate(self, t, pose):
        """Record the error of `pose` at time t (deferred if newer than the truth)."""
        if self.truth and t > self.truth[-1][0]:
            self.pending.append((t, pose))
            return True
        return self._record(t, pose)

    def _record(self, t, pose):
        for (t0, p0), (t1, p1) in zip(reversed(self.truth[:-1]), reversed(self.truth[1:])):
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                x = p0[0] + a * (p1[0] - p0[0])
                y = p0[1] + a * (p1[1] - p0[1])
                yaw = p0[2] + a * wrap(p1[2] - p0[2])
                self.errors.append((t, math.hypot(pose[0] - x, pose[1] - y),
                                    abs(wrap(pose[2] - yaw))))
                return True
        return False

    def summary(self):
        if not self.errors:
            return {'samples': 0}
        pos = sorted(e[1] for e in self.errors)
        head = sorted(e[2] for e in self.errors)
        n = len(pos)
        return {
            'samples': n,
            'position_error_mean_m': round(sum(pos) / n, 4),
            'position_error_p95_m': round(pos[min(n - 1, int(0.95 * n))], 4),
            'position_error_max_m': round(pos[-1], 4),
            'heading_error_mean_rad': round(sum(head) / n, 4),
            'heading_error_max_rad': round(head[-1], 4),
        }
