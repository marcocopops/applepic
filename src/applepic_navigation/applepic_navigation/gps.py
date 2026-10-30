"""GPS helpers: geodetic <-> local ENU (world frame) conversion and a GPS error model.

The world frame is ENU with its origin at the datum (`spherical_coordinates` of the orchard
config), as in the gz world. Over an orchard (tens of metres) a local tangent plane with the
WGS84 curvature radii at the datum is accurate to well under a millimetre.
"""
import math
import random

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def _radii(lat0_deg):
    """(meridional, prime vertical) radii of curvature in metres at latitude lat0_deg."""
    s = math.sin(math.radians(lat0_deg))
    d = 1.0 - WGS84_E2 * s * s
    return WGS84_A * (1.0 - WGS84_E2) / d ** 1.5, WGS84_A / math.sqrt(d)


def geodetic_to_enu(lat_deg, lon_deg, lat0_deg, lon0_deg):
    """(east, north) in metres of (lat, lon) relative to the datum (lat0, lon0)."""
    m, n = _radii(lat0_deg)
    north = math.radians(lat_deg - lat0_deg) * m
    east = math.radians(lon_deg - lon0_deg) * n * math.cos(math.radians(lat0_deg))
    return east, north


def enu_to_geodetic(east, north, lat0_deg, lon0_deg):
    """Inverse of geodetic_to_enu."""
    m, n = _radii(lat0_deg)
    lat = lat0_deg + math.degrees(north / m)
    lon = lon0_deg + math.degrees(east / (n * math.cos(math.radians(lat0_deg))))
    return lat, lon


class GpsErrorModel:
    """Horizontal GPS error: white noise + a slowly wandering bias, isotropic, in metres.

    The bias is a first-order Gauss-Markov process (std `bias_stddev`, correlation time `tau`),
    a common model for multipath / atmospheric errors that a filter cannot average out.
    """

    def __init__(self, stddev=0.02, bias_stddev=0.0, tau=60.0, seed=None):
        self.stddev = stddev
        self.bias_stddev = bias_stddev
        self.tau = max(tau, 1e-6)
        self.rng = random.Random(seed)
        self.bias = [self.rng.gauss(0.0, bias_stddev), self.rng.gauss(0.0, bias_stddev)]

    def sample(self, dt):
        """(east, north) error for a fix taken `dt` seconds after the previous one."""
        a = math.exp(-max(dt, 0.0) / self.tau)
        b = self.bias_stddev * math.sqrt(1.0 - a * a)
        self.bias = [a * v + b * self.rng.gauss(0.0, 1.0) for v in self.bias]
        return tuple(v + self.rng.gauss(0.0, self.stddev) for v in self.bias)
