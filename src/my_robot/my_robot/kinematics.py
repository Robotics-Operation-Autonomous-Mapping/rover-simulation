"""Pure geometry helpers for a simulated GNSS antenna velocity."""

import math


def antenna_position(position, quaternion, offset):
    """Rotate a body-fixed lever arm into world and add the body position."""
    x, y, z, w = quaternion
    vx, vy, vz = offset
    tx, ty, tz = 2 * (y*vz-z*vy), 2 * (z*vx-x*vz), 2 * (x*vy-y*vx)
    return (position[0] + vx + w*tx + y*tz-z*ty,
            position[1] + vy + w*ty + z*tx-x*tz,
            position[2] + vz + w*tz + x*ty-y*tx)


class VelocitySampler:
    """Sample interval-average world velocity using simulation nanoseconds."""

    def __init__(self, rate):
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('Velocity sample rate must be finite and positive')
        self.period_ns = round(1e9 / rate)
        if self.period_ns < 1:
            raise ValueError('Velocity sample period must be at least one nanosecond')
        self.previous = None

    def sample(self, stamp_ns, position):
        previous = self.previous
        if previous is None or stamp_ns < previous[0]:
            self.previous = (stamp_ns, position)
            return None
        elapsed = stamp_ns - previous[0]
        if elapsed < self.period_ns:
            return None
        self.previous = (stamp_ns, position)
        return tuple((value - previous[1][i]) / (elapsed * 1e-9) for i, value in enumerate(position))
