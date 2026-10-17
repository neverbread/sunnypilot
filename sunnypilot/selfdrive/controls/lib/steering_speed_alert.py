import math


class SteeringSpeedAlert:
  """One notification per downward crossing, rearmed above a hysteresis band."""

  def __init__(self, threshold: float, hysteresis: float):
    self.threshold = threshold
    self.hysteresis = hysteresis
    self.armed = False
    self.initialized = False

  def update(self, speed: float, enabled: bool) -> bool:
    if not enabled or not math.isfinite(speed) or speed < 0:
      self.armed = False
      self.initialized = False
      return False

    if not self.initialized:
      self.initialized = True
      self.armed = speed > self.threshold
      return False

    if speed >= self.threshold + self.hysteresis:
      self.armed = True
    elif self.armed and speed <= self.threshold:
      self.armed = False
      return True

    return False
