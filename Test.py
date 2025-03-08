import math
from wpimath.geometry import Rotation2d, Translation2d

ANGLE = Rotation2d.fromDegrees(-35)
MAGNITUDE = 4.5 * 0.0254

print(Translation2d(MAGNITUDE, ANGLE))